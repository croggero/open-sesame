// src/door_controller.rs
//
// Core assistive control loop for the sliding door.
//
// State machine:
//
//   Commands (open/close)
//   ┌─────────────────────────────────────────────────────────┐
//   │                                                         │
//   ▼                                                         │
//   ┌─────────┐  encoder moves > threshold  ┌──────────────┐ │
//   │  Idle   │ ─────────────────────────── │ Assist       │ │
//   │ (sleep) │ ◄──── timeout elapsed ───── │ (push/pull)  │ │
//   └─────────┘                             └──────┬───────┘ │
//        ▲                                         │         │
//        │ stop / endstop                          │ endstop │
//        │                                         ▼         │
//        └───────────────────────────────── ┌──────────────┐ │
//                                           │ Open/Closed  │─┘
//                                           └──────────────┘
//
// Remote drive (open/close command) overrides assist when a target is set.
// Endstops always take priority and clear the door target.
//
// The control loop runs at ~10 ms intervals on the calling thread.
// The encoder ISR updates position atomically from a separate context.

use crate::config;
use crate::encoder::Encoder;
use crate::motor::MotorDriver;
use anyhow::{Ok, Result};
use log::info;
use std::time::{Duration, Instant};

// All tuning constants live in config.toml and are exposed via crate::config.
use config::{
    ASSIST_LOCKOUT_TICKS, ASSIST_VELOCITY_THRESHOLD, CALIBRATE_POWER, CALIBRATION_PAUSE_TICKS,
    CALIBRATION_STALL_TICKS, CALIBRATION_TIMEOUT_TICKS, CLOSED_THRESHOLD, CLOSE_HOMING, DEAD_ZONE,
    IDLE_TIMEOUT_MS, INVERT_DIRECTION, LOOP_PERIOD_MS, LPF_ALPHA, OPERATING_POWER,
    RAMP_DOWN_COUNTS, SETTLE_TICKS, STALL_GRACE_TICKS, STALL_TICKS,
};

const IDLE_TIMEOUT: Duration = Duration::from_millis(IDLE_TIMEOUT_MS);
const LOOP_PERIOD: Duration = Duration::from_millis(LOOP_PERIOD_MS);

/// Returns 1 or -1 based on INVERT_DIRECTION.
const fn dir() -> i32 {
    if INVERT_DIRECTION {
        -1
    } else {
        1
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// DoorState
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq)]
pub enum DoorState {
    Idle,
    Stopping,
    Opening,
    Closing,
    Open,
    Closed,
    /// Boot-up homing: drive toward closed endstop, reset encoder, done.
    Homing {
        stall_count: u32,
        timeout_ticks: u32,
    },
    Calibrating,
}

impl DoorState {
    pub fn as_str(&self) -> &'static str {
        match self {
            DoorState::Idle => "idle",
            DoorState::Stopping => "stopping",
            DoorState::Opening => "opening",
            DoorState::Closing => "closing",
            DoorState::Open => "open",
            DoorState::Closed => "closed",
            DoorState::Homing {
                stall_count: _,
                timeout_ticks: _,
            } => "homing",
            DoorState::Calibrating => "calibrating",
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CalibrationStep
// ─────────────────────────────────────────────────────────────────────────────

/// Tracks progress through the non-blocking calibration sequence.
enum CalibrationStep {
    /// Driving toward the closed endstop; counting consecutive no-motion ticks.
    DrivingClosed {
        stall_count: u32,
        timeout_ticks: u32,
    },
    /// Brief pause between directions (ticks remaining).
    Pause { remaining: u32 },
    /// Driving toward the open endstop; counting consecutive no-motion ticks.
    DrivingOpen {
        stall_count: u32,
        timeout_ticks: u32,
    },
}

// ─────────────────────────────────────────────────────────────────────────────
// DoorController
// ─────────────────────────────────────────────────────────────────────────────

pub struct DoorController<'d, 'e> {
    motor: MotorDriver<'d>,
    encoder: Encoder<'e>,
    state: DoorState,
    last_pos: i32,
    velocity: f32,
    last_move_time: Instant,
    target: Option<i32>,
    opening_counts: i32,
    /// Active calibration step, or None when not calibrating.
    calibration: Option<CalibrationStep>,
    /// Consecutive no-motion ticks while motor is powered (obstacle detection).
    obstacle_stall_count: u32,
    /// Ticks since the current target was set. Stall detection is suppressed
    /// until this exceeds STALL_GRACE_TICKS.
    drive_ticks: u32,
    /// Ticks remaining before assist re-arms. Non-zero after an obstacle stop.
    assist_lockout: u32,
    /// Consecutive stationary ticks accumulated in the Stopping state.
    settle_count: u32,
}

impl<'d, 'e> DoorController<'d, 'e> {
    pub fn new(motor: MotorDriver<'d>, encoder: Encoder<'e>) -> Self {
        Self {
            motor,
            encoder,
            state: DoorState::Idle,
            last_pos: 0,
            velocity: 0.0,
            last_move_time: Instant::now(),
            target: None,
            opening_counts: 0,
            calibration: None,
            obstacle_stall_count: 0,
            drive_ticks: 0,
            assist_lockout: 0,
            settle_count: 0,
        }
    }

    /// Run one tick of the control loop. Call this every ~LOOP_PERIOD.
    pub fn tick(&mut self) -> Result<()> {
        let current_pos = self.encoder.read();
        let raw_velocity = (current_pos - self.last_pos) as f32;

        self.velocity = LPF_ALPHA * self.velocity + (1.0 - LPF_ALPHA) * raw_velocity;
        self.assist_lockout = self.assist_lockout.saturating_sub(1);

        let prev_state = self.state.clone();
        if self.calibration.is_some() {
            self.tick_calibration(current_pos)?;
        } else {
            self.state = self.next_state(current_pos)?;
        }

        self.last_pos = current_pos;

        if self.state != prev_state {
            info!("Door state: {:?} → {:?}", prev_state, self.state);
        }
        Ok(())
    }

    fn next_state(&mut self, pos: i32) -> Result<DoorState> {
        // Check if door is stopping
        if self.state == DoorState::Stopping {
            let moving = self.velocity.abs() as i32 > DEAD_ZONE;
            if moving {
                // Coast rather than hold the brake: dropping nSLP lets the door
                // decelerate on its own. Holding an active brake against a door
                // still moving at speed jams the drivetrain.
                self.settle_count = 0;
                self.motor.brake()?;
                self.motor.sleep()?;
                return Ok(DoorState::Stopping);
            }

            // A stalled door already reads as stationary, so one quiet tick means
            // nothing — the door has not yet recoiled off whatever it hit. Hold
            // Stopping until the motion is settled for real before handing back
            // to assist, since coasting leaves the recoil undamped.
            self.settle_count += 1;
            if self.settle_count < SETTLE_TICKS {
                return Ok(DoorState::Stopping);
            }

            info!("Door stopped");
            self.settle_count = 0;
            self.motor.sleep()?;

            return Ok(if self.at_open(pos) {
                DoorState::Open
            } else if self.at_closed(pos) {
                DoorState::Closed
            } else {
                DoorState::Idle
            });
        }

        // If homing then handle separately
        if let DoorState::Homing {
            stall_count,
            timeout_ticks,
        } = self.state
        {
            self.tick_homing(pos, stall_count, timeout_ticks)?;
            return Ok(self.state.clone());
        }

        // If no endstops are set then do no motion
        if self.opening_counts == 0 {
            self.target = None;
            self.motor.sleep()?;
            return Ok(DoorState::Idle);
        }

        // ── End-stops ─────────────────────────────────────────────────────────
        // Allow movement away from an endstop if a door target drives in the
        // opposite direction (e.g. door overshot open endstop, close is still valid).

        let at_open = self.at_open(pos);
        if at_open && self.target == Some(100) {
            self.target = None;
            self.motor.brake()?;
            self.motor.sleep()?;
            return Ok(DoorState::Open);
        }

        let at_closed = self.at_closed(pos);
        if at_closed && self.target == Some(0) {
            if CLOSE_HOMING {
                // Door reached the encoder-based closed position without obstacle.
                // Stall into the physical stop to correct encoder drift.
                self.target = None;
                return self.start_homing().map(|()| self.state.clone());
            }

            self.target = None;
            self.motor.brake()?;
            self.motor.sleep()?;
            return Ok(DoorState::Closed);
        }

        if let Some(target_pct) = self.target {
            // ── Obstacle detection ────────────────────────────────────────────
            //Count consecutive no-motion ticks while powered, after a grace period for spin-up.
            self.drive_ticks += 1;
            if self.velocity.abs() as i32 > DEAD_ZONE {
                self.obstacle_stall_count = 0;
            } else if self.drive_ticks > STALL_GRACE_TICKS {
                self.obstacle_stall_count += 1;
                if self.obstacle_stall_count >= STALL_TICKS {
                    log::warn!("Obstacle detected — stopping motor");
                    self.target = None;
                    self.obstacle_stall_count = 0;
                    self.assist_lockout = ASSIST_LOCKOUT_TICKS;
                    self.settle_count = 0;
                    self.motor.brake()?;
                    self.motor.sleep()?;
                    return Ok(DoorState::Stopping);
                }
            }

            let target_counts = self.pct_to_counts(target_pct);

            // TODO: Check if target and current pos are already withing range and go idle if true

            let ramp = self.ramp_factor(pos, target_counts);
            let power = ((OPERATING_POWER as f32 * ramp) as i32).max(1);

            // ── Handle Target ────────────────────────────────
            self.motor.wake()?;
            if (pos - target_counts) * dir() < 0 {
                self.motor.set_power(power * dir())?;
                return Ok(DoorState::Opening);
            } else {
                self.motor.set_power(-power * dir())?;
                return Ok(DoorState::Closing);
            };
        }

        // ── Assist mode (manual push/pull detection) ──────────────────────────
        let moving = self.velocity.abs() as i32 > DEAD_ZONE;

        // The door recoils off whatever it just stalled against, and that recoil is
        // indistinguishable from a manual push. Stay disarmed until it settles,
        // otherwise an obstacle while opening immediately drives the door closed.
        if self.assist_lockout > 0 {
            if moving {
                self.last_move_time = Instant::now();
            }
            return Ok(self.state.clone());
        }

        if moving {
            self.last_move_time = Instant::now();

            let open_vel = self.velocity * dir() as f32;
            if open_vel > ASSIST_VELOCITY_THRESHOLD && !at_open {
                self.set_position(100)?; // Open
                Ok(DoorState::Opening)
            } else if open_vel < -ASSIST_VELOCITY_THRESHOLD && !at_closed {
                self.set_position(0)?; // Close
                Ok(DoorState::Closing)
            } else {
                self.target = None;
                Ok(DoorState::Idle)
            }
        } else if self.last_move_time.elapsed() > IDLE_TIMEOUT {
            if self.motor.is_awake() {
                self.motor.sleep()?;
            }

            return Ok(if self.at_open(pos) {
                DoorState::Open
            } else if self.at_closed(pos) {
                DoorState::Closed
            } else {
                DoorState::Idle
            });
        } else {
            Ok(self.state.clone())
        }
    }

    fn ramp_factor(&self, pos: i32, target: i32) -> f32 {
        let dist = (pos - target).abs();
        if dist < RAMP_DOWN_COUNTS {
            dist as f32 / RAMP_DOWN_COUNTS as f32
        } else {
            1.0
        }
    }

    fn pct_to_counts(&self, target: i32) -> i32 {
        if self.opening_counts != 0 {
            let target = target.clamp(0, 100);
            (target * self.opening_counts) / 100
        } else {
            0
        }
    }

    fn at_open(&self, pos: i32) -> bool {
        if INVERT_DIRECTION {
            pos <= self.opening_counts
        } else {
            pos >= self.opening_counts
        }
    }

    fn at_closed(&self, pos: i32) -> bool {
        if INVERT_DIRECTION {
            pos >= -CLOSED_THRESHOLD
        } else {
            pos <= CLOSED_THRESHOLD
        }
    }

    // ── Commands ───────────────────────────────────────────────────────

    /// Set the door to a certain postion between its endstops (0 = closed, 100 = open)
    pub fn set_position(&mut self, pos: i32) -> Result<()> {
        let pos = pos.clamp(0, 100);

        // There is no need to set the target again, which would cause the stall detection to reset.
        if self.target == Some(pos) {
            return Ok(());
        }

        info!("Setting target position to: {}", pos);
        self.target = Some(pos);
        self.obstacle_stall_count = 0;
        self.drive_ticks = 0;
        Ok(())
    }

    /// Start driving the door toward the open endstop.
    pub fn drive_open(&mut self) -> Result<()> {
        info!("Setting target to: Open");
        self.set_position(100)
    }

    /// Start driving the door toward the closed endstop.
    pub fn drive_close(&mut self) -> Result<()> {
        info!("Setting Target to: Close");
        self.set_position(0)
    }

    /// Stop and clear any door target. The motor coasts; the Stopping state waits
    /// for motion to settle before declaring the door stopped.
    pub fn stop(&mut self) -> Result<()> {
        info!("Setting Target to: Stopping");
        self.target = None;
        self.settle_count = 0;
        self.motor.brake()?;
        self.motor.sleep()?;
        self.state = DoorState::Stopping;
        Ok(())
    }

    // ── Homing ───────────────────────────────────────────────────────────

    /// Drive toward the closed endstop at CALIBRATE_POWER until stall, then
    /// reset the encoder to zero. Call on boot to establish a known position
    /// after a power loss.
    pub fn start_homing(&mut self) -> Result<()> {
        info!("Homing: driving to closed endstop");
        self.target = None;
        self.motor.wake()?;
        self.motor.set_power(-CALIBRATE_POWER * dir())?;
        self.state = DoorState::Homing {
            stall_count: 0,
            timeout_ticks: 0,
        };
        Ok(())
    }

    fn tick_homing(&mut self, pos: i32, stall_count: u32, timeout_ticks: u32) -> Result<()> {
        let delta = (pos - self.last_pos).abs();
        let timeout_ticks = timeout_ticks + 1;
        if timeout_ticks >= CALIBRATION_TIMEOUT_TICKS {
            log::error!("Homing: timeout waiting for closed stall");
            self.motor.brake()?;
            self.motor.sleep()?;
            // Set to zero anyways, the door should have moved.
            // The user will intervene to fix issue.
            self.encoder.reset();
            self.last_pos = 0;
            self.state = DoorState::Idle;
        } else if delta <= DEAD_ZONE as i32 {
            let stall_count = stall_count + 1;
            if stall_count >= STALL_TICKS {
                info!("Homing: closed endstop found, resetting encoder to 0");
                self.motor.brake()?;
                self.motor.sleep()?;
                self.encoder.reset();
                self.last_pos = 0;
                return self.stop();
            } else {
                self.state = DoorState::Homing {
                    stall_count,
                    timeout_ticks,
                };
            }
        } else {
            self.state = DoorState::Homing {
                stall_count: 0,
                timeout_ticks,
            };
        }

        Ok(())
    }

    // ── Calibration ───────────────────────────────────────────────────────────

    /// Begin auto-calibration. The sequence runs non-blocking through tick():
    ///   1. Drive toward closed at CALIBRATE_POWER → stall → reset encoder to 0
    ///   2. Brief pause, then drive toward open → stall → record position
    ///
    /// Call take_calibration_result() each tick to receive (closed, open) when done.
    pub fn start_calibrate(&mut self) -> Result<()> {
        if self.state == DoorState::Calibrating {
            return Err(anyhow::anyhow!("Door is already calibrating"));
        }

        info!("Calibration: starting — driving to closed endstop");

        self.target = None;
        self.state = DoorState::Calibrating;
        self.motor.wake()?;
        self.motor.set_power(-CALIBRATE_POWER * dir())?;
        self.calibration = Some(CalibrationStep::DrivingClosed {
            stall_count: 0,
            timeout_ticks: 0,
        });
        Ok(())
    }

    /// Advance the calibration state machine by one tick. Called by tick().
    fn tick_calibration(&mut self, pos: i32) -> Result<()> {
        self.state = DoorState::Calibrating;

        let delta = (pos - self.last_pos).abs();

        let next = match self.calibration.take() {
            Some(CalibrationStep::DrivingClosed {
                stall_count,
                timeout_ticks,
            }) => {
                let timeout_ticks = timeout_ticks + 1;
                if timeout_ticks >= CALIBRATION_TIMEOUT_TICKS {
                    log::error!("Calibration: timeout waiting for closed stall");
                    self.motor.brake()?;
                    self.motor.sleep()?;
                    self.state = DoorState::Idle;
                    None
                } else if delta <= DEAD_ZONE as i32 {
                    let stall_count = stall_count + 1;
                    if stall_count >= CALIBRATION_STALL_TICKS {
                        // Stall detected — reset encoder, pause before driving open
                        info!("Calibration: closed endstop found, resetting encoder");
                        self.motor.brake()?;
                        self.encoder.reset();
                        self.last_pos = 0;
                        Some(CalibrationStep::Pause {
                            remaining: CALIBRATION_PAUSE_TICKS,
                        })
                    } else {
                        Some(CalibrationStep::DrivingClosed {
                            stall_count,
                            timeout_ticks,
                        })
                    }
                } else {
                    Some(CalibrationStep::DrivingClosed {
                        stall_count: 0,
                        timeout_ticks,
                    })
                }
            }

            Some(CalibrationStep::Pause { remaining }) => {
                if remaining == 0 {
                    info!("Calibration: driving to open endstop");
                    self.motor.wake()?;
                    self.motor.set_power(CALIBRATE_POWER * dir())?;
                    Some(CalibrationStep::DrivingOpen {
                        stall_count: 0,
                        timeout_ticks: 0,
                    })
                } else {
                    Some(CalibrationStep::Pause {
                        remaining: remaining - 1,
                    })
                }
            }

            Some(CalibrationStep::DrivingOpen {
                stall_count,
                timeout_ticks,
            }) => {
                let timeout_ticks = timeout_ticks + 1;
                if timeout_ticks >= CALIBRATION_TIMEOUT_TICKS {
                    log::error!("Calibration: timeout waiting for open stall");
                    self.motor.brake()?;
                    self.motor.sleep()?;
                    self.state = DoorState::Idle;
                    None
                } else if delta <= DEAD_ZONE {
                    let stall_count = stall_count + 1;
                    if stall_count >= CALIBRATION_STALL_TICKS {
                        // Done
                        let endstop_open = self.encoder.read();
                        self.motor.brake()?;
                        self.motor.sleep()?;
                        self.set_opening(endstop_open);
                        info!("Calibration complete: closed=0, open={endstop_open}");
                        self.state = DoorState::Open;
                        None
                    } else {
                        Some(CalibrationStep::DrivingOpen {
                            stall_count,
                            timeout_ticks,
                        })
                    }
                } else {
                    Some(CalibrationStep::DrivingOpen {
                        stall_count: 0,
                        timeout_ticks,
                    })
                }
            }

            None => None,
        };

        self.calibration = next;
        Ok(())
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    pub fn velocity(&self) -> f32 {
        self.velocity
    }

    pub fn position(&self) -> i32 {
        self.encoder.read()
    }

    pub fn position_pct(&self) -> i32 {
        if self.opening_counts != 0 {
            let position = self.position();
            let pct = position * 100 / self.opening_counts;
            pct.clamp(0, 100)
        } else {
            0
        }
    }

    pub fn state(&self) -> &DoorState {
        &self.state
    }

    pub fn power(&self) -> i32 {
        self.motor.get_power()
    }

    pub fn set_opening(&mut self, opening: i32) {
        info!("Opening set: open={opening}");
        self.opening_counts = opening;
    }

    pub fn opening(&self) -> i32 {
        self.opening_counts
    }

    pub fn loop_period() -> Duration {
        LOOP_PERIOD
    }
}

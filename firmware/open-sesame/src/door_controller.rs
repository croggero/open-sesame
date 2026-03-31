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
use anyhow::Result;
use log::info;
use std::time::{Duration, Instant};

// All tuning constants live in config.toml and are exposed via crate::config.
use config::{
    ASSIST_VELOCITY_THRESHOLD, CALIBRATE_POWER, CALIBRATION_PAUSE_TICKS, CALIBRATION_STALL_TICKS,
    CALIBRATION_TIMEOUT_TICKS, DEAD_ZONE, ENDSTOP_CLOSED, ENDSTOP_OPEN, ENDSTOP_RAMP_COUNTS,
    IDLE_TIMEOUT_MS, INVERT_DIRECTION, LOOP_PERIOD_MS, LPF_ALPHA, OPERATING_POWER, STALL_TICKS,
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
    endstop_open: i32,
    endstop_closed: i32,
    /// Active calibration step, or None when not calibrating.
    calibration: Option<CalibrationStep>,
    /// Consecutive no-motion ticks while motor is powered (obstacle detection).
    obstacle_stall_count: u32,
    /// True once we have seen movement after a door target was set.
    /// Prevents false triggers during motor spin-up.
    obstacle_moving_seen: bool,
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
            endstop_open: ENDSTOP_OPEN,
            endstop_closed: ENDSTOP_CLOSED,
            calibration: None,
            obstacle_stall_count: 0,
            obstacle_moving_seen: false,
        }
    }

    /// Override the endstop positions (e.g. after loading calibration from NVS).
    pub fn set_endstops(&mut self, closed: i32, open: i32) {
        info!("Endstops set: closed={closed}, open={open}");
        self.endstop_closed = closed;
        self.endstop_open = open;
    }

    /// Run one tick of the control loop. Call this every ~LOOP_PERIOD.
    pub fn tick(&mut self) -> Result<()> {
        let current_pos = self.encoder.read();
        let raw_velocity = (current_pos - self.last_pos) as f32;

        self.velocity = LPF_ALPHA * self.velocity + (1.0 - LPF_ALPHA) * raw_velocity;

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

    fn is_endstops_set(&self) -> bool {
        self.endstop_open != self.endstop_closed
    }

    fn pct_to_counts(&self, target: i32) -> i32 {
        let range = (self.endstop_open - self.endstop_closed).abs();
        if range > 0 {
            let target = target.clamp(0, 100);
            (target * range) / 100 + self.endstop_closed
        } else {
            0
        }
    }

    fn next_state(&mut self, pos: i32) -> Result<DoorState> {
        // Check if door is stopping
        if self.state == DoorState::Stopping {
            let moving = self.velocity.abs() as i32 > DEAD_ZONE;
            if moving {
                // If door has not stopped then let it continue stopping.
                self.motor.brake()?;
                self.motor.sleep()?;
                return Ok(DoorState::Stopping);
            } else {
                info!("Door stopped");
                return Ok(DoorState::Idle);
            }
        }

        // If no endstops are set then do no motion
        if !self.is_endstops_set() {
            self.target = None;
            self.motor.sleep()?;
            return Ok(DoorState::Idle);
        }

        // ── End-stops ─────────────────────────────────────────────────────────
        // Allow movement away from an endstop if a door target drives in the
        // opposite direction (e.g. door overshot open endstop, close is still valid).

        let at_open = if INVERT_DIRECTION {
            pos <= self.endstop_open
        } else {
            pos >= self.endstop_open
        };

        if at_open && self.target != Some(0) {
            self.target = None;
            self.motor.brake()?;
            self.motor.sleep()?;
            return Ok(DoorState::Open);
        }

        let at_closed = if INVERT_DIRECTION {
            pos >= self.endstop_closed
        } else {
            pos <= self.endstop_closed
        };

        if at_closed && self.target != Some(100) {
            self.target = None;
            self.motor.brake()?;
            self.motor.sleep()?;
            return Ok(DoorState::Closed);
        }

        if let Some(target_pct) = self.target {
            // ── Obstacle detection ────────────────────────────────────────────
            // Wait for first movement after motor starts, then watch for stall.
            if self.velocity.abs() as i32 > DEAD_ZONE {
                self.obstacle_moving_seen = true;
                self.obstacle_stall_count = 0;
            } else if self.obstacle_moving_seen {
                self.obstacle_stall_count += 1;
                if self.obstacle_stall_count >= STALL_TICKS {
                    log::warn!("Obstacle detected — stopping motor");
                    self.target = None;
                    self.obstacle_stall_count = 0;
                    self.obstacle_moving_seen = false;
                    self.motor.brake()?;
                    self.motor.sleep()?;
                    return Ok(DoorState::Stopping);
                }
            }

            let target_counts = self.pct_to_counts(target_pct);
            let ramp = self.ramp_factor(pos, target_counts);
            let power = ((OPERATING_POWER as f32 * ramp) as i32).max(1);

            // ── Handle Target ────────────────────────────────
            self.motor.wake()?;
            if pos - target_counts < 0 {
                self.motor.set_power(power * dir())?;
                return Ok(DoorState::Opening);
            } else {
                self.motor.set_power(-power * dir())?;
                return Ok(DoorState::Closing);
            };
        }

        // ── Assist mode (manual push/pull detection) ──────────────────────────
        let moving = self.velocity.abs() as i32 > DEAD_ZONE;

        if moving {
            self.last_move_time = Instant::now();

            Ok(if self.velocity > ASSIST_VELOCITY_THRESHOLD {
                self.target = Some(100); // Open
                DoorState::Opening
            } else if self.velocity < -ASSIST_VELOCITY_THRESHOLD {
                self.target = Some(0); // Close
                DoorState::Closing
            } else {
                self.target = None;
                DoorState::Idle
            })
        } else if self.last_move_time.elapsed() > IDLE_TIMEOUT {
            if self.motor.is_awake() {
                self.motor.sleep()?;
            }
            Ok(DoorState::Idle)
        } else {
            Ok(self.state.clone())
        }
    }

    fn ramp_factor(&self, pos: i32, target: i32) -> f32 {
        let dist = (pos - target).abs();
        if dist < ENDSTOP_RAMP_COUNTS {
            dist as f32 / ENDSTOP_RAMP_COUNTS as f32
        } else {
            1.0
        }
    }

    // ── Commands ───────────────────────────────────────────────────────

    /// Set the door to a certain postion between its endstops (0 = closed, 100 = open)
    pub fn set_position(&mut self, pos: i32) -> Result<()> {
        let pos = pos.clamp(0, 100);
        info!("Setting target position to: {}", pos);
        self.target = Some(pos);
        self.obstacle_stall_count = 0;
        self.obstacle_moving_seen = false;
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

    /// Stop and clear any door target. Motor sleeps.
    pub fn stop(&mut self) -> Result<()> {
        info!("Setting Target to: Stopping");
        self.target = None;
        self.motor.brake()?;
        self.motor.sleep()?;
        self.state = DoorState::Stopping;
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
                        self.set_endstops(0, endstop_open);
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
        let range = (self.endstop_open - self.endstop_closed).abs();
        if range > 0 {
            let position = self.position();
            let pct = (position - self.endstop_closed) * 100 / range;
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

    pub fn loop_period() -> Duration {
        LOOP_PERIOD
    }
}

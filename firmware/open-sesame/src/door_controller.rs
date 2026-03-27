// src/door_controller.rs
//
// Core assistive control loop for the sliding door.
//
// State machine:
//
//   Remote command (open/close)
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
// Endstops always take priority and clear the remote target.
//
// The control loop runs at ~10 ms intervals on the calling thread.
// The encoder ISR updates position atomically from a separate context.

use crate::encoder::Encoder;
use crate::motor::MotorDriver;
use crate::config;
use anyhow::Result;
use log::info;
use std::time::{Duration, Instant};

// All tuning constants live in config.toml and are exposed via crate::config.
use config::{
    ASSIST_GAIN, DEAD_ZONE, ENDSTOP_CLOSED, ENDSTOP_OPEN, ENDSTOP_RAMP_COUNTS,
    IDLE_TIMEOUT_MS, INVERT_DIRECTION, LPF_ALPHA, LOOP_PERIOD_MS, REMOTE_POWER,
};

const IDLE_TIMEOUT: Duration = Duration::from_millis(IDLE_TIMEOUT_MS);
const LOOP_PERIOD: Duration = Duration::from_millis(LOOP_PERIOD_MS);

/// Returns 1 or -1 based on INVERT_DIRECTION.
const fn dir() -> i32 {
    if INVERT_DIRECTION { -1 } else { 1 }
}

// ─────────────────────────────────────────────────────────────────────────────
// DoorState
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq)]
pub enum DoorState {
    Idle,
    Opening,
    Closing,
    Open,
    Closed,
}

impl DoorState {
    pub fn as_str(&self) -> &'static str {
        match self {
            DoorState::Idle => "idle",
            DoorState::Opening => "opening",
            DoorState::Closing => "closing",
            DoorState::Open => "open",
            DoorState::Closed => "closed",
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// RemoteTarget
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq)]
enum RemoteTarget {
    Open,
    Close,
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
    target: Option<RemoteTarget>,
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
        }
    }

    /// Run one tick of the control loop. Call this every ~LOOP_PERIOD.
    pub fn tick(&mut self) -> Result<()> {
        let current_pos = self.encoder.read();
        let raw_velocity = (current_pos - self.last_pos) as f32;
        self.last_pos = current_pos;

        self.velocity = LPF_ALPHA * self.velocity + (1.0 - LPF_ALPHA) * raw_velocity;

        let prev_state = self.state.clone();
        self.state = self.next_state(current_pos)?;

        if self.state != prev_state {
            info!("Door state: {:?} → {:?}", prev_state, self.state);
        }
        Ok(())
    }

    fn next_state(&mut self, pos: i32) -> Result<DoorState> {
        // ── End-stops ─────────────────────────────────────────────────────────
        // Allow movement away from an endstop if a remote target drives in the
        // opposite direction (e.g. door overshot open endstop, close is still valid).

        let at_open = pos >= ENDSTOP_OPEN;
        if at_open && self.target != Some(RemoteTarget::Close) {
            self.target = None;
            self.motor.brake()?;
            self.motor.sleep()?;
            return Ok(DoorState::Open);
        }

        let at_closed = pos <= ENDSTOP_CLOSED;
        if at_closed && self.target != Some(RemoteTarget::Open) {
            self.target = None;
            self.motor.brake()?;
            self.motor.sleep()?;
            return Ok(DoorState::Closed);
        }

        // ── Remote drive (open/close command) ────────────────────────────────
        if let Some(target) = self.target {
            let ramp = self.endstop_ramp_factor(pos);
            let power = (REMOTE_POWER as f32 * ramp) as i32;
            self.motor.wake()?;
            return match target {
                RemoteTarget::Open => {
                    self.motor.set_power(power * dir())?;
                    Ok(DoorState::Opening)
                }
                RemoteTarget::Close => {
                    self.motor.set_power(-power * dir())?;
                    Ok(DoorState::Closing)
                }
            };
        }

        // ── Assist mode (manual push/pull detection) ──────────────────────────
        let moving = self.velocity.abs() as i32 > DEAD_ZONE;

        if moving {
            self.last_move_time = Instant::now();

            if !self.motor.is_awake() {
                self.motor.wake()?;
            }

            let ramp = self.endstop_ramp_factor(pos);
            let power = (self.velocity * ASSIST_GAIN * ramp) as i32 * dir();
            self.motor.set_power(power.clamp(-255, 255))?;

            Ok(if self.velocity > 0.0 {
                self.target = Some(RemoteTarget::Open);
                DoorState::Opening
            } else if self.velocity < 0.0 {
                self.target = Some(RemoteTarget::Close);
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

    fn endstop_ramp_factor(&self, pos: i32) -> f32 {
        let dist_open = (pos - ENDSTOP_OPEN).abs();
        let dist_closed = (pos - ENDSTOP_CLOSED).abs();
        let min_dist = dist_open.min(dist_closed);

        if min_dist < ENDSTOP_RAMP_COUNTS {
            min_dist as f32 / ENDSTOP_RAMP_COUNTS as f32
        } else {
            1.0
        }
    }

    // ── Remote commands ───────────────────────────────────────────────────────

    /// Start driving the door toward the open endstop.
    pub fn drive_open(&mut self) {
        info!("Remote: drive open");
        self.target = Some(RemoteTarget::Open);
    }

    /// Start driving the door toward the closed endstop.
    pub fn drive_close(&mut self) {
        info!("Remote: drive close");
        self.target = Some(RemoteTarget::Close);
    }

    /// Stop and clear any remote target. Motor sleeps.
    pub fn stop(&mut self) -> Result<()> {
        info!("Remote: stop");
        self.target = None;
        self.motor.brake()?;
        self.motor.sleep()?;
        Ok(())
    }

    /// Directly set motor power (clears remote target).
    pub fn set_power(&mut self, power: i32) -> Result<()> {
        self.target = None;
        if power == 0 {
            self.motor.brake()?;
            self.motor.sleep()?;
        } else {
            self.motor.wake()?;
            self.motor.set_power(power)?;
        }
        Ok(())
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    pub fn velocity(&self) -> f32 {
        self.velocity
    }

    pub fn position(&self) -> i32 {
        self.encoder.read()
    }

    pub fn state(&self) -> &DoorState {
        &self.state
    }

    pub fn reset_position(&self) {
        self.encoder.reset();
    }

    pub fn power(&self) -> i32 {
        self.motor.get_power()
    }

    pub fn loop_period() -> Duration {
        LOOP_PERIOD
    }
}

// src/motor.rs
//
// Abstraction over the DRV8874 H-bridge motor driver.
//
// The DRV8874 supports two control modes selected by the hardware MODE pin:
//
//   IN/IN mode  (MODE = LOW):
//     Forward : IN1 = PWM,  IN2 = LOW   → variable speed
//     Reverse : IN1 = LOW,  IN2 = PWM   → requires second LEDC channel; we only
//               have one, so reverse is full-power only (IN2 = HIGH).
//     Brake   : IN1 = HIGH, IN2 = HIGH  (active brake via high-side + low-side)
//     Coast   : nSLP = LOW
//
//   PH/EN mode  (MODE = HIGH):
//     Forward : PH = HIGH, EN = PWM     → variable speed
//     Reverse : PH = LOW,  EN = PWM     → variable speed (advantage over IN/IN)
//     Brake   : EN = LOW,  nSLP = HIGH  (both low-side FETs on, motor shorted)
//     Coast   : nSLP = LOW
//
// Pin assignments — see src/pins.rs:
//   IN1 / EN  → PIN_MOTOR_IN1  (LEDC PWM channel 0)
//   IN2 / PH  → PIN_MOTOR_IN2  (digital output)
//   nSLP      → PIN_MOTOR_NSLP (active-high enable)

use anyhow::Result;
use esp_idf_hal::{
    gpio::{Output, PinDriver},
    ledc::{
        config::TimerConfig, LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, Resolution,
        SpeedMode,
    },
    peripheral::Peripheral,
    prelude::*,
};

use crate::config;

/// Maximum duty cycle for 8-bit resolution (2^8 - 1).
const MAX_DUTY: u32 = 255;

// ─────────────────────────────────────────────────────────────────────────────
// MotorMode
// ─────────────────────────────────────────────────────────────────────────────

/// Selects which DRV8874 control mode the hardware MODE pin is wired for.
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum MotorMode {
    /// IN/IN mode (MODE pin LOW). Variable-speed forward only; reverse is
    /// full-power (single LEDC channel limitation).
    InIn,
    /// PH/EN mode (MODE pin HIGH). Variable speed in both directions.
    PhEn,
}

// ─────────────────────────────────────────────────────────────────────────────
// MotorDriver
// ─────────────────────────────────────────────────────────────────────────────

pub struct MotorDriver<'d> {
    pwm: LedcDriver<'d>, // IN1 (IN/IN) or EN (PH/EN)
    in2: PinDriver<'d, esp_idf_hal::gpio::AnyOutputPin, Output>, // IN2 or PH
    nslp: PinDriver<'d, esp_idf_hal::gpio::AnyOutputPin, Output>,
    mode: MotorMode,
    awake: bool,
}

impl<'d> MotorDriver<'d> {
    pub fn new<S: SpeedMode>(
        timer: impl Peripheral<P: LedcTimer<SpeedMode = S>> + 'd,
        channel: impl Peripheral<P: LedcChannel<SpeedMode = S>> + 'd,
        pin_in1: impl Peripheral<P: esp_idf_hal::gpio::OutputPin> + 'd,
        pin_in2: impl Peripheral<P: esp_idf_hal::gpio::OutputPin> + 'd,
        pin_nslp: impl Peripheral<P: esp_idf_hal::gpio::OutputPin> + 'd,
        mode: MotorMode,
    ) -> Result<Self> {
        let timer_cfg = TimerConfig::default()
            .frequency(config::PWM_FREQ_HZ.Hz().into())
            .resolution(Resolution::Bits8);

        let timer_drv = LedcTimerDriver::new(timer, &timer_cfg)?;
        let pwm = LedcDriver::new(channel, &timer_drv, pin_in1)?;
        let in2 = PinDriver::output(pin_in2.into_ref().map_into())?;
        let nslp = PinDriver::output(pin_nslp.into_ref().map_into())?;

        let mut drv = Self {
            pwm,
            in2,
            nslp,
            mode,
            awake: false,
        };
        drv.sleep()?;
        Ok(drv)
    }

    // ── Power control ─────────────────────────────────────────────────────────

    /// Wake the driver (nSLP HIGH). Must be called before setting speed.
    pub fn wake(&mut self) -> Result<()> {
        if !self.awake {
            self.nslp.set_high()?;
            // DRV8874 needs ~1 ms after nSLP goes high before accepting commands
            std::thread::sleep(std::time::Duration::from_millis(1));
            self.awake = true;
        }
        Ok(())
    }

    /// Put the driver into coast/sleep mode (nSLP LOW).
    pub fn sleep(&mut self) -> Result<()> {
        self.pwm.set_duty(0)?;
        self.in2.set_low()?;
        self.nslp.set_low()?;
        self.awake = false;
        Ok(())
    }

    // ── Motion ────────────────────────────────────────────────────────────────

    /// Set motor output. `power` in [-255, 255].
    /// Positive = forward, negative = reverse, 0 = active brake.
    /// Non-zero values below MIN_POWER are clamped up to MIN_POWER.
    pub fn set_power(&mut self, power: i32) -> Result<()> {
        let power = match power.clamp(-255, 255) {
            p if p > 0 => p.max(config::MIN_DUTY),
            p if p < 0 => p.min(-config::MIN_DUTY),
            p => p,
        };
        match self.mode {
            MotorMode::InIn => self.set_power_inin(power),
            MotorMode::PhEn => self.set_power_phen(power),
        }
    }

    pub fn get_power(&self) -> i32 {
        let power = self.pwm.get_duty();
        let is_high = self.in2.is_set_high();

        (power as i32) * if is_high { 1 } else { -1 }
    }

    /// Convenience: active brake.
    pub fn brake(&mut self) -> Result<()> {
        self.set_power(0)
    }

    pub fn is_awake(&self) -> bool {
        self.awake
    }

    // ── Mode-specific helpers ─────────────────────────────────────────────────

    fn set_power_inin(&mut self, power: i32) -> Result<()> {
        if power == 0 {
            // Brake: IN1 = HIGH, IN2 = HIGH
            self.pwm.set_duty(MAX_DUTY)?;
            self.in2.set_high()?;
        } else if power > 0 {
            // Forward: IN1 = PWM, IN2 = LOW
            self.in2.set_low()?;
            self.pwm.set_duty(power as u32)?;
        } else {
            // Reverse: IN1 = LOW, IN2 = HIGH (full-power only — single LEDC limitation)
            self.pwm.set_duty(0)?;
            self.in2.set_high()?;
        }
        Ok(())
    }

    fn set_power_phen(&mut self, power: i32) -> Result<()> {
        if power == 0 {
            // Brake: EN = LOW, nSLP stays HIGH (both low-side FETs on)
            //self.pwm.set_duty(0)?

            self.in2.set_low()?;
            self.pwm.set_duty(0)?;
        } else if power > 0 {
            // Forward: PH = HIGH, EN = PWM
            self.in2.set_high()?;
            self.pwm.set_duty(power as u32)?;
        } else {
            // Reverse: PH = LOW, EN = PWM  (variable speed — PH/EN advantage)
            self.in2.set_low()?;
            self.pwm.set_duty(power.abs() as u32)?;
        }
        Ok(())
    }
}

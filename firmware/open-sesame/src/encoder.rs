// src/encoder.rs
//
// Hardware quadrature encoder reader using the ESP32's PCNT peripheral.
//
// PCNT counts edges in hardware — no CPU-driven ISR per edge. The only
// interrupt fires when the 16-bit counter hits HIGH_LIMIT or LOW_LIMIT,
// at which point we accumulate the overflow into an AtomicI32 so callers
// see a full 32-bit position.
//
// ESP32-S3 pin assignments (see pins.rs):
//   ENC_A (PCNT pulse / edge pin)
//   ENC_B (PCNT control / level pin)

use std::cmp::min;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::Arc;

use anyhow::Result;
use esp_idf_hal::gpio::AnyInputPin;
use esp_idf_hal::pcnt::*;
use esp_idf_hal::peripheral::Peripheral;

// Overflow thresholds — when the 16-bit counter hits either limit the ISR
// accumulates the value into the AtomicI32 and the hardware resets to zero.
const LOW_LIMIT: i16 = -100;
const HIGH_LIMIT: i16 = 100;

// APB-clock filter value.  Pulses shorter than this many APB cycles are
// ignored (hardware debounce).  10 µs × 80 MHz = 800 cycles, clamped to
// the 10-bit max of 1023.
const FILTER_VALUE: u16 = 800;

/// Shared atomic position counter. Positive = door moving toward open.
pub type EncoderPosition = Arc<AtomicI32>;

pub struct Encoder<'d> {
    unit: PcntDriver<'d>,
    position: EncoderPosition,
}

impl<'d> Encoder<'d> {
    /// Initialise the encoder on a PCNT unit with two GPIO pins.
    pub fn new<PCNT: Pcnt>(
        pcnt: impl Peripheral<P = PCNT> + 'd,
        pin_a: impl Peripheral<P = impl esp_idf_hal::gpio::InputPin> + 'd,
        pin_b: impl Peripheral<P = impl esp_idf_hal::gpio::InputPin> + 'd,
    ) -> Result<Self> {
        let mut unit = PcntDriver::new(
            pcnt,
            Some(pin_a),
            Some(pin_b),
            Option::<AnyInputPin>::None,
            Option::<AnyInputPin>::None,
        )?;

        // Channel 0: pin_a as edge input, pin_b as level control
        unit.channel_config(
            PcntChannel::Channel0,
            PinIndex::Pin0,
            PinIndex::Pin1,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Decrement,
                neg_mode: PcntCountMode::Increment,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        )?;

        // Channel 1: pin_b as edge input, pin_a as level control
        unit.channel_config(
            PcntChannel::Channel1,
            PinIndex::Pin1,
            PinIndex::Pin0,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Increment,
                neg_mode: PcntCountMode::Decrement,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        )?;

        // Hardware glitch filter
        unit.set_filter_value(min(FILTER_VALUE, 1023))?;
        unit.filter_enable()?;

        // Overflow tracking via ISR
        let position: EncoderPosition = Arc::new(AtomicI32::new(0));
        unsafe {
            let overflow = position.clone();
            unit.subscribe(move |status| {
                let status = PcntEventType::from_repr_truncated(status);
                if status.contains(PcntEvent::HighLimit) {
                    overflow.fetch_add(HIGH_LIMIT as i32, Ordering::SeqCst);
                }
                if status.contains(PcntEvent::LowLimit) {
                    overflow.fetch_add(LOW_LIMIT as i32, Ordering::SeqCst);
                }
            })?;
        }
        unit.event_enable(PcntEvent::HighLimit)?;
        unit.event_enable(PcntEvent::LowLimit)?;

        unit.counter_pause()?;
        unit.counter_clear()?;
        unit.counter_resume()?;

        Ok(Self { unit, position })
    }

    /// Read the current 32-bit position (overflow accumulator + hardware counter).
    pub fn read(&self) -> i32 {
        let hw = self.unit.get_counter_value().unwrap_or(0) as i32;
        self.position.load(Ordering::Relaxed) + hw
    }

    /// Reset the position counter to zero.
    pub fn reset(&self) {
        self.position.store(0, Ordering::Relaxed);
        let _ = self.unit.counter_clear();
    }
}

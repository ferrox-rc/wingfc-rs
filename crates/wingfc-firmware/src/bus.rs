//! Inter-task mailboxes for the multi-rate decoupled architecture.
//!
//! - `IMU_WATCH`: decimated 208.33 Hz IMU frames (Tier 1 -> Tier 2)
//! - `ACTUATOR_LATCH`: mixed pulse widths in RAM (Tier 2 -> PWM drivers)

#![allow(dead_code)]

use core::sync::atomic::{AtomicU16, Ordering};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use wingfc_core::downsample::ImuDecimated;

/// Number of actuator outputs latched (servos/motors).
pub const ACTUATOR_COUNT: usize = 8;

/// Decimated IMU frames published at 208.33 Hz.
pub static IMU_WATCH: Watch<ThreadModeRawMutex, ImuDecimated, 2> = Watch::new();

/// Lock-free per-channel pulse width latch, in microseconds.
///
/// The flight loop stores targets; PWM timers load them at their own
/// native rates (50 Hz analog, 208 Hz digital) independent of loop timing.
pub struct ActuatorLatch {
    pulses_us: [AtomicU16; ACTUATOR_COUNT],
}

impl ActuatorLatch {
    pub const fn new() -> Self {
        const Z: AtomicU16 = AtomicU16::new(0);
        Self { pulses_us: [Z; ACTUATOR_COUNT] }
    }

    #[inline]
    pub fn store(&self, ch: usize, pulse_us: u16) {
        if let Some(a) = self.pulses_us.get(ch) {
            a.store(pulse_us, Ordering::Relaxed);
        }
    }

    #[inline]
    pub fn load(&self, ch: usize) -> u16 {
        self.pulses_us.get(ch).map_or(0, |a| a.load(Ordering::Relaxed))
    }
}

pub static ACTUATOR_LATCH: ActuatorLatch = ActuatorLatch::new();

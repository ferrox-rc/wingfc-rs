//! Inter-task mailboxes for the multi-rate decoupled architecture.
//!
//! - `IMU_WATCH`: decimated 208.33 Hz IMU frames (Tier 1 -> Tier 2)
//! - `ACTUATOR_LATCH`: whole-frame atomic double-buffered pulse widths (Tier 2 -> PWM drivers)

#![allow(dead_code)]

use core::sync::atomic::{AtomicU8, AtomicU16, Ordering};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use wingfc_core::downsample::ImuDecimated;
use wingfc_core::mixer::AirframeOutputs;

/// Total number of actuator outputs latched (8 servos + 2 motors).
pub const ACTUATOR_COUNT: usize = 10;
pub const ACT_SERVO1: usize = 0;
pub const ACT_SERVO2: usize = 1;
pub const ACT_SERVO3: usize = 2;
pub const ACT_SERVO4: usize = 3;
pub const ACT_SERVO5: usize = 4;
pub const ACT_SERVO6: usize = 5;
pub const ACT_SERVO7: usize = 6;
pub const ACT_SERVO8: usize = 7;
pub const ACT_MOTOR1: usize = 8;
pub const ACT_MOTOR2: usize = 9;

/// Decimated IMU frames published at 208.33 Hz.
pub static IMU_WATCH: Watch<ThreadModeRawMutex, ImuDecimated, 2> = Watch::new();

/// Lock-free, double-buffered actuator latch.
///
/// Ensures cross-channel atomicity: all control surface deflections and motor
/// throttles update as a single atomic frame, preventing single-frame asymmetric
/// deflection splits (e.g. elevon roll twitch during pitch commands).
pub struct ActuatorLatch {
    buffers: [[AtomicU16; ACTUATOR_COUNT]; 2],
    active_bank: AtomicU8,
}

impl ActuatorLatch {
    pub const fn new() -> Self {
        const Z: AtomicU16 = AtomicU16::new(1000);
        Self {
            buffers: [
                [
                    AtomicU16::new(1500), AtomicU16::new(1500), AtomicU16::new(1500), AtomicU16::new(1500),
                    AtomicU16::new(1500), AtomicU16::new(1500), AtomicU16::new(1500), AtomicU16::new(1500),
                    AtomicU16::new(1000), AtomicU16::new(1000),
                ],
                [
                    AtomicU16::new(1500), AtomicU16::new(1500), AtomicU16::new(1500), AtomicU16::new(1500),
                    AtomicU16::new(1500), AtomicU16::new(1500), AtomicU16::new(1500), AtomicU16::new(1500),
                    AtomicU16::new(1000), AtomicU16::new(1000),
                ],
            ],
            active_bank: AtomicU8::new(0),
        }
    }

    /// Atomically commits a full mixer output frame into the inactive bank and flips active bank.
    #[inline]
    pub fn publish_outputs(&self, outputs: &AirframeOutputs) {
        let current = self.active_bank.load(Ordering::Acquire);
        let next = (1 - (current & 1)) as usize;

        self.buffers[next][ACT_SERVO1].store(outputs.servo1_us, Ordering::Relaxed);
        self.buffers[next][ACT_SERVO2].store(outputs.servo2_us, Ordering::Relaxed);
        self.buffers[next][ACT_SERVO3].store(outputs.servo3_us, Ordering::Relaxed);
        self.buffers[next][ACT_SERVO4].store(outputs.servo4_us, Ordering::Relaxed);
        self.buffers[next][ACT_SERVO5].store(outputs.servo5_us, Ordering::Relaxed);
        self.buffers[next][ACT_SERVO6].store(outputs.servo6_us, Ordering::Relaxed);
        self.buffers[next][ACT_SERVO7].store(outputs.servo7_us, Ordering::Relaxed);
        self.buffers[next][ACT_SERVO8].store(outputs.servo8_us, Ordering::Relaxed);
        self.buffers[next][ACT_MOTOR1].store(outputs.motor1_throttle, Ordering::Relaxed);
        self.buffers[next][ACT_MOTOR2].store(outputs.motor2_throttle, Ordering::Relaxed);

        self.active_bank.store(next as u8, Ordering::Release);
    }

    /// Loads the active pulse width for a given channel in microseconds.
    #[inline]
    pub fn load(&self, ch: usize) -> u16 {
        let bank = (self.active_bank.load(Ordering::Acquire) & 1) as usize;
        self.buffers[bank].get(ch).map_or(1000, |a| a.load(Ordering::Relaxed))
    }

    /// Convenience single-channel store into current active bank (e.g. for failsafe or initialization).
    #[inline]
    pub fn store(&self, ch: usize, pulse_us: u16) {
        let bank = (self.active_bank.load(Ordering::Acquire) & 1) as usize;
        if let Some(a) = self.buffers[bank].get(ch) {
            a.store(pulse_us, Ordering::Relaxed);
        }
    }
}

pub static ACTUATOR_LATCH: ActuatorLatch = ActuatorLatch::new();

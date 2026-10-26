//! 8:1 boxcar (sinc) decimator: 1666.6 Hz IMU samples -> 208.33 Hz flight loop.
//!
//! Averaging N consecutive samples is a moving-average (sinc) low-pass whose
//! nulls fall on multiples of `fs / N`, suppressing motor/prop harmonics while
//! producing an exactly regular `dt` of 8 / 1666.6 Hz = 4.8 ms.

use crate::math::Vector3;

/// Decimation factor.
pub const DECIMATION: u8 = 8;
/// Nominal decimated period in seconds (8 / 1666.6 Hz).
pub const DECIMATED_DT_S: f32 = 0.0048;

/// A decimated IMU frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuDecimated {
    /// Mean accelerometer (sensor units, as supplied).
    pub accel: Vector3,
    /// Mean gyroscope (sensor units, as supplied).
    pub gyro: Vector3,
    /// Fixed loop period in seconds.
    pub dt: f32,
    /// Monotonically increasing frame sequence counter for overrun detection.
    pub seq: u32,
}

/// Accumulates `DECIMATION` samples and emits their mean.
#[derive(Debug, Clone, Copy)]
pub struct BoxcarAccumulator8 {
    accel: Vector3,
    gyro: Vector3,
    count: u8,
    seq: u32,
}

impl Default for BoxcarAccumulator8 {
    fn default() -> Self {
        Self::new()
    }
}

impl BoxcarAccumulator8 {
    pub const fn new() -> Self {
        Self {
            accel: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
            gyro: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
            count: 0,
            seq: 0,
        }
    }

    /// Push one raw sample; returns a decimated frame on every 8th call.
    pub fn push(&mut self, accel: Vector3, gyro: Vector3) -> Option<ImuDecimated> {
        self.accel.x += accel.x;
        self.accel.y += accel.y;
        self.accel.z += accel.z;
        self.gyro.x += gyro.x;
        self.gyro.y += gyro.y;
        self.gyro.z += gyro.z;
        self.count += 1;

        if self.count < DECIMATION {
            return None;
        }

        let k = 1.0 / DECIMATION as f32;
        self.seq = self.seq.wrapping_add(1);
        let out = ImuDecimated {
            accel: Vector3 { x: self.accel.x * k, y: self.accel.y * k, z: self.accel.z * k },
            gyro: Vector3 { x: self.gyro.x * k, y: self.gyro.y * k, z: self.gyro.z * k },
            dt: DECIMATED_DT_S,
            seq: self.seq,
        };
        self.accel = Vector3::ZERO;
        self.gyro = Vector3::ZERO;
        self.count = 0;
        Some(out)
    }

    /// Resets accumulator sums and count, preserving sequence count.
    pub fn reset(&mut self) {
        self.accel = Vector3::ZERO;
        self.gyro = Vector3::ZERO;
        self.count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f32, y: f32, z: f32) -> Vector3 {
        Vector3 { x, y, z }
    }

    #[test]
    fn emits_on_every_eighth_sample() {
        let mut acc = BoxcarAccumulator8::new();
        for _ in 0..7 {
            assert!(acc.push(v(0.0, 0.0, 1.0), v(0.0, 0.0, 0.0)).is_none());
        }
        let out = acc.push(v(0.0, 0.0, 1.0), v(0.0, 0.0, 0.0)).unwrap();
        assert!((out.accel.z - 1.0).abs() < 1e-6);
        assert_eq!(out.dt, DECIMATED_DT_S);
        // Restarts cleanly.
        assert!(acc.push(v(0.0, 0.0, 1.0), v(0.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn computes_exact_mean() {
        let mut acc = BoxcarAccumulator8::new();
        let mut out = None;
        for i in 0..8 {
            out = acc.push(v(i as f32, 0.0, 0.0), v(0.0, -(i as f32), 2.0));
        }
        let out = out.unwrap();
        assert!((out.accel.x - 3.5).abs() < 1e-6);
        assert!((out.gyro.y + 3.5).abs() < 1e-6);
        assert!((out.gyro.z - 2.0).abs() < 1e-6);
    }

    #[test]
    fn alternating_nyquist_noise_is_nulled() {
        let mut acc = BoxcarAccumulator8::new();
        let mut out = None;
        for i in 0..8 {
            let s = if i % 2 == 0 { 1.0 } else { -1.0 };
            out = acc.push(v(s, 0.0, 0.0), v(s, 0.0, 0.0));
        }
        assert!(out.unwrap().gyro.x.abs() < 1e-6);
    }

    #[test]
    fn seq_increments_monotonically() {
        let mut acc = BoxcarAccumulator8::new();
        let mut out = None;
        for _ in 0..8 {
            out = acc.push(v(0.0, 0.0, 1.0), v(0.0, 0.0, 0.0));
        }
        assert_eq!(out.unwrap().seq, 1);

        for _ in 0..8 {
            out = acc.push(v(0.0, 0.0, 1.0), v(0.0, 0.0, 0.0));
        }
        assert_eq!(out.unwrap().seq, 2);
    }
}

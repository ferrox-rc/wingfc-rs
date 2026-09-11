use core::ops::{Add, Mul, Sub};
use libm::{asinf, atan2f, sqrtf};

pub const PI: f32 = core::f32::consts::PI;
pub const DEG_TO_RAD: f32 = PI / 180.0;
pub const RAD_TO_DEG: f32 = 180.0 / PI;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    #[inline(always)]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    #[inline(always)]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline(always)]
    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    #[inline(always)]
    pub fn norm_sq(self) -> f32 {
        self.dot(self)
    }

    #[inline(always)]
    pub fn norm(self) -> f32 {
        sqrtf(self.norm_sq())
    }

    #[inline(always)]
    pub fn normalize(self) -> Self {
        let n = self.norm();
        if n > 1e-6 {
            let inv = 1.0 / n;
            Self {
                x: self.x * inv,
                y: self.y * inv,
                z: self.z * inv,
            }
        } else {
            Self::ZERO
        }
    }
}

impl Add for Vector3 {
    type Output = Self;
    #[inline(always)]
    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Sub for Vector3 {
    type Output = Self;
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Mul<f32> for Vector3 {
    type Output = Self;
    #[inline(always)]
    fn mul(self, rhs: f32) -> Self {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quaternion {
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    #[inline(always)]
    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    #[inline(always)]
    pub fn norm_sq(self) -> f32 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    #[inline(always)]
    pub fn norm(self) -> f32 {
        sqrtf(self.norm_sq())
    }

    #[inline(always)]
    pub fn normalize(self) -> Self {
        let n = self.norm();
        if n > 1e-6 {
            let inv = 1.0 / n;
            Self {
                w: self.w * inv,
                x: self.x * inv,
                y: self.y * inv,
                z: self.z * inv,
            }
        } else {
            Self::IDENTITY
        }
    }

    #[inline(always)]
    pub fn conjugate(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    #[inline(always)]
    pub fn multiply(self, rhs: Self) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }

    /// Converts quaternion to Euler Angles: (pitch, roll, yaw) in Radians.
    /// Euler sequence: Z-Y-X (Yaw-Pitch-Roll)
    pub fn to_euler_angles(self) -> (f32, f32, f32) {
        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = atan2f(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if sinp >= 1.0 {
            PI / 2.0
        } else if sinp <= -1.0 {
            -PI / 2.0
        } else {
            asinf(sinp)
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = atan2f(siny_cosp, cosy_cosp);

        (pitch, roll, yaw)
    }
}

#[inline(always)]
pub fn clamp(val: f32, min: f32, max: f32) -> f32 {
    if val < min {
        min
    } else if val > max {
        max
    } else {
        val
    }
}

#[inline(always)]
pub fn map_range(val: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    if (in_max - in_min).abs() < 1e-6 {
        return out_min;
    }
    (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

/// Precomputed linear range mapping: avoids runtime division when input and output scale factors are known.
///
/// `inv_in_range` is `1.0 / (in_max - in_min)` and `out_range` is `(out_max - out_min)`.
#[inline(always)]
pub fn map_range_precomputed(
    val: f32,
    in_min: f32,
    inv_in_range: f32,
    out_min: f32,
    out_range: f32,
) -> f32 {
    (val - in_min) * inv_in_range * out_range + out_min
}

/// Board mounting orientation transformations.
/// Maps sensor axes (X=forward, Y=right, Z=down or sensor frame) to standard aircraft body frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum BoardOrientation {
    #[default]
    Default = 0,
    CW90 = 1,
    CW180 = 2,
    CW270 = 3,
    Flip = 4,
    FlipCW90 = 5,
    FlipCW180 = 6,
    FlipCW270 = 7,
}

impl BoardOrientation {
    /// Transforms a 3D vector from sensor frame to aircraft frame according to board orientation.
    #[inline(always)]
    pub fn transform(&self, v: Vector3) -> Vector3 {
        match self {
            BoardOrientation::Default => Vector3::new(v.x, v.y, v.z),
            BoardOrientation::CW90 => Vector3::new(v.y, -v.x, v.z),
            BoardOrientation::CW180 => Vector3::new(-v.x, -v.y, v.z),
            BoardOrientation::CW270 => Vector3::new(-v.y, v.x, v.z),
            BoardOrientation::Flip => Vector3::new(v.x, -v.y, -v.z),
            BoardOrientation::FlipCW90 => Vector3::new(-v.y, -v.x, -v.z),
            BoardOrientation::FlipCW180 => Vector3::new(-v.x, v.y, -v.z),
            BoardOrientation::FlipCW270 => Vector3::new(v.y, v.x, -v.z),
        }
    }
}

//! Magnetometer calibration and tilt-compensated heading.
//!
//! Frame convention: aerospace body frame (x forward, y right, z down) with roll `phi`
//! about x and pitch `theta` about y. Heading is returned in radians, wrapped to
//! `(-pi, pi]`, increasing clockwise from magnetic north when viewed from above.

use crate::math::Vector3;

const PI: f32 = core::f32::consts::PI;
const TWO_PI: f32 = 2.0 * PI;

/// Hard-iron offset plus 3x3 soft-iron correction: `m_cal = S * (m_raw - offset)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagCalibration {
    pub hard_iron: Vector3,
    pub soft_iron: [[f32; 3]; 3],
}

impl MagCalibration {
    pub const IDENTITY: Self = Self {
        hard_iron: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        soft_iron: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    /// Apply the calibration to a raw magnetometer vector.
    pub fn apply(&self, raw: Vector3) -> Vector3 {
        let v = [
            raw.x - self.hard_iron.x,
            raw.y - self.hard_iron.y,
            raw.z - self.hard_iron.z,
        ];
        let s = &self.soft_iron;
        Vector3 {
            x: s[0][0] * v[0] + s[0][1] * v[1] + s[0][2] * v[2],
            y: s[1][0] * v[0] + s[1][1] * v[1] + s[1][2] * v[2],
            z: s[2][0] * v[0] + s[2][1] * v[1] + s[2][2] * v[2],
        }
    }
}

impl Default for MagCalibration {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// Min/max tracker: while the aircraft is rotated through all orientations it yields a
/// hard-iron offset (mid-point) and a diagonal soft-iron scale (axis range equalisation).
#[derive(Debug, Clone, Copy)]
pub struct MagCalibrator {
    min: [f32; 3],
    max: [f32; 3],
    samples: u32,
}

impl Default for MagCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

impl MagCalibrator {
    pub const fn new() -> Self {
        Self {
            min: [f32::MAX; 3],
            max: [f32::MIN; 3],
            samples: 0,
        }
    }

    pub fn update(&mut self, m: Vector3) {
        let v = [m.x, m.y, m.z];
        for (i, &val) in v.iter().enumerate() {
            if val < self.min[i] {
                self.min[i] = val;
            }
            if val > self.max[i] {
                self.max[i] = val;
            }
        }
        self.samples += 1;
    }

    pub fn samples(&self) -> u32 {
        self.samples
    }

    /// Returns a calibration, or `None` if any axis range is degenerate (insufficient rotation).
    pub fn finish(&self, min_range: f32) -> Option<MagCalibration> {
        if self.samples == 0 {
            return None;
        }
        let mut radius = [0.0f32; 3];
        for (i, r) in radius.iter_mut().enumerate() {
            *r = 0.5 * (self.max[i] - self.min[i]);
            if *r < min_range {
                return None;
            }
        }
        let avg = (radius[0] + radius[1] + radius[2]) / 3.0;
        let off = |i: usize| 0.5 * (self.max[i] + self.min[i]);
        Some(MagCalibration {
            hard_iron: Vector3 {
                x: off(0),
                y: off(1),
                z: off(2),
            },
            soft_iron: [
                [avg / radius[0], 0.0, 0.0],
                [0.0, avg / radius[1], 0.0],
                [0.0, 0.0, avg / radius[2]],
            ],
        })
    }
}

/// Wrap an angle to `(-pi, pi]`.
pub fn wrap_pi(mut a: f32) -> f32 {
    while a > PI {
        a -= TWO_PI;
    }
    while a <= -PI {
        a += TWO_PI;
    }
    a
}

/// Project a calibrated magnetic vector onto the horizontal plane using roll/pitch and
/// return `(Xh, Yh)`.
pub fn horizontal_components(mag: Vector3, roll: f32, pitch: f32) -> (f32, f32) {
    let (sp, cp) = (libm::sinf(pitch), libm::cosf(pitch));
    let (sr, cr) = (libm::sinf(roll), libm::cosf(roll));
    let xh = mag.x * cp + mag.y * sp * sr + mag.z * sp * cr;
    let yh = mag.y * cr - mag.z * sr;
    (xh, yh)
}

/// Tilt-compensated magnetic heading plus declination, wrapped to `(-pi, pi]`.
pub fn tilt_compensated_heading(mag: Vector3, roll: f32, pitch: f32, declination: f32) -> f32 {
    let (xh, yh) = horizontal_components(mag, roll, pitch);
    wrap_pi(libm::atan2f(-yh, xh) + declination)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f32, y: f32, z: f32) -> Vector3 {
        Vector3 { x, y, z }
    }

    /// Rotate a NED-frame field into the body frame (yaw psi, pitch theta, roll phi).
    fn ned_to_body(n: [f32; 3], phi: f32, theta: f32, psi: f32) -> Vector3 {
        let (sp, cp) = (libm::sinf(phi), libm::cosf(phi));
        let (st, ct) = (libm::sinf(theta), libm::cosf(theta));
        let (ss, cs) = (libm::sinf(psi), libm::cosf(psi));
        // Body = R^T * NED, with R = Rz(psi) Ry(theta) Rx(phi)
        let x = ct * cs * n[0] + ct * ss * n[1] - st * n[2];
        let y = (sp * st * cs - cp * ss) * n[0] + (sp * st * ss + cp * cs) * n[1] + sp * ct * n[2];
        let z = (cp * st * cs + sp * ss) * n[0] + (cp * st * ss - sp * cs) * n[1] + cp * ct * n[2];
        v(x, y, z)
    }

    #[test]
    fn level_heading_cardinals() {
        let north = tilt_compensated_heading(v(1.0, 0.0, 0.0), 0.0, 0.0, 0.0);
        assert!(north.abs() < 1e-5);
        // Field pointing out the left wing (-y) means the nose points east.
        let east = tilt_compensated_heading(v(0.0, -1.0, 0.0), 0.0, 0.0, 0.0);
        assert!((east - PI / 2.0).abs() < 1e-5);
    }

    #[test]
    fn tilt_compensation_recovers_yaw() {
        let field = [0.2, 0.0, 0.45]; // typical northern-hemisphere dip
        let deg = PI / 180.0;
        for &(phi, theta) in &[
            (0.0, 0.0),
            (30.0 * deg, 0.0),
            (0.0, -30.0 * deg),
            (45.0 * deg, 45.0 * deg),
        ] {
            for &psi in &[0.0, 1.0, -2.0, 3.0] {
                let body = ned_to_body(field, phi, theta, psi);
                let h = tilt_compensated_heading(body, phi, theta, 0.0);
                assert!(
                    wrap_pi(h - psi).abs() < 1e-3,
                    "phi={phi} theta={theta} psi={psi} h={h}"
                );
            }
        }
    }

    #[test]
    fn declination_is_added_and_wrapped() {
        let h = tilt_compensated_heading(v(-1.0, 0.0, 0.0), 0.0, 0.0, 0.2);
        // atan2(0, -1) = pi; + 0.2 wraps to -pi + 0.2
        assert!((h - (-PI + 0.2)).abs() < 1e-5);
    }

    #[test]
    fn calibrator_finds_offset_and_scale() {
        let mut cal = MagCalibrator::new();
        // Ellipsoid centred at (0.1, -0.2, 0.3) with radii (0.5, 1.0, 0.25).
        for &(sx, sy, sz) in &[
            (1.0, 0.0, 0.0),
            (-1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, 0.0, -1.0),
        ] {
            cal.update(v(0.1 + 0.5 * sx, -0.2 + 1.0 * sy, 0.3 + 0.25 * sz));
        }
        let c = cal.finish(0.01).unwrap();
        assert!((c.hard_iron.x - 0.1).abs() < 1e-6);
        assert!((c.hard_iron.y + 0.2).abs() < 1e-6);
        assert!((c.hard_iron.z - 0.3).abs() < 1e-6);
        let out = c.apply(v(0.1 + 0.5, -0.2, 0.3));
        let out_y = c.apply(v(0.1, -0.2 + 1.0, 0.3));
        let out_z = c.apply(v(0.1, -0.2, 0.3 + 0.25));
        assert!((out.x - out_y.y).abs() < 1e-5 && (out_y.y - out_z.z).abs() < 1e-5);
    }

    #[test]
    fn calibrator_rejects_insufficient_rotation() {
        let mut cal = MagCalibrator::new();
        cal.update(v(0.0, 0.0, 0.0));
        cal.update(v(1.0, 0.0, 0.0));
        assert!(cal.finish(0.1).is_none());
        assert!(MagCalibrator::new().finish(0.1).is_none());
    }
}

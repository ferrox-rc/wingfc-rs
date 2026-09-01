use crate::math::{Vector3, Quaternion};
use libm::sqrtf;

/// Madgwick 6-DoF AHRS with Dynamic G-Rejection
/// Designed for high-speed aerobatics and high-G fixed-wing maneuvers.
#[derive(Debug, Clone, Copy)]
pub struct MadgwickAHRS {
    pub q: Quaternion,
    pub beta: f32,
    /// Acceleration rejection window: [1.0 - g_window, 1.0 + g_window] in g's
    pub g_window: f32,
}

impl MadgwickAHRS {
    pub const fn new(beta: f32) -> Self {
        Self {
            q: Quaternion::IDENTITY,
            beta,
            g_window: 0.35, // Reject accel corrections outside [0.65g, 1.35g]
        }
    }

    pub fn reset(&mut self) {
        self.q = Quaternion::IDENTITY;
    }

    /// Update 6-DoF AHRS with Gyro (rad/s) and Accel (m/s^2 or g)
    pub fn update_6dof(&mut self, gyro: Vector3, accel: Vector3, dt: f32) {
        let mut q = self.q;

        // Rate of change of quaternion from gyroscope
        let q_dot_omega = Quaternion::new(
            0.5 * (-q.x * gyro.x - q.y * gyro.y - q.z * gyro.z),
            0.5 * ( q.w * gyro.x + q.y * gyro.z - q.z * gyro.y),
            0.5 * ( q.w * gyro.y - q.x * gyro.z + q.z * gyro.x),
            0.5 * ( q.w * gyro.z + q.x * gyro.y - q.y * gyro.x),
        );

        let accel_norm_sq = accel.norm_sq();
        // Dynamic G-Rejection: evaluate if accel magnitude is close to 1G (e.g. ~9.81 m/s^2 or ~1.0g)
        let is_accel_valid = if accel_norm_sq > 0.01 {
            let a_norm = sqrtf(accel_norm_sq);
            let a_g = if a_norm > 5.0 { a_norm / 9.80665 } else { a_norm };
            (a_g >= (1.0 - self.g_window)) && (a_g <= (1.0 + self.g_window))
        } else {
            false
        };

        if is_accel_valid {
            let a = accel.normalize();

            // Objective function f_g(q, a)
            let f1 = 2.0 * (q.x * q.z - q.w * q.y) - a.x;
            let f2 = 2.0 * (q.w * q.x + q.y * q.z) - a.y;
            let f3 = 2.0 * (0.5 - q.x * q.x - q.y * q.y) - a.z;

            // Jacobian transposed J_g^T * f_g
            let s_w = -2.0 * q.y * f1 + 2.0 * q.x * f2;
            let s_x =  2.0 * q.z * f1 + 2.0 * q.w * f2 - 4.0 * q.x * f3;
            let s_y = -2.0 * q.w * f1 + 2.0 * q.z * f2 - 4.0 * q.y * f3;
            let s_z =  2.0 * q.x * f1 + 2.0 * q.y * f2;

            // Normalize gradient step
            let s_norm = sqrtf(s_w * s_w + s_x * s_x + s_y * s_y + s_z * s_z);
            if s_norm > 1e-6 {
                let inv_s = 1.0 / s_norm;
                let step_w = s_w * inv_s;
                let step_x = s_x * inv_s;
                let step_y = s_y * inv_s;
                let step_z = s_z * inv_s;

                // Apply gradient descent correction
                q.w += (q_dot_omega.w - self.beta * step_w) * dt;
                q.x += (q_dot_omega.x - self.beta * step_x) * dt;
                q.y += (q_dot_omega.y - self.beta * step_y) * dt;
                q.z += (q_dot_omega.z - self.beta * step_z) * dt;
            } else {
                q.w += q_dot_omega.w * dt;
                q.x += q_dot_omega.x * dt;
                q.y += q_dot_omega.y * dt;
                q.z += q_dot_omega.z * dt;
            }
        } else {
            // High-G or Zero-G acrobatic maneuver: pure gyro integration (no centripetal distortion)
            q.w += q_dot_omega.w * dt;
            q.x += q_dot_omega.x * dt;
            q.y += q_dot_omega.y * dt;
            q.z += q_dot_omega.z * dt;
        }

        self.q = q.normalize();
    }

    #[inline(always)]
    pub fn get_euler_angles(&self) -> (f32, f32, f32) {
        self.q.to_euler_angles()
    }
}

/// 2-State Vertical Kalman Filter for Altitude and Climb/Sink Rate Estimation.
/// Fuses high-rate (500Hz) vertical accelerometer data with lower-rate (20-50Hz) Barometer altitude.
///
/// State Vector: X = [ Altitude (meters), Vertical Velocity (m/s) ]^T

#[derive(Debug, Clone, Copy)]
pub struct VerticalKalmanFilter {
    pub altitude_m: f32,
    pub velocity_ms: f32,
    // Error covariance matrix P (symmetric 2x2)
    p00: f32,
    p01: f32,
    p11: f32,
    // Process noise Q
    q_accel: f32,
    // Measurement noise R (barometer variance)
    r_baro: f32,
}

impl Default for VerticalKalmanFilter {
    fn default() -> Self {
        Self::new(0.04, 0.25)
    }
}

impl VerticalKalmanFilter {
    pub const fn new(q_accel: f32, r_baro: f32) -> Self {
        Self {
            altitude_m: 0.0,
            velocity_ms: 0.0,
            p00: 1.0,
            p01: 0.0,
            p11: 1.0,
            q_accel,
            r_baro,
        }
    }

    pub fn reset(&mut self, initial_altitude_m: f32) {
        self.altitude_m = initial_altitude_m;
        self.velocity_ms = 0.0;
        self.p00 = 1.0;
        self.p01 = 0.0;
        self.p11 = 1.0;
    }

    /// High-rate prediction step driven by earth-frame vertical acceleration (m/s^2, gravity removed)
    /// Runs every flight control loop cycle (e.g. 500Hz)
    pub fn predict(&mut self, vertical_accel_ms2: f32, dt: f32) {
        // State extrapolation:
        // alt = alt + vel * dt + 0.5 * accel * dt^2
        // vel = vel + accel * dt
        let dt_sq_half = 0.5 * dt * dt;
        self.altitude_m += self.velocity_ms * dt + vertical_accel_ms2 * dt_sq_half;
        self.velocity_ms += vertical_accel_ms2 * dt;

        // Covariance extrapolation: P = F * P * F^T + Q
        // F = [ [1, dt], [0, 1] ]
        // G = [ 0.5*dt^2, dt ]^T
        // Q = G * G^T * q_accel
        let dt_p11 = dt * self.p11;
        self.p00 += dt * (self.p01 + self.p01 + dt_p11) + dt_sq_half * dt_sq_half * self.q_accel;
        self.p01 += dt_p11 + dt_sq_half * dt * self.q_accel;
        self.p11 += dt * dt * self.q_accel;
    }

    /// Measurement update step driven by barometer relative altitude (meters)
    /// Runs whenever a new barometer pressure reading is available (e.g. 20-50Hz)
    pub fn update_baro(&mut self, baro_altitude_m: f32) {
        // Innovation: y = z - H * X (where H = [1, 0])
        let y = baro_altitude_m - self.altitude_m;

        // Innovation covariance: S = H * P * H^T + R = P00 + R
        let s = self.p00 + self.r_baro;
        if s.abs() < 1e-6 {
            return;
        }
        let inv_s = 1.0 / s;

        // Kalman gain: K = P * H^T * S^-1 = [ P00/S, P01/S ]^T
        let k0 = self.p00 * inv_s;
        let k1 = self.p01 * inv_s;

        // State update: X = X + K * y
        self.altitude_m += k0 * y;
        self.velocity_ms += k1 * y;

        // Covariance update: P = (I - K * H) * P
        let p00_temp = self.p00;
        let p01_temp = self.p01;

        self.p00 -= k0 * p00_temp;
        self.p01 -= k0 * p01_temp;
        self.p11 -= k1 * p01_temp;
    }
}

use crate::filter::Pt1Filter;
use crate::math::{clamp, Quaternion, Vector3, PI};
use libm::{asinf, atan2f, cosf, fabsf, sinf, sqrtf};

const TWO_PI: f32 = 2.0 * PI;

/// Wrap angle in radians to `(-pi, pi]`.
#[inline(always)]
fn wrap_pi(mut a: f32) -> f32 {
    while a > PI {
        a -= TWO_PI;
    }
    while a <= -PI {
        a += TWO_PI;
    }
    a
}

/// Versatile Quaternion-based Filter (VQF) AHRS (Laidig & Seel, 2023).
///
/// Features:
/// - Strapdown gyro integration.
/// - Acceleration filtering in the almost-inertial frame using cascaded `Pt1Filter`s.
/// - Strict decoupling between 6-DoF inclination (roll/pitch) and 9-DoF heading (yaw).
/// - Online rest detection and stationary gyroscope bias tracking.
/// - Magnetic disturbance rejection via norm and dip-angle monitoring.
/// - 100% single-precision (`f32`) for deterministic Cortex-M4F hardware FPU execution.
#[derive(Debug, Clone, Copy)]
pub struct VqfAHRS {
    // Quaternions
    /// Strapdown gyro integration quaternion: S_i -> I_i
    pub gyr_quat: Quaternion,
    /// Inclination correction quaternion: I_i -> E_i
    pub acc_quat: Quaternion,
    /// Heading difference between 6D Earth frame and magnetic reference: E_i -> E
    pub delta: f32,

    // Time constants
    pub tau_acc: f32,
    pub tau_mag: f32,

    // Inertial acceleration filtering (2-stage cascaded Pt1Filter per axis)
    acc_lp_stage1: [Pt1Filter; 3],
    acc_lp_stage2: [Pt1Filter; 3],
    acc_initialized: bool,

    // Online Gyro Bias & Rest Detection
    pub bias: Vector3,
    pub bias_clip: f32,
    pub rest_detected: bool,
    rest_t: f32,
    rest_min_t: f32,
    rest_th_gyr_sq: f32,
    rest_th_acc_sq: f32,
    rest_gyr_lp: [Pt1Filter; 3],
    rest_acc_lp: [Pt1Filter; 3],

    // Magnetic Disturbance Rejection
    pub mag_dist_detected: bool,
    mag_initialized: bool,
    mag_ref_norm: f32,
    mag_ref_dip: f32,
    mag_undisturbed_t: f32,
    mag_min_undisturbed_t: f32,
    mag_norm_th: f32,
    mag_dip_th: f32,
    k_mag_init: f32,
    pub declination: f32,
}

impl VqfAHRS {
    /// Creates a new VqfAHRS instance.
    ///
    /// - `tau_acc`: Time constant for accelerometer filtering in seconds (nominal: 3.5s).
    /// - `tau_mag`: Time constant for magnetometer heading correction in seconds (nominal: 9.0s).
    /// - `dt`: Nominal flight loop sample time (e.g. 0.0048s for 208.33 Hz).
    pub fn new(tau_acc: f32, tau_mag: f32, dt: f32) -> Self {
        let mut s = Self {
            gyr_quat: Quaternion::IDENTITY,
            acc_quat: Quaternion::IDENTITY,
            delta: 0.0,
            tau_acc,
            tau_mag,
            acc_lp_stage1: [Pt1Filter::new(); 3],
            acc_lp_stage2: [Pt1Filter::new(); 3],
            acc_initialized: false,
            bias: Vector3::ZERO,
            bias_clip: 2.0 * (PI / 180.0), // 2.0 deg/s in rad/s
            rest_detected: false,
            rest_t: 0.0,
            rest_min_t: 1.5,
            rest_th_gyr_sq: (2.0 * (PI / 180.0)) * (2.0 * (PI / 180.0)),
            rest_th_acc_sq: 0.5 * 0.5,
            rest_gyr_lp: [Pt1Filter::new(); 3],
            rest_acc_lp: [Pt1Filter::new(); 3],
            mag_dist_detected: false,
            mag_initialized: false,
            mag_ref_norm: 0.0,
            mag_ref_dip: 0.0,
            mag_undisturbed_t: 0.0,
            mag_min_undisturbed_t: 0.5,
            mag_norm_th: 0.10,               // 10% deviation threshold
            mag_dip_th: 10.0 * (PI / 180.0), // 10 deg threshold
            k_mag_init: 1.0,
            declination: 0.0,
        };

        // Initialize inertial filter stages
        // fc = sqrt(2) / (2 * pi * tau_acc)
        // Stage cutoff for 2-stage cascaded filter: fc_stage = fc * 1.554
        let fc = sqrtf(2.0) / (TWO_PI * tau_acc);
        let stage_cutoff = fc * 1.554;
        for i in 0..3 {
            s.acc_lp_stage1[i].init(stage_cutoff, dt);
            s.acc_lp_stage2[i].init(stage_cutoff, dt);
            s.rest_gyr_lp[i].init(2.0, dt); // 2.0 Hz cutoff for rest tracking
            s.rest_acc_lp[i].init(2.0, dt);
        }

        s
    }

    /// Resets orientation states, bias, and filters.
    pub fn reset(&mut self) {
        self.gyr_quat = Quaternion::IDENTITY;
        self.acc_quat = Quaternion::IDENTITY;
        self.delta = 0.0;
        self.bias = Vector3::ZERO;
        self.acc_initialized = false;
        self.rest_detected = false;
        self.rest_t = 0.0;
        self.mag_dist_detected = false;
        self.mag_initialized = false;
        self.k_mag_init = 1.0;
    }

    /// Sets magnetic declination in radians.
    pub fn set_declination(&mut self, declination_rad: f32) {
        self.declination = declination_rad;
    }

    /// Updates 6-DoF AHRS using high-rate IMU data (called at 208.33 Hz).
    ///
    /// - `raw_gyro`: Angular velocity in rad/s (aircraft body frame).
    /// - `accel`: Specific force in m/s^2 (aircraft body frame, level upright ≈ [0, 0, +9.81]).
    /// - `dt`: Elapsed loop time in seconds.
    pub fn update_6dof(&mut self, raw_gyro: Vector3, accel: Vector3, dt: f32) {
        self.update_gyr(raw_gyro, dt);
        self.update_acc(accel, dt);
    }

    /// Updates 6-DoF AHRS with GPS groundspeed centripetal turn compensation.
    ///
    /// Fixed-wing coordinated turns produce sustained non-gravitational lateral and vertical
    /// acceleration (a_c = omega x [v, 0, 0]). Subtracting this kinematic term prevents the
    /// filtered gravity vector from tilting toward the aircraft's lift vector during long loiters.
    pub fn update_6dof_centripetal_aided(
        &mut self,
        raw_gyro: Vector3,
        accel: Vector3,
        groundspeed_mps: f32,
        dt: f32,
    ) {
        if groundspeed_mps > 1.0 {
            let gyro_body = raw_gyro - self.bias;
            let a_centripetal = Vector3::new(
                0.0,
                -gyro_body.z * groundspeed_mps,
                gyro_body.y * groundspeed_mps,
            );
            let accel_gravity = accel - a_centripetal;
            self.update_6dof(raw_gyro, accel_gravity, dt);
        } else {
            self.update_6dof(raw_gyro, accel, dt);
        }
    }

    /// Gyroscope strapdown integration and rest detection.
    pub fn update_gyr(&mut self, raw_gyro: Vector3, dt: f32) {
        // 1. Rest detection filter on gyro
        let gyr_filt = Vector3::new(
            self.rest_gyr_lp[0].update(raw_gyro.x),
            self.rest_gyr_lp[1].update(raw_gyro.y),
            self.rest_gyr_lp[2].update(raw_gyro.z),
        );
        let d_gx = raw_gyro.x - gyr_filt.x;
        let d_gy = raw_gyro.y - gyr_filt.y;
        let d_gz = raw_gyro.z - gyr_filt.z;
        let gyr_dev_sq = d_gx * d_gx + d_gy * d_gy + d_gz * d_gz;

        if gyr_dev_sq >= self.rest_th_gyr_sq
            || fabsf(gyr_filt.x) > self.bias_clip
            || fabsf(gyr_filt.y) > self.bias_clip
            || fabsf(gyr_filt.z) > self.bias_clip
        {
            self.rest_t = 0.0;
            self.rest_detected = false;
        }

        // 2. Bias-corrected angular rate
        let gyro = raw_gyro - self.bias;

        // 3. Strapdown quaternion propagation
        let gyr_norm = gyro.norm();
        if gyr_norm > 1e-6 {
            let half_angle = 0.5 * gyr_norm * dt;
            let c = cosf(half_angle);
            let s = sinf(half_angle) / gyr_norm;
            let step = Quaternion::new(c, s * gyro.x, s * gyro.y, s * gyro.z);
            self.gyr_quat = self.gyr_quat.multiply(step).normalize();
        }
    }

    /// Accelerometer inertial-frame filtering and inclination correction.
    pub fn update_acc(&mut self, accel: Vector3, dt: f32) {
        let acc_norm_sq = accel.norm_sq();
        if acc_norm_sq < 0.01 {
            return;
        }

        // 1. Rest detection on accel
        let acc_filt = Vector3::new(
            self.rest_acc_lp[0].update(accel.x),
            self.rest_acc_lp[1].update(accel.y),
            self.rest_acc_lp[2].update(accel.z),
        );
        let d_ax = accel.x - acc_filt.x;
        let d_ay = accel.y - acc_filt.y;
        let d_az = accel.z - acc_filt.z;
        let acc_dev_sq = d_ax * d_ax + d_ay * d_ay + d_az * d_az;

        if acc_dev_sq >= self.rest_th_acc_sq {
            self.rest_t = 0.0;
            self.rest_detected = false;
        } else {
            self.rest_t += dt;
            if self.rest_t >= self.rest_min_t {
                self.rest_detected = true;
            }
        }

        // 2. Stationary gyro bias update
        if self.rest_detected {
            // Track bias during confirmed rest (~0.8 s time constant for fast pre-flight alignment)
            let k_bias = dt / (0.8 + dt);
            let gyr_lp = Vector3::new(
                self.rest_gyr_lp[0].state(),
                self.rest_gyr_lp[1].state(),
                self.rest_gyr_lp[2].state(),
            );
            self.bias.x = clamp(
                self.bias.x + k_bias * (gyr_lp.x - self.bias.x),
                -self.bias_clip,
                self.bias_clip,
            );
            self.bias.y = clamp(
                self.bias.y + k_bias * (gyr_lp.y - self.bias.y),
                -self.bias_clip,
                self.bias_clip,
            );
            self.bias.z = clamp(
                self.bias.z + k_bias * (gyr_lp.z - self.bias.z),
                -self.bias_clip,
                self.bias_clip,
            );
        }

        // 3. Transform body accel into the almost-inertial frame
        let acc_inertial = self.gyr_quat.rotate_vector(accel);

        // Initialize filters on first valid sample to avoid long settling transient
        if !self.acc_initialized {
            let v = [acc_inertial.x, acc_inertial.y, acc_inertial.z];
            for i in 0..3 {
                self.acc_lp_stage1[i].reset(v[i]);
                self.acc_lp_stage2[i].reset(v[i]);
            }
            self.acc_initialized = true;
        }

        // 4. Low-pass filter in the almost-inertial frame (2-stage cascaded Pt1)
        let acc_inertial_filt = Vector3::new(
            self.acc_lp_stage2[0].update(self.acc_lp_stage1[0].update(acc_inertial.x)),
            self.acc_lp_stage2[1].update(self.acc_lp_stage1[1].update(acc_inertial.y)),
            self.acc_lp_stage2[2].update(self.acc_lp_stage1[2].update(acc_inertial.z)),
        );

        // 5. Transform into 6D Earth frame
        let acc_earth = self.acc_quat.rotate_vector(acc_inertial_filt).normalize();

        // 6. Inclination correction step
        // In upright level flight, acc_earth ≈ [0, 0, 1].
        let q_w = sqrtf((acc_earth.z + 1.0) * 0.5);
        let acc_corr_quat = if q_w > 1e-6 {
            let inv_2qw = 0.5 / q_w;
            Quaternion::new(q_w, acc_earth.y * inv_2qw, -acc_earth.x * inv_2qw, 0.0)
        } else {
            // Singularity guard when inverted 180 degrees
            Quaternion::new(0.0, 1.0, 0.0, 0.0)
        };

        self.acc_quat = acc_corr_quat.multiply(self.acc_quat).normalize();
    }

    /// Asynchronous 20 Hz magnetometer update with disturbance rejection.
    ///
    /// - `mag`: Calibrated magnetometer vector (aircraft body frame).
    /// - `dt_mag`: Elapsed time since last magnetometer reading in seconds (e.g. 0.05s).
    pub fn update_mag(&mut self, mag: Vector3, dt_mag: f32) {
        let mag_norm_sq = mag.norm_sq();
        if mag_norm_sq < 1e-4 {
            return;
        }

        // 1. Rotate body mag into 6D Earth frame
        let q_6d = self.get_quat_6d();
        let mag_earth = q_6d.rotate_vector(mag);
        let mag_norm = sqrtf(mag_norm_sq);

        // 2. Dip angle (inclination angle of magnetic field relative to horizontal plane)
        // dip = -asin(mag_earth.z / norm)
        let dip_ratio = clamp(mag_earth.z / mag_norm, -1.0, 1.0);
        let mag_dip = -asinf(dip_ratio);

        // Initialize reference field on first reading
        if !self.mag_initialized {
            self.mag_ref_norm = mag_norm;
            self.mag_ref_dip = mag_dip;
            self.mag_initialized = true;
        }

        // 3. Magnetic disturbance detection (compare norm and dip against reference)
        let norm_err = fabsf(mag_norm - self.mag_ref_norm) / self.mag_ref_norm;
        let dip_err = fabsf(mag_dip - self.mag_ref_dip);

        let is_undisturbed = norm_err < self.mag_norm_th && dip_err < self.mag_dip_th;

        if is_undisturbed {
            self.mag_undisturbed_t += dt_mag;
            if self.mag_undisturbed_t >= self.mag_min_undisturbed_t {
                self.mag_dist_detected = false;
                // Slowly adapt magnetic reference to steady field changes (~20s time constant)
                let k_ref = dt_mag / (20.0 + dt_mag);
                self.mag_ref_norm += k_ref * (mag_norm - self.mag_ref_norm);
                self.mag_ref_dip += k_ref * (mag_dip - self.mag_ref_dip);
            }
        } else {
            self.mag_undisturbed_t = 0.0;
            self.mag_dist_detected = true;
        }

        // 4. Calculate heading disagreement angle
        // In aerospace convention (x forward/North, y right/East):
        // Nose pointing North: field along +x, heading = 0.
        // Nose pointing East: field along left wing (-y), heading = +pi/2.
        // Heading psi = atan2(-mag_earth.y, mag_earth.x) + declination
        let mag_heading = atan2f(-mag_earth.y, mag_earth.x) + self.declination;
        let mag_dis_angle = wrap_pi(mag_heading - self.delta);

        // 5. Heading filter gain
        let k = if self.mag_dist_detected {
            0.0 // Reject completely during magnetic anomaly
        } else {
            let normal_k = 1.0 - libm::expf(-dt_mag / self.tau_mag);
            // Fast initial convergence: k >= 1/N for initial undisturbed samples
            if self.k_mag_init > 0.0 {
                let init_k = self.k_mag_init;
                self.k_mag_init = self.k_mag_init / (self.k_mag_init + 1.0);
                if self.k_mag_init * self.tau_mag < dt_mag {
                    self.k_mag_init = 0.0;
                }
                if normal_k < init_k {
                    init_k
                } else {
                    normal_k
                }
            } else {
                normal_k
            }
        };

        // 6. First-order delta update step
        self.delta = wrap_pi(self.delta + k * mag_dis_angle);
    }

    /// Combined 6-DoF orientation quaternion: `q_6d = q_acc * q_gyr`.
    #[inline(always)]
    pub fn get_quat_6d(&self) -> Quaternion {
        self.acc_quat.multiply(self.gyr_quat)
    }

    /// Full 9-DoF orientation quaternion: `q_9d = q_mag * q_acc * q_gyr`.
    #[inline(always)]
    pub fn quaternion(&self) -> Quaternion {
        let half_delta = 0.5 * self.delta;
        let q_mag = Quaternion::new(cosf(half_delta), 0.0, 0.0, sinf(half_delta));
        q_mag.multiply(self.get_quat_6d()).normalize()
    }

    /// Returns Euler Angles: `(pitch, roll, yaw)` in radians.
    ///
    /// - `pitch`: Elevation angle about lateral y-axis.
    /// - `roll`: Bank angle about longitudinal x-axis.
    /// - `yaw`: Tilt-compensated heading about vertical z-axis.
    #[inline(always)]
    pub fn euler_angles(&self) -> (f32, f32, f32) {
        self.quaternion().to_euler_angles()
    }
}

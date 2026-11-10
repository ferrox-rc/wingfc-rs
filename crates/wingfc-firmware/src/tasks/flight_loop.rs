use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Receiver;
#[cfg(any(feature = "baro", feature = "compass"))]
use embassy_sync::watch::Receiver as WatchReceiver;
use embassy_sync::watch::Sender;
use embassy_time::Instant;
#[cfg(not(feature = "dshot"))]
use embassy_time::{Duration, Timer};
use wingfc_core::ahrs::VqfAHRS;
use wingfc_core::filter::Pt1Filter;
use wingfc_core::math::Vector3;

use wingfc_core::mixer::{AirframeMixer, AirframeOutputs, AirframeType};
use wingfc_core::pid::PidController;
use wingfc_core::protocol::RcChannels;
use wingfc_core::safety::{ArmingSafety, ArmingStatus};

use crate::config::{
    AIRFRAME_TYPE, ANGLE_P_GAIN, ARM_SWITCH_THRESHOLD, DIFFERENTIAL_THRUST, DT, FAILSAFE_TIMEOUT,
    GYRO_LPF_CUTOFF_HZ, IN_FLIGHT_TUNING, IN_FLIGHT_TUNING_ENABLED, MAX_PITCH_ANGLE,
    MAX_PITCH_RATE, MAX_ROLL_ANGLE, MAX_ROLL_RATE, MAX_YAW_RATE, MODE_MATRIX, RATE_PID_CONFIG,
    RC_CH_ARM, RC_CH_PITCH, RC_CH_ROLL, RC_CH_THROTTLE, RC_CH_YAW, SERVO_CONFIGS,
    THROTTLE_ZERO_THRESHOLD, VQF_TAU_ACC, VQF_TAU_MAG,
};
#[cfg(not(feature = "dshot"))]
use wingfc_drivers::MotorOutput;
use wingfc_drivers::Lsm6ds3Driver;
use crate::tasks::telemetry::VisualStatus;
#[cfg(any(feature = "baro", feature = "compass"))]
use crate::tasks::ExternalSensorSnapshot;
#[cfg(feature = "baro")]
use wingfc_core::mode::AltHoldController;
#[cfg(feature = "compass")]
use wingfc_core::mode::HeadingHoldController;
#[cfg(any(feature = "baro", feature = "compass"))]
use wingfc_core::sensor_health::SensorHealth;
#[cfg(feature = "baro")]
use wingfc_core::vertical_kf::VerticalKalmanFilter;

/// Runs boot-time ESC calibration check if PWM is used (Universal Rule #3).
#[cfg(not(feature = "dshot"))]
pub async fn calibrate_esc_if_requested<M: MotorOutput>(
    rc_rx: &Receiver<'static, ThreadModeRawMutex, RcChannels, 2>,
    status_tx: &Sender<'static, ThreadModeRawMutex, VisualStatus, 2>,
    esc_pwm: &mut M,
) -> Option<RcChannels> {
    let start = Instant::now();
    let mut in_esc_calib = false;
    let mut latest_rc = None;

    while start.elapsed() < Duration::from_millis(600) {
        if let Ok(rc) = rc_rx.try_receive() {
            latest_rc = Some(rc);
            if rc.channels[RC_CH_THROTTLE] > 1900 {
                in_esc_calib = true;
                break;
            }
        }
        Timer::after(Duration::from_millis(20)).await;
    }

    if in_esc_calib {
        defmt::warn!(">>> ESC Throttle Calibration Mode Requested! Pass-through active <<<");
        status_tx.send(VisualStatus::esc_calibrating());

        loop {
            if let Ok(rc) = rc_rx.try_receive() {
                latest_rc = Some(rc);
                let throttle = rc.channels[RC_CH_THROTTLE].clamp(1000, 2000);
                esc_pwm.set_throttle_us(0, throttle);
                esc_pwm.set_throttle_us(1, throttle);

                // Pilot lowered stick to zero: exit calibration
                if throttle <= 1050 {
                    defmt::info!("ESC Throttle Minimum reached. Finalizing ESC calibration.");
                    esc_pwm.set_throttle_us(0, 1000);
                    esc_pwm.set_throttle_us(1, 1000);
                    Timer::after(Duration::from_millis(500)).await;
                    break;
                }
            }
            Timer::after(Duration::from_millis(10)).await;
        }
    }

    latest_rc
}

/// Initializes IMU hardware and runs gyro stationary bias calibration.
pub async fn calibrate_imu_and_gyro<
    I2C: embedded_hal_async::i2c::I2c,
    INT: embedded_hal_async::digital::Wait,
>(
    imu: &mut Lsm6ds3Driver<I2C, INT>,
    status_tx: &Sender<'static, ThreadModeRawMutex, VisualStatus, 2>,
) -> Result<(), crate::drivers::ImuError> {
    status_tx.send(VisualStatus::calibrating());

    if let Err(e) = imu.init().await {
        defmt::error!("IMU init failed: {:?}", e);
        status_tx.send(VisualStatus::status(ArmingStatus::Failsafe));
        return Err(e);
    }

    if let Err(e) = imu.calibrate_gyro(250).await {
        defmt::error!("Gyro calib failed: {:?}", e);
        status_tx.send(VisualStatus::status(ArmingStatus::Failsafe));
        return Err(e);
    }

    status_tx.send(VisualStatus::status(ArmingStatus::Disarmed));
    defmt::info!("Flight controller IMU calibrated and ready!");
    Ok(())
}

pub struct FlightLoopState {
    pub ahrs: VqfAHRS,
    pub gyro_lpf_x: Pt1Filter,
    pub gyro_lpf_y: Pt1Filter,
    pub gyro_lpf_z: Pt1Filter,
    pub pitch_pid: PidController,
    pub roll_pid: PidController,
    pub yaw_pid: PidController,
    pub mixer: AirframeMixer,
    pub safety: ArmingSafety,
    #[cfg(feature = "compass")]
    pub heading_hold: HeadingHoldController,
    #[cfg(feature = "compass")]
    pub latest_mag: Vector3,
    #[cfg(feature = "compass")]
    pub mag_health: SensorHealth,
    #[cfg(feature = "compass")]
    pub tilt_heading_rad: f32,
    #[cfg(feature = "baro")]
    pub alt_hold: AltHoldController,
    #[cfg(feature = "baro")]
    pub vkf: VerticalKalmanFilter,
    #[cfg(feature = "baro")]
    pub baro_health: SensorHealth,
    pub last_packet_time: Instant,
    pub latest_rc: RcChannels,
    pub status: ArmingStatus,
    pub last_rc_connected: bool,
    pub cached_euler: (f32, f32, f32),
    pub cached_pitch_terms: wingfc_core::pid::PidTerms,
    pub cached_roll_terms: wingfc_core::pid::PidTerms,
    pub last_imu_seq: u32,
    pub imu_overrun_count: u32,
    pub gps_groundspeed_mps: f32,
}

impl Default for FlightLoopState {
    fn default() -> Self {
        Self::new()
    }
}

impl FlightLoopState {
    pub fn new() -> Self {
        let mut gyro_lpf_x = Pt1Filter::new();
        let mut gyro_lpf_y = Pt1Filter::new();
        let mut gyro_lpf_z = Pt1Filter::new();
        gyro_lpf_x.init(GYRO_LPF_CUTOFF_HZ, DT);
        gyro_lpf_y.init(GYRO_LPF_CUTOFF_HZ, DT);
        gyro_lpf_z.init(GYRO_LPF_CUTOFF_HZ, DT);

        #[allow(unused_mut)]
        let mut ahrs = VqfAHRS::new(VQF_TAU_ACC, VQF_TAU_MAG, DT);
        #[cfg(feature = "compass")]
        ahrs.set_declination(crate::config::MAG_DECLINATION_RAD);

        Self {
            ahrs,
            gyro_lpf_x,
            gyro_lpf_y,
            gyro_lpf_z,
            pitch_pid: PidController::new(RATE_PID_CONFIG, DT),
            roll_pid: PidController::new(RATE_PID_CONFIG, DT),
            yaw_pid: PidController::new(RATE_PID_CONFIG, DT),
            mixer: {
                if AIRFRAME_TYPE == AirframeType::ElevonWithDiffThrust {
                    let mut m = AirframeMixer::with_diff_thrust(AIRFRAME_TYPE, DIFFERENTIAL_THRUST);
                    m.servos = SERVO_CONFIGS;
                    m
                } else {
                    let mut m = AirframeMixer::new(AIRFRAME_TYPE);
                    m.servos = SERVO_CONFIGS;
                    m
                }
            },
            safety: ArmingSafety::with_thresholds(ARM_SWITCH_THRESHOLD, THROTTLE_ZERO_THRESHOLD),
            #[cfg(feature = "compass")]
            heading_hold: HeadingHoldController::default(),
            #[cfg(feature = "compass")]
            latest_mag: Vector3::ZERO,
            #[cfg(feature = "compass")]
            mag_health: SensorHealth::Absent,
            #[cfg(feature = "compass")]
            tilt_heading_rad: 0.0,
            #[cfg(feature = "baro")]
            alt_hold: AltHoldController::default(),
            #[cfg(feature = "baro")]
            vkf: VerticalKalmanFilter::default(),
            #[cfg(feature = "baro")]
            baro_health: SensorHealth::Absent,
            last_packet_time: Instant::now(),
            latest_rc: RcChannels::default(),
            status: ArmingStatus::Disarmed,
            last_rc_connected: false,
            cached_euler: (0.0, 0.0, 0.0),
            cached_pitch_terms: wingfc_core::pid::PidTerms::default(),
            cached_roll_terms: wingfc_core::pid::PidTerms::default(),
            last_imu_seq: 0,
            imu_overrun_count: 0,
            gps_groundspeed_mps: 0.0,
        }
    }

    /// Execute one flight control cycle with dynamically measured delta time (dt)
    pub fn step(
        &mut self,
        rc_rx: &Receiver<'static, ThreadModeRawMutex, RcChannels, 2>,
        status_tx: &Sender<'static, ThreadModeRawMutex, VisualStatus, 2>,
        #[cfg(any(feature = "baro", feature = "compass"))] ext_sensor_rx: &mut WatchReceiver<
            'static,
            ThreadModeRawMutex,
            ExternalSensorSnapshot,
            2,
        >,
        imu_data: Result<wingfc_core::downsample::ImuDecimated, crate::drivers::ImuError>,
        dt: f32,
    ) -> AirframeOutputs {
        // 1. Drain latest RC packet
        let mut rc_packet_received = false;
        while let Ok(channels) = rc_rx.try_receive() {
            self.latest_rc = channels;
            self.last_packet_time = Instant::now();
            rc_packet_received = true;
        }

        // 2. Failsafe timeout check
        let signal_lost = Instant::now().duration_since(self.last_packet_time) > FAILSAFE_TIMEOUT
            || self.latest_rc.failsafe;

        // 3. Configurable Channel Mapping
        // Map RC pulses in [988.0, 2012.0] us to [-1.0, 1.0] using hoisted constant scale:
        // (ch - 988.0) * (2.0 / 1024.0) - 1.0 = (ch - 988.0) * (1.0 / 512.0) - 1.0
        const RC_MIN: f32 = 988.0;
        const RC_INV_HALF_RANGE: f32 = 1.0 / 512.0;
        let roll_stick =
            (self.latest_rc.channels[RC_CH_ROLL] as f32 - RC_MIN) * RC_INV_HALF_RANGE - 1.0;
        let pitch_stick =
            (self.latest_rc.channels[RC_CH_PITCH] as f32 - RC_MIN) * RC_INV_HALF_RANGE - 1.0;
        let throttle_raw = self.latest_rc.channels[RC_CH_THROTTLE].clamp(1000, 2000);
        let yaw_stick =
            (self.latest_rc.channels[RC_CH_YAW] as f32 - RC_MIN) * RC_INV_HALF_RANGE - 1.0;

        // Optional in-flight potentiometer PID tuning
        if IN_FLIGHT_TUNING_ENABLED {
            IN_FLIGHT_TUNING.update(
                &self.latest_rc,
                &mut self.pitch_pid,
                &mut self.roll_pid,
                &mut self.yaw_pid,
            );
        }

        // 4. Update Arming Safety State Machine with INAV-style Mode Matrix
        let evaluated_mode = MODE_MATRIX.evaluate(&self.latest_rc.channels);
        let new_status = self.safety.update_with_mode(
            self.latest_rc.channels[RC_CH_ARM],
            evaluated_mode,
            throttle_raw,
            signal_lost,
        );

        let mut visual = VisualStatus::status(new_status);
        visual.rc_connected = !signal_lost;
        visual.rc_packet_tick = rc_packet_received;
        #[cfg(feature = "gps")]
        {
            visual.gps_sats = 0;
            visual.gps_has_fix = false;
        }

        let rc_conn_changed = visual.rc_connected != self.last_rc_connected;
        if new_status != self.status || rc_conn_changed || rc_packet_received {
            if new_status != self.status {
                defmt::info!(
                    "Flight status transition: {:?} -> {:?}",
                    self.status,
                    new_status
                );
            }
            if rc_conn_changed {
                defmt::info!(
                    "RC link status: {}",
                    if visual.rc_connected {
                        "CONNECTED"
                    } else {
                        "LOST"
                    }
                );
            }
            self.status = new_status;
            self.last_rc_connected = visual.rc_connected;
            status_tx.send(visual);
        }

        // 5. Ingest External Sensor Snapshot (Baro / Compass)
        #[cfg(any(feature = "baro", feature = "compass"))]
        {
            if let Some(snapshot) = ext_sensor_rx.try_get() {
                #[cfg(feature = "compass")]
                {
                    self.mag_health = snapshot.mag_health;
                    if snapshot.mag_updated && snapshot.mag_health.is_healthy() {
                        self.latest_mag = snapshot.mag;
                        self.ahrs.update_mag(snapshot.mag, 0.05);
                    }
                }
                #[cfg(feature = "baro")]
                {
                    self.baro_health = snapshot.baro_health;
                    if snapshot.baro_updated && snapshot.baro_health.is_healthy() {
                        self.vkf.update_baro(snapshot.baro_alt_m);
                    }
                }
            }
        }

        // 6. Check IMU sequence and overruns
        if let Ok(ref frame) = imu_data {
            if self.last_imu_seq != 0 {
                let expected = self.last_imu_seq.wrapping_add(1);
                if frame.seq != expected {
                    let missed = frame.seq.wrapping_sub(expected);
                    self.imu_overrun_count = self.imu_overrun_count.saturating_add(missed);
                    defmt::warn!(
                        "IMU frame overrun: missed {} frames (seq: {}, expected: {})",
                        missed,
                        frame.seq,
                        expected
                    );
                }
            }
            self.last_imu_seq = frame.seq;
        }

        match self.status {
            ArmingStatus::Disarmed
            | ArmingStatus::RefusedThrottleRaised
            | ArmingStatus::RefusedSwitchArmedAtBoot => {
                // Ground pre-flight check: Servos follow stick inputs directly!
                self.pitch_pid.reset();
                self.roll_pid.reset();
                self.yaw_pid.reset();
                #[cfg(feature = "baro")]
                self.alt_hold.reset();
                #[cfg(feature = "compass")]
                self.heading_hold.reset();

                // If IMU read succeeded, update AHRS so attitude is initialized before takeoff
                if let Ok(ref frame) = imu_data {
                    let gyro = Vector3::new(
                        self.gyro_lpf_x.update(frame.gyro.x),
                        self.gyro_lpf_y.update(frame.gyro.y),
                        self.gyro_lpf_z.update(frame.gyro.z),
                    );
                    self.ahrs.update_6dof_centripetal_aided(
                        gyro,
                        frame.accel,
                        self.gps_groundspeed_mps,
                        dt,
                    );
                }

                return self.mixer.mix(pitch_stick, roll_stick, yaw_stick, 1000);
            }
            ArmingStatus::Failsafe => {
                // Motor cut + level flight surfaces neutralized
                return self.mixer.mix(0.0, 0.0, 0.0, 1000);
            }
            _ => {}
        }

        // Armed flight modes: IMU is required for rate/angle stabilization
        let imu_frame = match imu_data {
            Ok(data) => data,
            Err(_) => {
                // Failsafe fallback: retain manual stick authority so pilot does not lose aircraft control
                return self
                    .mixer
                    .mix(pitch_stick, roll_stick, yaw_stick, throttle_raw);
            }
        };

        // Filter high-frequency motor vibration from gyroscope
        let gyro = Vector3::new(
            self.gyro_lpf_x.update(imu_frame.gyro.x),
            self.gyro_lpf_y.update(imu_frame.gyro.y),
            self.gyro_lpf_z.update(imu_frame.gyro.z),
        );
        let accel = imu_frame.accel;

        // 7. Update Attitude Estimation (VQF AHRS at 208.33 Hz with GPS centripetal compensation)
        self.ahrs
            .update_6dof_centripetal_aided(gyro, accel, self.gps_groundspeed_mps, dt);

        if self.status.requires_attitude() {
            self.cached_euler = self.ahrs.euler_angles();
        }
        let (est_pitch, est_roll, est_yaw) = self.cached_euler;

        #[cfg(feature = "compass")]
        {
            self.tilt_heading_rad = est_yaw;
        }
        #[cfg(not(feature = "compass"))]
        {
            let _ = est_yaw;
        }

        // 8. Vertical Kalman Filter High-Rate Prediction Step
        #[cfg(feature = "baro")]
        {
            // Earth-frame vertical acceleration:
            // a_z_earth = -sin(pitch)*a_x + sin(roll)*cos(pitch)*a_y + cos(roll)*cos(pitch)*a_z - 9.80665
            let sp = libm::sinf(est_pitch);
            let cp = libm::cosf(est_pitch);
            let sr = libm::sinf(est_roll);
            let cr = libm::cosf(est_roll);
            let az_earth = -sp * accel.x + sr * cp * accel.y + cr * cp * accel.z - 9.80665;
            self.vkf.predict(az_earth, dt);
        }

        let pitch_mix;
        let roll_mix;
        let yaw_mix;
        let throttle_out;

        match self.status {
            ArmingStatus::ArmedAcro => {
                // ACRO / RATES MODE: Stick commands desired angular rate directly
                let target_roll_rate = roll_stick * MAX_ROLL_RATE;
                let target_pitch_rate = pitch_stick * MAX_PITCH_RATE;
                let target_yaw_rate = yaw_stick * MAX_YAW_RATE;

                let r_terms = self.roll_pid.update_ext(target_roll_rate, gyro.x, dt);
                let p_terms = self.pitch_pid.update_ext(target_pitch_rate, gyro.y, dt);
                self.cached_roll_terms = r_terms;
                self.cached_pitch_terms = p_terms;
                roll_mix = r_terms.total;
                pitch_mix = p_terms.total;
                yaw_mix = self.yaw_pid.update(target_yaw_rate, gyro.z, dt);
                throttle_out = throttle_raw;
            }
            ArmingStatus::ArmedAngle => {
                // ANGLE / STABILIZED MODE: Stick commands target pitch/roll angle
                let target_pitch = pitch_stick * MAX_PITCH_ANGLE;
                let target_roll = roll_stick * MAX_ROLL_ANGLE;

                let target_pitch_rate = (target_pitch - est_pitch) * ANGLE_P_GAIN;
                let target_roll_rate = (target_roll - est_roll) * ANGLE_P_GAIN;
                let target_yaw_rate = yaw_stick * MAX_YAW_RATE;

                let r_terms = self.roll_pid.update_ext(target_roll_rate, gyro.x, dt);
                let p_terms = self.pitch_pid.update_ext(target_pitch_rate, gyro.y, dt);
                self.cached_roll_terms = r_terms;
                self.cached_pitch_terms = p_terms;
                roll_mix = r_terms.total;
                pitch_mix = p_terms.total;
                yaw_mix = self.yaw_pid.update(target_yaw_rate, gyro.z, dt);
                throttle_out = throttle_raw;
            }
            #[cfg(feature = "compass")]
            ArmingStatus::ArmedHeadingHold => {
                // HEADING HOLD MODE: Cascaded heading controller onto Angle roll
                // Graceful fallback: If compass is degraded/lost mid-flight, fallback to standard Angle mode
                let target_roll = if self.mag_health.is_healthy() {
                    self.heading_hold
                        .update(self.tilt_heading_rad, roll_stick, yaw_stick)
                } else {
                    roll_stick * MAX_ROLL_ANGLE
                };
                let target_pitch = pitch_stick * MAX_PITCH_ANGLE;

                let target_pitch_rate = (target_pitch - est_pitch) * ANGLE_P_GAIN;
                let target_roll_rate = (target_roll - est_roll) * ANGLE_P_GAIN;
                let target_yaw_rate = if self.mag_health.is_healthy() {
                    0.0
                } else {
                    yaw_stick * MAX_YAW_RATE
                };

                let r_terms = self.roll_pid.update_ext(target_roll_rate, gyro.x, dt);
                let p_terms = self.pitch_pid.update_ext(target_pitch_rate, gyro.y, dt);
                self.cached_roll_terms = r_terms;
                self.cached_pitch_terms = p_terms;
                roll_mix = r_terms.total;
                pitch_mix = p_terms.total;
                yaw_mix = self.yaw_pid.update(target_yaw_rate, gyro.z, dt);
                throttle_out = throttle_raw;
            }
            #[cfg(feature = "baro")]
            ArmingStatus::ArmedAltHold => {
                // ALTITUDE HOLD MODE: Cascaded baro/KF altitude controller onto Angle pitch
                // Graceful fallback: If barometer is degraded/lost mid-flight, fallback to standard Angle mode
                let (target_pitch, alt_thr) = if self.baro_health.is_healthy() {
                    self.alt_hold.update(
                        self.vkf.altitude_m,
                        self.vkf.velocity_ms,
                        pitch_stick,
                        throttle_raw,
                    )
                } else {
                    (pitch_stick * MAX_PITCH_ANGLE, throttle_raw)
                };
                let target_roll = roll_stick * MAX_ROLL_ANGLE;

                let target_pitch_rate = (target_pitch - est_pitch) * ANGLE_P_GAIN;
                let target_roll_rate = (target_roll - est_roll) * ANGLE_P_GAIN;
                let target_yaw_rate = yaw_stick * MAX_YAW_RATE;

                let r_terms = self.roll_pid.update_ext(target_roll_rate, gyro.x, dt);
                let p_terms = self.pitch_pid.update_ext(target_pitch_rate, gyro.y, dt);
                self.cached_roll_terms = r_terms;
                self.cached_pitch_terms = p_terms;
                roll_mix = r_terms.total;
                pitch_mix = p_terms.total;
                yaw_mix = self.yaw_pid.update(target_yaw_rate, gyro.z, dt);
                throttle_out = alt_thr;
            }
            #[cfg(feature = "gps")]
            ArmingStatus::ArmedRth => {
                // RTH MODE: Autonomous navigation hold (Angle mode level flight fallback)
                let target_pitch = 0.05; // Slight gentle climb
                let target_roll = 0.0; // Level wings

                let target_pitch_rate = (target_pitch - est_pitch) * ANGLE_P_GAIN;
                let target_roll_rate = (target_roll - est_roll) * ANGLE_P_GAIN;
                let target_yaw_rate = 0.0;

                let r_terms = self.roll_pid.update_ext(target_roll_rate, gyro.x, dt);
                let p_terms = self.pitch_pid.update_ext(target_pitch_rate, gyro.y, dt);
                self.cached_roll_terms = r_terms;
                self.cached_pitch_terms = p_terms;
                roll_mix = r_terms.total;
                pitch_mix = p_terms.total;
                yaw_mix = self.yaw_pid.update(target_yaw_rate, gyro.z, dt);
                throttle_out = 1500; // Safe cruise throttle
            }

            _ => {
                self.cached_roll_terms = wingfc_core::pid::PidTerms::default();
                self.cached_pitch_terms = wingfc_core::pid::PidTerms::default();
                pitch_mix = 0.0;
                roll_mix = 0.0;
                yaw_mix = 0.0;
                throttle_out = 1000;
            }
        }

        self.mixer.mix(pitch_mix, roll_mix, yaw_mix, throttle_out)
    }

    /// Returns the latest vehicle state snapshot for telemetry broadcasting
    #[cfg(feature = "crsf")]
    pub fn telemetry_snapshot(&mut self) -> crate::tasks::FlightTelemetrySnapshot {
        // If the active mode did not require attitude estimation, compute angles now for telemetry
        if !self.status.requires_attitude() {
            self.cached_euler = self.ahrs.euler_angles();
        }
        let (pitch, roll, yaw) = self.cached_euler;
        crate::tasks::FlightTelemetrySnapshot {
            roll_rad: roll,
            pitch_rad: pitch,
            yaw_rad: yaw,
            status: self.status,
            #[cfg(feature = "gps")]
            gps_lat: 0.0, // TODO: Connect to GPS driver state once GPS task is merged
            #[cfg(feature = "gps")]
            gps_lon: 0.0, // TODO: Connect to GPS driver state once GPS task is merged
            #[cfg(feature = "gps")]
            gps_speed_mps: 0.0, // TODO: Connect to GPS driver state once GPS task is merged
            #[cfg(feature = "gps")]
            gps_heading_deg: 0.0, // TODO: Connect to GPS driver state once GPS task is merged
            #[cfg(feature = "gps")]
            gps_alt_m: 0.0, // TODO: Connect to GPS driver state once GPS task is merged
            #[cfg(feature = "gps")]
            gps_sats: 0, // TODO: Connect to GPS driver state once GPS task is merged
            #[cfg(feature = "gps")]
            gps_has_fix: false, // TODO: Connect to GPS driver state once GPS task is merged
            #[cfg(feature = "baro")]
            baro_alt_m: self.vkf.altitude_m,
            #[cfg(feature = "baro")]
            vario_cms: (self.vkf.velocity_ms * 100.0) as i16,
        }
    }
}

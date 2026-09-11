use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::watch::Sender;
use embassy_time::Instant;
#[cfg(not(feature = "dshot"))]
use embassy_time::{Duration, Timer};
use wingfc_core::ahrs::MadgwickAHRS;
use wingfc_core::filter::Pt1Filter;
use wingfc_core::math::Vector3;

use wingfc_core::mixer::{AirframeMixer, AirframeOutputs, AirframeType};
use wingfc_core::pid::PidController;
use wingfc_core::protocol::RcChannels;
use wingfc_core::safety::{ArmingSafety, ArmingStatus};

use crate::config::{
    AHRS_BETA, AIRFRAME_TYPE, ANGLE_P_GAIN, ARM_SWITCH_THRESHOLD, DIFFERENTIAL_THRUST, DT,
    FAILSAFE_TIMEOUT, GYRO_LPF_CUTOFF_HZ, IN_FLIGHT_TUNING, IN_FLIGHT_TUNING_ENABLED,
    MAX_PITCH_ANGLE, MAX_PITCH_RATE, MAX_ROLL_ANGLE, MAX_ROLL_RATE, MAX_YAW_RATE, MODE_MATRIX,
    RATE_PID_CONFIG, RC_CH_ARM, RC_CH_PITCH, RC_CH_ROLL, RC_CH_THROTTLE, RC_CH_YAW, SERVO_CONFIGS,
    THROTTLE_ZERO_THRESHOLD,
};
#[cfg(not(feature = "dshot"))]
use crate::drivers::EscPwmDriver;
use crate::drivers::Lsm6ds3Driver;
use crate::tasks::telemetry::VisualStatus;
#[cfg(feature = "baro")]
use wingfc_core::mode::AltHoldController;
#[cfg(feature = "compass")]
use wingfc_core::mode::HeadingHoldController;

pub struct FlightLoopState<'d> {
    pub imu: Lsm6ds3Driver<'d>,
    pub ahrs: MadgwickAHRS,
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
    #[cfg(feature = "baro")]
    pub alt_hold: AltHoldController,
    pub last_packet_time: Instant,
    pub latest_rc: RcChannels,
    pub status: ArmingStatus,
    pub cached_euler: (f32, f32, f32),
}

impl<'d> FlightLoopState<'d> {
    pub fn new(imu: Lsm6ds3Driver<'d>) -> Self {
        let mut gyro_lpf_x = Pt1Filter::new();
        let mut gyro_lpf_y = Pt1Filter::new();
        let mut gyro_lpf_z = Pt1Filter::new();
        gyro_lpf_x.init(GYRO_LPF_CUTOFF_HZ, DT);
        gyro_lpf_y.init(GYRO_LPF_CUTOFF_HZ, DT);
        gyro_lpf_z.init(GYRO_LPF_CUTOFF_HZ, DT);

        Self {
            imu,
            ahrs: MadgwickAHRS::new(AHRS_BETA),
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
            #[cfg(feature = "baro")]
            alt_hold: AltHoldController::default(),
            last_packet_time: Instant::now(),
            latest_rc: RcChannels::default(),
            status: ArmingStatus::Disarmed,
            cached_euler: (0.0, 0.0, 0.0),
        }
    }

    /// Initializes hardware, checks for ESC Throttle Calibration, and runs Gyro Calibration (Analog PWM)
    #[cfg(not(feature = "dshot"))]
    pub async fn init(
        &mut self,
        rc_rx: &Receiver<'static, ThreadModeRawMutex, RcChannels, 2>,
        status_tx: &Sender<'static, ThreadModeRawMutex, VisualStatus, 2>,
        esc_pwm: &mut EscPwmDriver<'_>,
    ) {
        // Step 1: Wait briefly (~500ms) to check if pilot powered on with Throttle HIGH for ESC Calibration
        let start = Instant::now();
        let mut in_esc_calib = false;

        while start.elapsed() < Duration::from_millis(600) {
            if let Ok(rc) = rc_rx.try_receive() {
                self.latest_rc = rc;
                self.last_packet_time = Instant::now();
                if rc.channels[RC_CH_THROTTLE] > 1900 {
                    in_esc_calib = true;
                    break;
                }
            }
            Timer::after(Duration::from_millis(20)).await;
        }

        // ESC Throttle Range Calibration Mode (Universal Rule #3)
        if in_esc_calib {
            defmt::warn!(">>> ESC Throttle Calibration Mode Requested! Pass-through active <<<");
            status_tx.send(VisualStatus::esc_calibrating());

            loop {
                if let Ok(rc) = rc_rx.try_receive() {
                    self.latest_rc = rc;
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

        // Step 2: Gyro Stationary Calibration
        status_tx.send(VisualStatus::calibrating());

        if let Err(e) = self.imu.init().await {
            defmt::error!("IMU init failed: {:?}", e);
            status_tx.send(VisualStatus::status(ArmingStatus::Failsafe));
            return;
        }

        if let Err(e) = self.imu.calibrate_gyro(250).await {
            defmt::error!("Gyro calib failed: {:?}", e);
            status_tx.send(VisualStatus::status(ArmingStatus::Failsafe));
            return;
        }

        self.status = ArmingStatus::Disarmed;
        status_tx.send(VisualStatus::status(ArmingStatus::Disarmed));
        defmt::info!("Flight controller initialized and ready!");
    }

    /// Initializes hardware and runs Gyro Calibration (Digital DShot ESC - no analog calibration needed)
    #[cfg(feature = "dshot")]
    pub async fn init(&mut self, status_tx: &Sender<'static, ThreadModeRawMutex, VisualStatus, 2>) {
        // Gyro Stationary Calibration
        status_tx.send(VisualStatus::calibrating());

        if let Err(e) = self.imu.init().await {
            defmt::error!("IMU init failed: {:?}", e);
            status_tx.send(VisualStatus::status(ArmingStatus::Failsafe));
            return;
        }

        if let Err(e) = self.imu.calibrate_gyro(250).await {
            defmt::error!("Gyro calib failed: {:?}", e);
            status_tx.send(VisualStatus::status(ArmingStatus::Failsafe));
            return;
        }

        self.status = ArmingStatus::Disarmed;
        status_tx.send(VisualStatus::status(ArmingStatus::Disarmed));
        defmt::info!("Flight controller initialized and ready!");
    }

    /// Execute one flight control cycle with dynamically measured delta time (dt)
    pub async fn step(
        &mut self,
        rc_rx: &Receiver<'static, ThreadModeRawMutex, RcChannels, 2>,
        status_tx: &Sender<'static, ThreadModeRawMutex, VisualStatus, 2>,
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
            IN_FLIGHT_TUNING.update(&self.latest_rc, &mut self.pitch_pid, &mut self.roll_pid);
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

        if new_status != self.status || rc_packet_received {
            self.status = new_status;
            status_tx.send(visual);
        }

        // 5. Read IMU
        let imu_data = self.imu.read().await;

        match self.status {
            ArmingStatus::Disarmed
            | ArmingStatus::RefusedThrottleRaised
            | ArmingStatus::RefusedSwitchArmedAtBoot => {
                // Ground pre-flight check: Servos follow stick inputs directly!
                self.pitch_pid.reset();
                self.roll_pid.reset();
                self.yaw_pid.reset();

                // If IMU read succeeded, update AHRS so attitude is initialized before takeoff
                if let Ok((raw_gyro, accel)) = imu_data {
                    let gyro = Vector3::new(
                        self.gyro_lpf_x.update(raw_gyro.x),
                        self.gyro_lpf_y.update(raw_gyro.y),
                        self.gyro_lpf_z.update(raw_gyro.z),
                    );
                    self.ahrs.update_6dof(gyro, accel, dt);
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
        let (raw_gyro, accel) = match imu_data {
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
            self.gyro_lpf_x.update(raw_gyro.x),
            self.gyro_lpf_y.update(raw_gyro.y),
            self.gyro_lpf_z.update(raw_gyro.z),
        );

        // 6. Update Attitude Estimation (Madgwick AHRS)
        self.ahrs.update_6dof(gyro, accel, dt);
        if self.status.requires_attitude() {
            self.cached_euler = self.ahrs.get_euler_angles();
        }
        let (est_pitch, est_roll, _est_yaw) = self.cached_euler;

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

                roll_mix = self.roll_pid.update(target_roll_rate, gyro.x, dt);
                pitch_mix = self.pitch_pid.update(target_pitch_rate, gyro.y, dt);
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

                roll_mix = self.roll_pid.update(target_roll_rate, gyro.x, dt);
                pitch_mix = self.pitch_pid.update(target_pitch_rate, gyro.y, dt);
                yaw_mix = self.yaw_pid.update(target_yaw_rate, gyro.z, dt);
                throttle_out = throttle_raw;
            }
            #[cfg(feature = "compass")]
            ArmingStatus::ArmedHeadingHold => {
                // HEADING HOLD MODE: Cascaded heading controller onto Angle roll
                let target_roll = self.heading_hold.update(_est_yaw, roll_stick, yaw_stick);
                let target_pitch = pitch_stick * MAX_PITCH_ANGLE;

                let target_pitch_rate = (target_pitch - est_pitch) * ANGLE_P_GAIN;
                let target_roll_rate = (target_roll - est_roll) * ANGLE_P_GAIN;
                let target_yaw_rate = 0.0; // Heading hold locks yaw via roll banking

                roll_mix = self.roll_pid.update(target_roll_rate, gyro.x, dt);
                pitch_mix = self.pitch_pid.update(target_pitch_rate, gyro.y, dt);
                yaw_mix = self.yaw_pid.update(target_yaw_rate, gyro.z, dt);
                throttle_out = throttle_raw;
            }
            #[cfg(feature = "baro")]
            ArmingStatus::ArmedAltHold => {
                // ALTITUDE HOLD MODE: Cascaded baro/KF altitude controller onto Angle pitch
                // (Fallback to current est_pitch if baro/KF not yet integrated into loop state)
                let (target_pitch, alt_thr) =
                    self.alt_hold.update(0.0, 0.0, pitch_stick, throttle_raw);
                let target_roll = roll_stick * MAX_ROLL_ANGLE;

                let target_pitch_rate = (target_pitch - est_pitch) * ANGLE_P_GAIN;
                let target_roll_rate = (target_roll - est_roll) * ANGLE_P_GAIN;
                let target_yaw_rate = yaw_stick * MAX_YAW_RATE;

                roll_mix = self.roll_pid.update(target_roll_rate, gyro.x, dt);
                pitch_mix = self.pitch_pid.update(target_pitch_rate, gyro.y, dt);
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

                roll_mix = self.roll_pid.update(target_roll_rate, gyro.x, dt);
                pitch_mix = self.pitch_pid.update(target_pitch_rate, gyro.y, dt);
                yaw_mix = self.yaw_pid.update(target_yaw_rate, gyro.z, dt);
                throttle_out = 1500; // Safe cruise throttle
            }

            _ => {
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
            self.cached_euler = self.ahrs.get_euler_angles();
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
            baro_alt_m: 0.0, // TODO: Connect to Baro driver state once Baro task is merged
            #[cfg(feature = "baro")]
            vario_cms: 0, // TODO: Connect to Baro/KF vario state once Baro task is merged
        }
    }
}

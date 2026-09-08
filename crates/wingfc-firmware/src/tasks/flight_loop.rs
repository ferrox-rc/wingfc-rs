use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::watch::Sender;
#[cfg(not(feature = "dshot"))]
use embassy_time::Timer;
use embassy_time::{Duration, Instant};
use wingfc_core::ahrs::MadgwickAHRS;
use wingfc_core::filter::Pt1Filter;
use wingfc_core::math::{map_range, Vector3};
use wingfc_core::mixer::{AirframeMixer, AirframeOutputs};
use wingfc_core::pid::PidController;
use wingfc_core::protocol::RcChannels;
use wingfc_core::safety::{ArmingSafety, ArmingStatus};

use crate::config::{
    AHRS_BETA, AIRFRAME_TYPE, ANGLE_P_GAIN, ARM_SWITCH_THRESHOLD, DT, FAILSAFE_TIMEOUT,
    GYRO_LPF_CUTOFF_HZ, IN_FLIGHT_TUNING, IN_FLIGHT_TUNING_ENABLED, MAX_PITCH_ANGLE,
    MAX_PITCH_RATE, MAX_ROLL_ANGLE, MAX_ROLL_RATE, MAX_YAW_RATE, RATE_PID_CONFIG, SERVO_CONFIGS,
    THROTTLE_ZERO_THRESHOLD,
};
#[cfg(not(feature = "dshot"))]
use crate::drivers::EscPwmDriver;
use crate::drivers::Lsm6ds3Driver;
use crate::tasks::telemetry::VisualStatus;

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
    pub last_packet_time: Instant,
    pub latest_rc: RcChannels,
    pub status: ArmingStatus,
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
                let mut m = AirframeMixer::new(AIRFRAME_TYPE);
                m.servos = SERVO_CONFIGS;
                m
            },
            safety: ArmingSafety::with_thresholds(ARM_SWITCH_THRESHOLD, THROTTLE_ZERO_THRESHOLD),
            last_packet_time: Instant::now(),
            latest_rc: RcChannels::default(),
            status: ArmingStatus::Disarmed,
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
                if rc.channels[2] > 1900 {
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
                    let throttle = rc.channels[2].clamp(1000, 2000);
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

    /// Execute one 500Hz flight cycle
    pub async fn step(
        &mut self,
        rc_rx: &Receiver<'static, ThreadModeRawMutex, RcChannels, 2>,
        status_tx: &Sender<'static, ThreadModeRawMutex, VisualStatus, 2>,
    ) -> AirframeOutputs {
        // 1. Drain latest RC packet
        let mut rc_packet_received = false;
        while let Ok(channels) = rc_rx.try_receive() {
            self.latest_rc = channels;
            self.last_packet_time = Instant::now();
            rc_packet_received = true;
        }

        // 2. Failsafe timeout check
        let signal_lost = Instant::now().duration_since(self.last_packet_time) > FAILSAFE_TIMEOUT;

        // 3. Channel Mapping:
        // CH1: Roll (0), CH2: Pitch (1), CH3: Throttle (2), CH4: Yaw (3), CH5: Arm (4), CH6: Mode (5)
        let roll_stick = map_range(self.latest_rc.channels[0] as f32, 988.0, 2012.0, -1.0, 1.0);
        let pitch_stick = map_range(self.latest_rc.channels[1] as f32, 988.0, 2012.0, -1.0, 1.0);
        let throttle_raw = self.latest_rc.channels[2].clamp(1000, 2000);
        let yaw_stick = map_range(self.latest_rc.channels[3] as f32, 988.0, 2012.0, -1.0, 1.0);

        // Optional in-flight potentiometer PID tuning
        if IN_FLIGHT_TUNING_ENABLED {
            IN_FLIGHT_TUNING.update(&self.latest_rc, &mut self.pitch_pid, &mut self.roll_pid);
        }

        // 4. Update Arming Safety State Machine
        let new_status = self.safety.update(
            self.latest_rc.channels[4],
            self.latest_rc.channels[5],
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
        let (raw_gyro, accel) = match self.imu.read().await {
            Ok(data) => data,
            Err(_) => {
                return self.mixer.mix(0.0, 0.0, 0.0, 1000);
            }
        };

        // Filter high-frequency motor vibration from gyroscope
        let gyro = Vector3::new(
            self.gyro_lpf_x.update(raw_gyro.x),
            self.gyro_lpf_y.update(raw_gyro.y),
            self.gyro_lpf_z.update(raw_gyro.z),
        );

        // 6. Update Attitude Estimation (Madgwick AHRS)
        self.ahrs.update_6dof(gyro, accel, DT);
        let (est_pitch, est_roll, _est_yaw) = self.ahrs.get_euler_angles();

        let pitch_mix;
        let roll_mix;
        let yaw_mix;
        let throttle_out;

        match self.status {
            ArmingStatus::Disarmed
            | ArmingStatus::RefusedThrottleRaised
            | ArmingStatus::RefusedSwitchArmedAtBoot => {
                // Servos follow stick inputs directly for ground pre-flight check
                pitch_mix = pitch_stick;
                roll_mix = roll_stick;
                yaw_mix = yaw_stick;
                throttle_out = 1000; // Locked zero throttle

                self.pitch_pid.reset();
                self.roll_pid.reset();
                self.yaw_pid.reset();
            }
            ArmingStatus::ArmedAcro => {
                // ACRO / RATES MODE: Stick commands desired angular rate directly
                let target_roll_rate = roll_stick * MAX_ROLL_RATE;
                let target_pitch_rate = pitch_stick * MAX_PITCH_RATE;
                let target_yaw_rate = yaw_stick * MAX_YAW_RATE;

                roll_mix = self.roll_pid.update(target_roll_rate, gyro.x, DT);
                pitch_mix = self.pitch_pid.update(target_pitch_rate, gyro.y, DT);
                yaw_mix = self.yaw_pid.update(target_yaw_rate, gyro.z, DT);
                throttle_out = throttle_raw;
            }
            ArmingStatus::ArmedAngle => {
                // ANGLE / STABILIZED MODE: Stick commands target pitch/roll angle
                let target_pitch = pitch_stick * MAX_PITCH_ANGLE;
                let target_roll = roll_stick * MAX_ROLL_ANGLE;

                // Outer P-loop calculates target rates from angle errors
                let target_pitch_rate = (target_pitch - est_pitch) * ANGLE_P_GAIN;
                let target_roll_rate = (target_roll - est_roll) * ANGLE_P_GAIN;
                let target_yaw_rate = yaw_stick * MAX_YAW_RATE;

                roll_mix = self.roll_pid.update(target_roll_rate, gyro.x, DT);
                pitch_mix = self.pitch_pid.update(target_pitch_rate, gyro.y, DT);
                yaw_mix = self.yaw_pid.update(target_yaw_rate, gyro.z, DT);
                throttle_out = throttle_raw;
            }
            ArmingStatus::Failsafe => {
                // Motor cut + level flight surfaces neutralized
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
    pub fn telemetry_snapshot(&self) -> crate::tasks::FlightTelemetrySnapshot {
        let (pitch, roll, yaw) = self.ahrs.get_euler_angles();
        crate::tasks::FlightTelemetrySnapshot {
            roll_rad: roll,
            pitch_rad: pitch,
            yaw_rad: yaw,
            status: self.status,
            #[cfg(feature = "gps")]
            gps_lat: 0.0,
            #[cfg(feature = "gps")]
            gps_lon: 0.0,
            #[cfg(feature = "gps")]
            gps_speed_mps: 0.0,
            #[cfg(feature = "gps")]
            gps_heading_deg: 0.0,
            #[cfg(feature = "gps")]
            gps_alt_m: 0.0,
            #[cfg(feature = "gps")]
            gps_sats: 0,
            #[cfg(feature = "gps")]
            gps_has_fix: false,
            #[cfg(feature = "baro")]
            baro_alt_m: 0.0,
            #[cfg(feature = "baro")]
            vario_cms: 0,
        }
    }
}

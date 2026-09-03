use embassy_nrf::twim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Instant};
use wingfc_core::ahrs::MadgwickAHRS;
use wingfc_core::filter::Pt1Filter;
use wingfc_core::math::{map_range, Vector3, DEG_TO_RAD};
use wingfc_core::mixer::{AirframeMixer, AirframeOutputs, AirframeType};
use wingfc_core::pid::{PidConfig, PidController};
use wingfc_core::protocol::RcChannels;
use wingfc_core::safety::{ArmingSafety, ArmingStatus};

use crate::drivers::Lsm6ds3Driver;
use crate::tasks::telemetry::VisualStatus;

const DT: f32 = 0.002; // 500 Hz control loop (2ms)
const FAILSAFE_TIMEOUT: Duration = Duration::from_millis(500);

const MAX_ROLL_RATE: f32 = 600.0 * DEG_TO_RAD;   // 600 deg/s in rad/s
const MAX_PITCH_RATE: f32 = 200.0 * DEG_TO_RAD;  // 200 deg/s in rad/s
const MAX_YAW_RATE: f32 = 100.0 * DEG_TO_RAD;    // 100 deg/s in rad/s

const MAX_PITCH_ANGLE: f32 = 45.0 * DEG_TO_RAD;  // 45 deg max bank in Angle mode
const MAX_ROLL_ANGLE: f32 = 60.0 * DEG_TO_RAD;   // 60 deg max bank in Angle mode

const GYRO_LPF_CUTOFF_HZ: f32 = 90.0; // 90 Hz low-pass filter to reject motor/prop vibration

pub struct FlightLoopState<'d, T: twim::Instance> {
    pub imu: Lsm6ds3Driver<'d, T>,
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

impl<'d, T: twim::Instance> FlightLoopState<'d, T> {
    pub fn new(imu: Lsm6ds3Driver<'d, T>) -> Self {
        let rate_pid_cfg = PidConfig {
            kp: 0.15,
            ki: 0.05,
            kd: 0.008,
            max_integral: 0.4,
            dterm_lpf_hz: 40.0,
        };

        let mut gyro_lpf_x = Pt1Filter::new();
        let mut gyro_lpf_y = Pt1Filter::new();
        let mut gyro_lpf_z = Pt1Filter::new();
        gyro_lpf_x.init(GYRO_LPF_CUTOFF_HZ, DT);
        gyro_lpf_y.init(GYRO_LPF_CUTOFF_HZ, DT);
        gyro_lpf_z.init(GYRO_LPF_CUTOFF_HZ, DT);

        Self {
            imu,
            ahrs: MadgwickAHRS::new(0.04),
            gyro_lpf_x,
            gyro_lpf_y,
            gyro_lpf_z,
            pitch_pid: PidController::new(rate_pid_cfg, DT),
            roll_pid: PidController::new(rate_pid_cfg, DT),
            yaw_pid: PidController::new(rate_pid_cfg, DT),
            mixer: AirframeMixer::new(AirframeType::Elevon),
            safety: ArmingSafety::new(),
            last_packet_time: Instant::now(),
            latest_rc: RcChannels::default(),
            status: ArmingStatus::Disarmed,
        }
    }

    pub async fn init(&mut self, status_tx: &Sender<'static, CriticalSectionRawMutex, VisualStatus, 2>) {
        status_tx.send(VisualStatus::Calibration);

        if let Err(e) = self.imu.init().await {
            defmt::error!("IMU init failed: {:?}", e);
            status_tx.send(VisualStatus::Status(ArmingStatus::Failsafe));
            return;
        }

        if let Err(e) = self.imu.calibrate_gyro(250).await {
            defmt::error!("Gyro calib failed: {:?}", e);
            status_tx.send(VisualStatus::Status(ArmingStatus::Failsafe));
            return;
        }

        self.status = ArmingStatus::Disarmed;
        status_tx.send(VisualStatus::Status(ArmingStatus::Disarmed));
        defmt::info!("Flight controller initialized and ready!");
    }

    /// Execute one 500Hz flight cycle
    pub async fn step(
        &mut self,
        rc_rx: &Receiver<'static, CriticalSectionRawMutex, RcChannels, 2>,
        status_tx: &Sender<'static, CriticalSectionRawMutex, VisualStatus, 2>,
    ) -> AirframeOutputs {
        // 1. Drain latest RC packet
        while let Ok(channels) = rc_rx.try_receive() {
            self.latest_rc = channels;
            self.last_packet_time = Instant::now();
        }

        // 2. Failsafe timeout check
        let signal_lost = Instant::now().duration_since(self.last_packet_time) > FAILSAFE_TIMEOUT;

        // 3. Channel Mapping:
        // CH1: Roll (0), CH2: Pitch (1), CH3: Throttle (2), CH4: Yaw (3), CH5: Arm (4), CH6: Mode (5)
        let roll_stick = map_range(self.latest_rc.channels[0] as f32, 988.0, 2012.0, -1.0, 1.0);
        let pitch_stick = map_range(self.latest_rc.channels[1] as f32, 988.0, 2012.0, -1.0, 1.0);
        let throttle_raw = self.latest_rc.channels[2].clamp(1000, 2000);
        let yaw_stick = map_range(self.latest_rc.channels[3] as f32, 988.0, 2012.0, -1.0, 1.0);

        // 4. Update Arming Safety State Machine
        let new_status = self.safety.update(
            self.latest_rc.channels[4],
            self.latest_rc.channels[5],
            throttle_raw,
            signal_lost,
        );

        if new_status != self.status {
            self.status = new_status;
            status_tx.send(VisualStatus::Status(new_status));
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
                let target_pitch_rate = (target_pitch - est_pitch) * 4.0;
                let target_roll_rate = (target_roll - est_roll) * 4.0;
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
}

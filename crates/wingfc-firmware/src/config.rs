// User aircraft configuration parameters for `wingfc-firmware`.
// Generated automatically from `wingfc.yaml` by the build system.
// Users configure their aircraft via `wingfc.yaml` without editing Rust source code.

#[allow(unused_imports)]
use embassy_time::Duration;
#[allow(unused_imports)]
use wingfc_core::math::{BoardOrientation, Vector3, DEG_TO_RAD};
#[allow(unused_imports)]
use wingfc_core::mixer::{AirframeType, ServoConfig};
#[allow(unused_imports)]
use wingfc_core::mode::{FlightMode, ModeMatrix, ModeRange};
#[allow(unused_imports)]
use wingfc_core::pid::PidConfig;
#[allow(unused_imports)]
use wingfc_core::tuning::{InFlightTuning, TuneBank, TuneParameter, TuneSlot};

/// Board mounting orientation
pub const BOARD_ORIENTATION: BoardOrientation = BoardOrientation::Default;

/// Configured airframe mixer topology
pub const AIRFRAME_TYPE: AirframeType = AirframeType::Elevon;

/// Selectable flight loop timing profiles
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoopRateProfile {
    Standard416Hz,
    High833Hz,
    MultiRate208Hz,
}

/// Configured flight loop rate profile
pub const LOOP_RATE_PROFILE: LoopRateProfile = LoopRateProfile::MultiRate208Hz;

/// Nominal flight loop frequency in Hz
pub const NOMINAL_LOOP_HZ: u32 = 208;

/// Nominal flight loop time-step in seconds
pub const NOMINAL_DT: f32 = 0.004800;

/// Retained alias for nominal DT (used for initial static filter cutoffs)
pub const DT: f32 = NOMINAL_DT;

/// LSM6DS3 register value for Accel (CTRL1_XL): configured ODR + +/-16g range
pub const IMU_ODR_XL_REG_VAL: u8 = 0x84;

/// LSM6DS3 register value for Gyro (CTRL2_G): configured ODR + +/-2000 dps range
pub const IMU_ODR_G_REG_VAL: u8 = 0x8c;

/// Number of raw IMU samples boxcar-averaged per flight loop iteration (1 = no decimation)
pub const IMU_DECIMATION: u8 = 8;

/// Duration without valid RC packets before entering failsafe mode
pub const FAILSAFE_TIMEOUT: Duration = Duration::from_millis(500);

/// Configured RC channel indexes (0-indexed)
pub const RC_CH_ROLL: usize = 0;
pub const RC_CH_PITCH: usize = 1;
pub const RC_CH_THROTTLE: usize = 2;
pub const RC_CH_YAW: usize = 3;
pub const RC_CH_ARM: usize = 4;
pub const RC_CH_MODE: usize = 5;

/// INAV-style multi-switch Flight Mode Activation Matrix
pub const MODE_MATRIX: ModeMatrix<2> = ModeMatrix::new([
    ModeRange::new(FlightMode::Angle, 5, 1300, 2100),
    ModeRange::new(FlightMode::Acro, 5, 900, 1300),
]);

/// Microsecond threshold for Arm Switch (CH5 / AUX1) above which aircraft arms
pub const ARM_SWITCH_THRESHOLD: u16 = 1800;

/// Maximum allowable throttle pulse in microseconds to permit arming
pub const THROTTLE_ZERO_THRESHOLD: u16 = 1050;

/// Configured PWM frame frequency for servo outputs in Hz (50Hz - 400Hz).
/// Standard analog servos require 50 Hz; digital servos can accept up to 400 Hz.
pub const SERVO_PWM_HZ: u32 = 50;

/// Per-servo output endpoint, subtrim, and inversion calibration profiles
pub const SERVO_CONFIGS: [ServoConfig; 8] = [
    ServoConfig {
        min_us: 1000,
        max_us: 2000,
        subtrim_us: 0,
        reversed: false,
    },
    ServoConfig {
        min_us: 1000,
        max_us: 2000,
        subtrim_us: 0,
        reversed: false,
    },
    ServoConfig {
        min_us: 1000,
        max_us: 2000,
        subtrim_us: 0,
        reversed: false,
    },
    ServoConfig {
        min_us: 1000,
        max_us: 2000,
        subtrim_us: 0,
        reversed: false,
    },
    ServoConfig {
        min_us: 1000,
        max_us: 2000,
        subtrim_us: 0,
        reversed: false,
    },
    ServoConfig {
        min_us: 1000,
        max_us: 2000,
        subtrim_us: 0,
        reversed: false,
    },
    ServoConfig {
        min_us: 1000,
        max_us: 2000,
        subtrim_us: 0,
        reversed: false,
    },
    ServoConfig {
        min_us: 1000,
        max_us: 2000,
        subtrim_us: 0,
        reversed: false,
    },
];

/// Configured PWM frame frequency for analog ESC output in Hz (50Hz - 400Hz).
/// Legacy airplane ESCs require 50 Hz; high-speed multirotor ESCs support 400 Hz.
pub const ESC_PWM_HZ: u32 = 400;

/// Differential thrust authority for twin-motor airframes (0.0 .. 1.0 fraction of throttle range)
pub const DIFFERENTIAL_THRUST: f32 = 0.1500;

/// Maximum commanded roll rate in Acro / Rates mode (rad/s)
pub const MAX_ROLL_RATE: f32 = 600.0 * DEG_TO_RAD;

/// Maximum commanded pitch rate in Acro / Rates mode (rad/s)
pub const MAX_PITCH_RATE: f32 = 200.0 * DEG_TO_RAD;

/// Maximum commanded yaw rate in Acro / Rates mode (rad/s)
pub const MAX_YAW_RATE: f32 = 100.0 * DEG_TO_RAD;

/// Maximum bank angle in Angle / Self-Leveling mode (rad)
pub const MAX_ROLL_ANGLE: f32 = 60.0 * DEG_TO_RAD;

/// Maximum pitch angle in Angle / Self-Leveling mode (rad)
pub const MAX_PITCH_ANGLE: f32 = 45.0 * DEG_TO_RAD;

/// Outer proportional gain converting attitude angle error to target rate in Angle mode (s^-1)
pub const ANGLE_P_GAIN: f32 = 4.00;

/// In-flight real-time PID potentiometer tuning configuration
pub const IN_FLIGHT_TUNING_ENABLED: bool = false;
pub const IN_FLIGHT_TUNING: InFlightTuning = InFlightTuning {
    selector_channel_idx: 0xFF,
    num_banks: 1,
    banks: [
        TuneBank {
            range_min_us: 800,
            range_max_us: 2200,
            slot_a: TuneSlot::DEFAULT,
            slot_b: TuneSlot::DEFAULT,
        },
        TuneBank::DEFAULT,
        TuneBank::DEFAULT,
        TuneBank::DEFAULT,
        TuneBank::DEFAULT,
        TuneBank::DEFAULT,
    ],
};

/// Emergency hard-deck floor in meters Above Ground Level (AGL)
pub const HARD_DECK_AGL_M: f32 = 15.0;

/// Barometer configuration
#[cfg(feature = "baro")]
pub use crate::drivers::BaroModel;

pub const BARO_ENABLED: bool = false;
#[cfg(feature = "baro")]
pub const BARO_MODEL: BaroModel = BaroModel::Bmp280;
#[cfg(feature = "baro")]
pub const BARO_I2C_ADDR: u8 = 0x77;

/// External Compass / Magnetometer configuration
#[cfg(feature = "compass")]
pub use crate::drivers::CompassModel;

pub const COMPASS_ENABLED: bool = false;
#[cfg(feature = "compass")]
pub const COMPASS_MODEL: CompassModel = CompassModel::Hmc5883l;
#[cfg(feature = "compass")]
pub const MAG_DECLINATION_RAD: f32 = 0.000000;
#[cfg(feature = "compass")]
pub const MAG_CALIBRATION: wingfc_core::compass::MagCalibration =
    wingfc_core::compass::MagCalibration {
        hard_iron: Vector3::new(0.0000, 0.0000, 0.0000),
        soft_iron: [
            [1.0000, 0.0000, 0.0000],
            [0.0000, 1.0000, 0.0000],
            [0.0000, 0.0000, 1.0000],
        ],
    };

/// GPS autonomous navigation and Return-to-Home parameters
pub const GPS_ENABLED: bool = false;
pub const RTH_MIN_SATS: u8 = 6;
pub const RTH_MAX_HDOP: f32 = 2.50;
pub const RTH_SAFE_ALTITUDE_M: f32 = 30.0;

/// Statutory operational ceiling limiter in meters Above Ground Level (AGL)
/// Note: FAA / EASA standard legal limit is 120.0 meters (400 ft) AGL.
pub const MAX_CEILING_AGL_M: f32 = 120.0;

/// PT1 low-pass filter cutoff frequency for gyro harmonic vibration rejection (Hz)
pub const GYRO_LPF_CUTOFF_HZ: f32 = 90.0;

/// VQF AHRS accelerometer low-pass filtering time constant in seconds (Laidig & Seel 2023)
pub const VQF_TAU_ACC: f32 = 3.50;

/// VQF AHRS magnetometer heading correction time constant in seconds
pub const VQF_TAU_MAG: f32 = 9.00;

/// Inner rate loop PID + FeedForward configuration
pub const RATE_PID_CONFIG: PidConfig = PidConfig {
    kp: 1.0000,
    ki: 0.1000,
    kd: 0.0100,
    kff: 0.0400,
    max_integral: 0.4000,
    dterm_lpf_hz: 40.0,
    ff_lpf_hz: 45.0,
};

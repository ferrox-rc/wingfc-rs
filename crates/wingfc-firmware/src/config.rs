//! User aircraft configuration parameters for `wingfc-firmware`.
//!
//! Generated automatically from `wingfc.yaml` by the build system.
//! Users configure their aircraft via `wingfc.yaml` without editing Rust source code.

use embassy_time::Duration;
use wingfc_core::math::{BoardOrientation, DEG_TO_RAD};
use wingfc_core::mixer::{AirframeType, ServoConfig};
use wingfc_core::mode::{FlightMode, ModeMatrix, ModeRange};
use wingfc_core::pid::PidConfig;
use wingfc_core::tuning::{InFlightTuning, TuneParameter, TuneSlot};

/// Board mounting orientation
pub const BOARD_ORIENTATION: BoardOrientation = BoardOrientation::Default;

/// Configured airframe mixer topology
pub const AIRFRAME_TYPE: AirframeType = AirframeType::Elevon;

/// Hard real-time control loop time-step (500 Hz = 0.002s)
pub const DT: f32 = 0.002;

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
    slot_a: TuneSlot {
        parameter: TuneParameter::None,
        channel_idx: 5,
        min_val: 0.0500,
        max_val: 0.5000,
    },
    slot_b: TuneSlot {
        parameter: TuneParameter::None,
        channel_idx: 6,
        min_val: 0.0050,
        max_val: 0.0500,
    },
};

/// Emergency hard-deck floor in meters Above Ground Level (AGL)
pub const HARD_DECK_AGL_M: f32 = 15.0;

/// GPS autonomous navigation and Return-to-Home parameters
#[cfg(feature = "compass")]
pub use crate::drivers::CompassModel;

/// External Compass / Magnetometer configuration
pub const COMPASS_ENABLED: bool = false;
#[cfg(feature = "compass")]
pub const COMPASS_MODEL: CompassModel = CompassModel::Qmc5883l;

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

/// Madgwick AHRS filter convergence gain beta
pub const AHRS_BETA: f32 = 0.5000;

/// Inner rate loop PID + FeedForward configuration
pub const RATE_PID_CONFIG: PidConfig = PidConfig {
    kp: 1.0000,
    ki: 0.5000,
    kd: 0.0800,
    kff: 0.0400,
    max_integral: 0.4000,
    dterm_lpf_hz: 40.0,
    ff_lpf_hz: 45.0,
};

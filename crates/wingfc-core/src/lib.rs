#![no_std]

pub mod ahrs;
#[cfg(feature = "baro")]
pub mod altitude_safety;
pub mod autolaunch;
pub mod dshot;
pub mod filter;
#[cfg(feature = "gps")]
pub mod gps;
pub mod math;
pub mod mixer;
pub mod pid;
pub mod protocol;
pub mod safety;
pub mod tuning;
#[cfg(feature = "baro")]
pub mod vertical_kf;

pub use ahrs::MadgwickAHRS;
#[cfg(feature = "baro")]
pub use altitude_safety::{AltitudeIntervention, AltitudeSafetyConfig, AltitudeSafetyController};
pub use autolaunch::{AutolaunchConfig, AutolaunchController, AutolaunchState};
pub use filter::{BiquadFilter, Pt1Filter};
#[cfg(feature = "gps")]
pub use gps::{GpsData, GpsFixType, GpsNavigation, NmeaParser, RthGuidance};
pub use math::{BoardOrientation, Quaternion, Vector3};
pub use mixer::{AirframeMixer, AirframeOutputs, AirframeType, ServoConfig};
pub use pid::{PidConfig, PidController};
pub use protocol::{CrsfTelemetryEncoder, RcChannels};
pub use safety::{ArmingSafety, ArmingStatus};
pub use tuning::{InFlightTuning, TuneParameter, TuneSlot};
#[cfg(feature = "baro")]
pub use vertical_kf::VerticalKalmanFilter;

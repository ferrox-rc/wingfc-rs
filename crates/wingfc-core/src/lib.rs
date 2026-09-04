#![no_std]

pub mod math;
pub mod filter;
pub mod ahrs;
pub mod pid;
pub mod mixer;
pub mod dshot;
pub mod protocol;
pub mod safety;
pub mod autolaunch;
#[cfg(feature = "baro")]
pub mod vertical_kf;
#[cfg(feature = "baro")]
pub mod altitude_safety;

pub use math::{Vector3, Quaternion};
pub use filter::{Pt1Filter, BiquadFilter};
pub use ahrs::MadgwickAHRS;
pub use pid::{PidController, PidConfig};
pub use mixer::{AirframeMixer, AirframeType, ServoConfig, AirframeOutputs};
pub use safety::{ArmingSafety, ArmingStatus};
pub use autolaunch::{AutolaunchController, AutolaunchConfig, AutolaunchState};
#[cfg(feature = "baro")]
pub use vertical_kf::VerticalKalmanFilter;
#[cfg(feature = "baro")]
pub use altitude_safety::{AltitudeSafetyController, AltitudeSafetyConfig, AltitudeIntervention};

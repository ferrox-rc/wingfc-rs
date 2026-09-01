#![no_std]

pub mod math;
pub mod filter;
pub mod ahrs;
pub mod pid;
pub mod mixer;
pub mod dshot;
pub mod protocol;

pub use math::{Vector3, Quaternion};
pub use filter::{Pt1Filter, BiquadFilter};
pub use ahrs::MadgwickAHRS;
pub use pid::{PidController, PidConfig};
pub use mixer::{AirframeMixer, AirframeType, ServoConfig, AirframeOutputs};

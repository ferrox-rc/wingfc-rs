//! Inter-task mailbox for the 3-tier multi-rate architecture.
//!
//! - `IMU_WATCH`: 1.666 kHz INT1 DRDY samples decimated 8:1 to 208.33 Hz (Tier 1 -> Tier 2)

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use wingfc_core::downsample::ImuDecimated;

/// Decimated IMU frames published at 208.33 Hz (1.666 kHz / 8).
pub static IMU_WATCH: Watch<ThreadModeRawMutex, ImuDecimated, 2> = Watch::new();

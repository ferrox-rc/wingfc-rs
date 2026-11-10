#[cfg(feature = "baro")]
pub use wingfc_drivers::baro;
#[cfg(feature = "compass")]
pub use wingfc_drivers::compass;
pub mod imu;
pub mod led;
pub mod pwm;
pub mod qspi;

#[cfg(feature = "baro")]
pub use baro::BaroModel;
#[cfg(feature = "compass")]
pub use compass::CompassModel;
pub use imu::{imu_worker_task, new_board_imu, ImuError};
pub use led::{init_board_leds, LedColor, RgbLedDriver, StatusLed};
#[cfg(not(feature = "dshot"))]
pub use pwm::EscPwmDriver;
pub use pwm::ServoPwmDriver;
pub use qspi::QspiDriver;

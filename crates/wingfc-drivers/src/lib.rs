#![no_std]

#[cfg(feature = "baro")]
pub mod baro;
#[cfg(feature = "compass")]
pub mod compass;
pub mod imu;
pub mod led;
pub mod traits;

#[cfg(feature = "baro")]
pub use baro::{pressure_to_altitude, BaroError, BaroModel, Bmp280Driver, Dps310Driver};
#[cfg(feature = "compass")]
pub use compass::{
    CompassError, CompassModel, Hmc5883lDriver, Qmc5883lDriver, HMC5883L_ADDR, QMC5883L_ADDR,
};
pub use imu::{ImuError, Lsm6ds3Config, Lsm6ds3Driver, LSM6DS3_ADDR};
pub use led::{LedColor, NoPin, RgbLedDriver, StatusLed};
pub use traits::{
    FlashError, MotorOutput, NorFlashStorage, PageBuffer, ServoOutput, FLASH_PAGE_SIZE,
    FLASH_SECTOR_SIZE,
};

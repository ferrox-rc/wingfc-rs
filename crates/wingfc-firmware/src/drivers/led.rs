use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::peripherals;
use embassy_nrf::Peri;
pub use wingfc_drivers::led::{LedColor, StatusLed};

pub type RgbLedDriver = wingfc_drivers::led::RgbLedDriver<
    Output<'static>,
    Output<'static>,
    Output<'static>,
    Output<'static>,
>;

/// Initializes the active-LOW 4-channel RGB + User LED driver on the Seeed Studio XIAO nRF52840 Sense.
pub fn init_board_leds(
    red_pin: Peri<'static, peripherals::P0_26>,
    green_pin: Peri<'static, peripherals::P0_30>,
    blue_pin: Peri<'static, peripherals::P0_06>,
    user_pin: Peri<'static, peripherals::P0_17>,
) -> RgbLedDriver {
    let red = Output::new(red_pin, Level::High, OutputDrive::Standard);
    let green = Output::new(green_pin, Level::High, OutputDrive::Standard);
    let blue = Output::new(blue_pin, Level::High, OutputDrive::Standard);
    let user = Output::new(user_pin, Level::High, OutputDrive::Standard);

    wingfc_drivers::led::RgbLedDriver::new(red, green, blue, user)
}

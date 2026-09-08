use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::peripherals;
use embassy_nrf::Peri;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LedColor {
    Off,
    Red,
    Green,
    Blue,
    Cyan,
    Yellow,
    White,
}

pub struct RgbLedDriver {
    red: Output<'static>,
    green: Output<'static>,
    blue: Output<'static>,
}

impl RgbLedDriver {
    pub fn new(
        red_pin: Peri<'static, peripherals::P0_26>,
        green_pin: Peri<'static, peripherals::P0_30>,
        blue_pin: Peri<'static, peripherals::P0_06>,
    ) -> Self {
        // Active-low: start with all LEDs OFF (Level::High)
        let red = Output::new(red_pin, Level::High, OutputDrive::Standard);
        let green = Output::new(green_pin, Level::High, OutputDrive::Standard);
        let blue = Output::new(blue_pin, Level::High, OutputDrive::Standard);

        Self { red, green, blue }
    }

    /// Sets the physical RGB color immediately (Active-LOW: low = ON, high = OFF)
    pub fn set_color(&mut self, color: LedColor) {
        match color {
            LedColor::Off => {
                self.red.set_high();
                self.green.set_high();
                self.blue.set_high();
            }
            LedColor::Red => {
                self.red.set_low();
                self.green.set_high();
                self.blue.set_high();
            }
            LedColor::Green => {
                self.red.set_high();
                self.green.set_low();
                self.blue.set_high();
            }
            LedColor::Blue => {
                self.red.set_high();
                self.green.set_high();
                self.blue.set_low();
            }
            LedColor::Cyan => {
                self.red.set_high();
                self.green.set_low();
                self.blue.set_low();
            }
            LedColor::Yellow => {
                self.red.set_low();
                self.green.set_low();
                self.blue.set_high();
            }
            LedColor::White => {
                self.red.set_low();
                self.green.set_low();
                self.blue.set_low();
            }
        }
    }
}

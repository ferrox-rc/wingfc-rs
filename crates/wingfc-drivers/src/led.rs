//! Generic Status & RGB LED driver built on `embedded_hal::digital::OutputPin`.
//!
//! Supports full 4-pin RGB + User LED boards (like Seeed Studio XIAO nRF52840 Sense)
//! as well as single-LED boards (like Seeed Studio XIAO STM32C5) by substituting [`NoPin`]
//! for absent RGB channels.

use core::convert::Infallible;
use embedded_hal::digital::{ErrorType, OutputPin};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LedColor {
    Off,
    Red,
    Green,
    Blue,
    Cyan,
    Yellow,
    White,
}

/// Zero-cost dummy output pin for boards that lack discrete RGB channels (e.g. XIAO STM32C5).
#[derive(Debug, Default, Clone, Copy)]
pub struct NoPin;

impl ErrorType for NoPin {
    type Error = Infallible;
}

impl OutputPin for NoPin {
    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Hardware-agnostic status LED controller.
pub trait StatusLed {
    /// Sets the primary RGB status color.
    fn set_color(&mut self, color: LedColor);
    /// Sets the secondary discrete user indicator LED.
    fn set_user_led(&mut self, on: bool);
}

/// Active-LOW RGB + User LED driver generic over `embedded_hal::digital::OutputPin`.
pub struct RgbLedDriver<R, G, B, U> {
    red: R,
    green: G,
    blue: B,
    user: U,
}

impl<R, G, B, U> RgbLedDriver<R, G, B, U>
where
    R: OutputPin,
    G: OutputPin,
    B: OutputPin,
    U: OutputPin,
{
    /// Wraps active-LOW output pins and ensures all LEDs start in the OFF (`high`) state.
    pub fn new(mut red: R, mut green: G, mut blue: B, mut user: U) -> Self {
        let _ = red.set_high();
        let _ = green.set_high();
        let _ = blue.set_high();
        let _ = user.set_high();
        Self {
            red,
            green,
            blue,
            user,
        }
    }

    /// Sets the physical RGB color immediately (Active-LOW: low = ON, high = OFF).
    pub fn set_color(&mut self, color: LedColor) {
        match color {
            LedColor::Off => {
                let _ = self.red.set_high();
                let _ = self.green.set_high();
                let _ = self.blue.set_high();
            }
            LedColor::Red => {
                let _ = self.red.set_low();
                let _ = self.green.set_high();
                let _ = self.blue.set_high();
            }
            LedColor::Green => {
                let _ = self.red.set_high();
                let _ = self.green.set_low();
                let _ = self.blue.set_high();
            }
            LedColor::Blue => {
                let _ = self.red.set_high();
                let _ = self.green.set_high();
                let _ = self.blue.set_low();
            }
            LedColor::Cyan => {
                let _ = self.red.set_high();
                let _ = self.green.set_low();
                let _ = self.blue.set_low();
            }
            LedColor::Yellow => {
                let _ = self.red.set_low();
                let _ = self.green.set_low();
                let _ = self.blue.set_high();
            }
            LedColor::White => {
                let _ = self.red.set_low();
                let _ = self.green.set_low();
                let _ = self.blue.set_low();
            }
        }
    }

    /// Controls the discrete User LED (Active-LOW: on = low, off = high).
    pub fn set_user_led(&mut self, on: bool) {
        if on {
            let _ = self.user.set_low();
        } else {
            let _ = self.user.set_high();
        }
    }
}

impl<R, G, B, U> StatusLed for RgbLedDriver<R, G, B, U>
where
    R: OutputPin,
    G: OutputPin,
    B: OutputPin,
    U: OutputPin,
{
    #[inline]
    fn set_color(&mut self, color: LedColor) {
        RgbLedDriver::set_color(self, color);
    }

    #[inline]
    fn set_user_led(&mut self, on: bool) {
        RgbLedDriver::set_user_led(self, on);
    }
}

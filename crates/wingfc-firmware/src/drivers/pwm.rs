use embassy_nrf::gpio::OutputDrive;
use embassy_nrf::pwm::{Instance, Prescaler, SimplePwm};

/// Multi-Channel Servo PWM Driver with configurable frame frequency (50Hz - 400Hz) and 1us resolution.
///
/// NOTE on nRF52 PWM Hardware Polarity:
/// In the Nordic nRF52 PWM peripheral with `CounterMode::Up` and default sequence bit 15 = 0,
/// the output pin starts LOW at counter = 0, and transitions HIGH at counter = compare value.
/// At counter = countertop, the pin resets to LOW.
/// Therefore, to generate an active-HIGH RC servo pulse (e.g. 1000..2000 us HIGH followed by
/// 18000..19000 us LOW), the compare value must be loaded as `countertop - pulse_us`.
pub struct ServoPwmDriver<'d, T: Instance> {
    pwm: SimplePwm<'d, T>,
    current_us: [u16; 3],
}

impl<'d, T: Instance> ServoPwmDriver<'d, T> {
    pub fn new(pwm: SimplePwm<'d, T>, frequency_hz: u32) -> Self {
        let mut driver = Self {
            pwm,
            current_us: [0; 3],
        };
        driver.init(frequency_hz);
        driver
    }

    fn init(&mut self, frequency_hz: u32) {
        // HighDrive output drive for clean, sharp 3.3V logic transitions to servo signal lines
        self.pwm.set_ch0_drive(OutputDrive::Standard0HighDrive1);
        self.pwm.set_ch1_drive(OutputDrive::Standard0HighDrive1);
        self.pwm.set_ch2_drive(OutputDrive::Standard0HighDrive1);

        // Prescaler Div16 = 1 MHz clock (1 tick = 1 microsecond)
        self.pwm.set_prescaler(Prescaler::Div16);
        // Valid frequency range: 50 Hz (20,000 ticks) to 400 Hz (2,500 ticks)
        // Standard analog servos require 50 Hz; digital servos can accept up to 400 Hz.
        let clamped_hz = frequency_hz.clamp(50, 400);
        self.pwm.set_period(clamped_hz);

        // Center 3 channels at 1500us on boot
        let duty = 1500 * self.pwm.max_duty() / self.pwm.period() as u16;
        self.pwm.set_duty(0, duty);
        self.pwm.set_duty(1, duty);
        self.pwm.set_duty(2, duty);
        self.current_us = [1500, 1500, 1500];
    }

    /// Update pulse width for a specific channel (1000..2000 us)
    #[inline(always)]
    pub fn set_channel_us(&mut self, channel: usize, pulse_us: u16) {
        if channel < 3 {
            let clamped = pulse_us.clamp(900, 2200);
            if self.current_us[channel] != clamped {
                self.current_us[channel] = clamped;
                let compare = self.pwm.max_duty().saturating_sub(clamped);
                self.pwm.set_duty(channel, compare);
            }
        }
    }
}

/// Configurable ESC PWM Driver (50Hz - 400Hz) with 1us resolution.
///
/// Converts commanded throttle (1000..2000 us) into active-HIGH PWM pulses.
pub struct EscPwmDriver<'d, T: Instance> {
    pwm: SimplePwm<'d, T>,
    current_throttle: u16,
}

impl<'d, T: Instance> EscPwmDriver<'d, T> {
    pub fn new(pwm: SimplePwm<'d, T>, frequency_hz: u32) -> Self {
        let mut driver = Self {
            pwm,
            current_throttle: 0,
        };
        driver.init(frequency_hz);
        driver
    }

    fn init(&mut self, frequency_hz: u32) {
        self.pwm.set_ch0_drive(OutputDrive::Standard0HighDrive1);

        // Prescaler Div16 = 1 MHz clock (1 tick = 1 microsecond)
        self.pwm.set_prescaler(Prescaler::Div16);
        // Valid frequency range: 50 Hz (20,000 ticks) to 400 Hz (2,500 ticks)
        let clamped_hz = frequency_hz.clamp(50, 400);
        self.pwm.set_period(clamped_hz);

        // Minimum throttle 1000us on boot
        self.current_throttle = 1000;
        let duty = self.current_throttle * self.pwm.max_duty() / self.pwm.period() as u16;
        self.pwm.set_duty(0, duty);
    }

    /// Update throttle pulse width (1000..2000 us)
    #[inline(always)]
    pub fn set_throttle_us(&mut self, pulse_us: u16) {
        let clamped = pulse_us.clamp(1000, 2000);
        if self.current_throttle != clamped {
            self.current_throttle = clamped;
            let compare = self.pwm.max_duty().saturating_sub(clamped);
            self.pwm.set_duty(0, compare);
        }
    }
}

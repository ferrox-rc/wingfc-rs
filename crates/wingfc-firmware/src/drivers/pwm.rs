use embassy_nrf::pwm::{Instance, Prescaler, SimplePwm};

/// Multi-Channel Servo PWM Driver with configurable frame frequency (50Hz - 400Hz) and 1us resolution
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
        // Prescaler Div16 = 1 MHz clock (1 tick = 1 microsecond)
        self.pwm.set_prescaler(Prescaler::Div16);
        // Valid frequency range: 50 Hz (20,000 ticks) to 400 Hz (2,500 ticks)
        // Standard analog servos require 50 Hz; digital servos can accept up to 400 Hz.
        let clamped_hz = frequency_hz.clamp(50, 400);
        let max_ticks = (1_000_000 / clamped_hz) as u16;
        self.pwm.set_max_duty(max_ticks);
        // Center 3 channels at 1500us on boot
        self.pwm.set_duty(0, 1500);
        self.pwm.set_duty(1, 1500);
        self.pwm.set_duty(2, 1500);
        self.current_us = [1500, 1500, 1500];
    }

    /// Update pulse width for a specific channel (1000..2000 us)
    #[inline(always)]
    pub fn set_channel_us(&mut self, channel: usize, pulse_us: u16) {
        if channel < 3 {
            let clamped = pulse_us.clamp(900, 2200);
            if self.current_us[channel] != clamped {
                self.current_us[channel] = clamped;
                self.pwm.set_duty(channel, clamped);
            }
        }
    }
}

/// Configurable ESC PWM Driver (50Hz - 400Hz) with 1us resolution
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
        // Prescaler Div16 = 1 MHz clock (1 tick = 1 microsecond)
        self.pwm.set_prescaler(Prescaler::Div16);
        // Valid frequency range: 50 Hz (20,000 ticks) to 400 Hz (2,500 ticks)
        let clamped_hz = frequency_hz.clamp(50, 400);
        let max_ticks = (1_000_000 / clamped_hz) as u16;
        self.pwm.set_max_duty(max_ticks);
        // Minimum throttle 1000us on boot
        self.pwm.set_duty(0, 1000);
        self.current_throttle = 1000;
    }

    /// Update throttle pulse width (1000..2000 us)
    #[inline(always)]
    pub fn set_throttle_us(&mut self, pulse_us: u16) {
        let clamped = pulse_us.clamp(1000, 2000);
        if self.current_throttle != clamped {
            self.current_throttle = clamped;
            self.pwm.set_duty(0, clamped);
        }
    }
}

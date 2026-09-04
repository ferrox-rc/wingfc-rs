use embassy_nrf::pwm::{Instance, Prescaler, SimplePwm};

/// Multi-Channel Servo PWM Driver with configurable frame frequency (50Hz - 400Hz) and 1us resolution
pub struct ServoPwmDriver<'d, T: Instance> {
    pwm: SimplePwm<'d, T>,
}

impl<'d, T: Instance> ServoPwmDriver<'d, T> {
    pub fn new(pwm: SimplePwm<'d, T>, frequency_hz: u32) -> Self {
        let mut driver = Self { pwm };
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
        // Center all channels at 1500us on boot
        self.pwm.set_duty(0, 1500);
        self.pwm.set_duty(1, 1500);
        self.pwm.set_duty(2, 1500);
        self.pwm.set_duty(3, 1500);
    }

    /// Update pulse width for a specific channel (1000..2000 us)
    #[inline(always)]
    pub fn set_channel_us(&mut self, channel: usize, pulse_us: u16) {
        let clamped = pulse_us.clamp(900, 2200);
        self.pwm.set_duty(channel, clamped);
    }
}

/// Configurable ESC PWM Driver (50Hz - 400Hz) with 1us resolution
pub struct EscPwmDriver<'d, T: Instance> {
    pwm: SimplePwm<'d, T>,
}

impl<'d, T: Instance> EscPwmDriver<'d, T> {
    pub fn new(pwm: SimplePwm<'d, T>, frequency_hz: u32) -> Self {
        let mut driver = Self { pwm };
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
    }

    /// Update throttle pulse width (1000..2000 us)
    #[inline(always)]
    pub fn set_throttle_us(&mut self, pulse_us: u16) {
        let clamped = pulse_us.clamp(1000, 2000);
        self.pwm.set_duty(0, clamped);
    }
}

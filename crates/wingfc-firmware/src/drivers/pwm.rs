use embassy_nrf::pwm::{Instance, Prescaler, SimplePwm};

/// Multi-Channel Servo PWM Driver for 50Hz Servos with 1us resolution
pub struct ServoPwmDriver<'d, T: Instance> {
    pwm: SimplePwm<'d, T>,
}

impl<'d, T: Instance> ServoPwmDriver<'d, T> {
    pub fn new(pwm: SimplePwm<'d, T>) -> Self {
        let mut driver = Self { pwm };
        driver.init();
        driver
    }

    fn init(&mut self) {
        // Prescaler Div16 = 1 MHz clock (1 tick = 1 microsecond)
        self.pwm.set_prescaler(Prescaler::Div16);
        // 50Hz frame rate = 20,000 ticks (20ms period)
        self.pwm.set_max_duty(20_000);
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

/// Dedicated 400Hz High-Speed ESC PWM Driver
pub struct EscPwmDriver<'d, T: Instance> {
    pwm: SimplePwm<'d, T>,
}

impl<'d, T: Instance> EscPwmDriver<'d, T> {
    pub fn new(pwm: SimplePwm<'d, T>) -> Self {
        let mut driver = Self { pwm };
        driver.init();
        driver
    }

    fn init(&mut self) {
        // Prescaler Div16 = 1 MHz clock (1 tick = 1 microsecond)
        self.pwm.set_prescaler(Prescaler::Div16);
        // 400Hz frame rate = 2,500 ticks (2.5ms period) - 8x faster than analog 50Hz!
        self.pwm.set_max_duty(2_500);
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

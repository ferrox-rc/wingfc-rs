use embassy_nrf::gpio::OutputDrive;
use embassy_nrf::pwm::{CounterMode, DutyCycle, Prescaler, SimpleConfig, SimplePwm};

/// Multi-Channel Servo PWM Driver with configurable frame frequency (50Hz - 400Hz) and 1us resolution.
///
/// NOTE on nRF52 PWM Hardware Polarity & DutyCycle:
/// In the Nordic nRF52 PWM peripheral with `CounterMode::Up`, `DutyCycle::inverted(pulse_us)` sets
/// bit 15 = 1, ensuring the output pin is driven HIGH while the counter is below `pulse_us`,
/// and transitions to LOW for the remainder of the period (`max_duty`).
/// This produces a standard active-HIGH RC PWM servo pulse (e.g. 1000..2000 us HIGH followed by
/// 18000..19000 us LOW at 50 Hz).
pub struct ServoPwmDriver<'d> {
    pwm: SimplePwm<'d>,
    current_us: [u16; 3],
}

impl<'d> ServoPwmDriver<'d> {
    /// Creates the `SimpleConfig` required to instantiate `SimplePwm::new_3ch`.
    pub fn config(frequency_hz: u32) -> SimpleConfig {
        let mut config = SimpleConfig::default();
        let clamped_hz = frequency_hz.clamp(50, 400);
        let max_duty = (1_000_000 / clamped_hz) as u16;

        config.counter_mode = CounterMode::Up;
        config.prescaler = Prescaler::Div16;
        config.max_duty = max_duty;
        config.ch0_drive = OutputDrive::HighDrive;
        config.ch1_drive = OutputDrive::HighDrive;
        config.ch2_drive = OutputDrive::HighDrive;
        config
    }

    /// Wraps the initialized `SimplePwm` instance and centers servos at 1500us.
    pub fn new(mut pwm: SimplePwm<'d>) -> Self {
        let center = DutyCycle::inverted(1500);
        pwm.set_all_duties([center, center, center, DutyCycle::normal(0)]);
        Self {
            pwm,
            current_us: [1500, 1500, 1500],
        }
    }

    /// Update pulse width for a specific channel (1000..2000 us)
    #[inline(always)]
    pub fn set_channel_us(&mut self, channel: usize, pulse_us: u16) {
        if channel < 3 {
            let clamped = pulse_us.clamp(900, 2200);
            if self.current_us[channel] != clamped {
                self.current_us[channel] = clamped;
                self.pwm.set_duty(channel, DutyCycle::inverted(clamped));
            }
        }
    }
}

/// Configurable ESC PWM Driver (50Hz - 400Hz) with 1us resolution.
///
/// Converts commanded throttle (1000..2000 us) into active-HIGH PWM pulses.
pub struct EscPwmDriver<'d> {
    pwm: SimplePwm<'d>,
    current_throttle: u16,
}

impl<'d> EscPwmDriver<'d> {
    /// Creates the `SimpleConfig` required to instantiate `SimplePwm::new_1ch`.
    pub fn config(frequency_hz: u32) -> SimpleConfig {
        let mut config = SimpleConfig::default();
        let clamped_hz = frequency_hz.clamp(50, 400);
        let max_duty = (1_000_000 / clamped_hz) as u16;

        config.counter_mode = CounterMode::Up;
        config.prescaler = Prescaler::Div16;
        config.max_duty = max_duty;
        config.ch0_drive = OutputDrive::HighDrive;
        config
    }

    /// Wraps the initialized `SimplePwm` instance and initializes throttle to 1000us idle.
    pub fn new(mut pwm: SimplePwm<'d>) -> Self {
        let idle = DutyCycle::inverted(1000);
        pwm.set_duty(0, idle);
        Self {
            pwm,
            current_throttle: 1000,
        }
    }

    /// Update throttle pulse width (1000..2000 us)
    #[inline(always)]
    pub fn set_throttle_us(&mut self, pulse_us: u16) {
        let clamped = pulse_us.clamp(1000, 2000);
        if self.current_throttle != clamped {
            self.current_throttle = clamped;
            self.pwm.set_duty(0, DutyCycle::inverted(clamped));
        }
    }
}

use crate::filter::Pt1Filter;
use crate::math::clamp;

#[derive(Debug, Clone, Copy)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub kff: f32, // FeedForward gain for instantaneous aerodynamic stick response
    pub max_integral: f32,
    pub dterm_lpf_hz: f32,
    pub ff_lpf_hz: f32, // Low-pass filter cutoff for FeedForward derivative
}

impl Default for PidConfig {
    fn default() -> Self {
        Self {
            kp: 0.15,
            ki: 0.05,
            kd: 0.01,
            kff: 0.05,
            max_integral: 0.5,
            dterm_lpf_hz: 30.0,
            ff_lpf_hz: 45.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PidController {
    pub config: PidConfig,
    integral: f32,
    last_measurement: f32,
    last_setpoint: f32,
    dterm_filter: Pt1Filter,
    ff_filter: Pt1Filter,
}

impl PidController {
    pub fn new(config: PidConfig, dt: f32) -> Self {
        let mut dterm_filter = Pt1Filter::new();
        dterm_filter.init(config.dterm_lpf_hz, dt);

        let mut ff_filter = Pt1Filter::new();
        ff_filter.init(config.ff_lpf_hz, dt);

        Self {
            config,
            integral: 0.0,
            last_measurement: 0.0,
            last_setpoint: 0.0,
            dterm_filter,
            ff_filter,
        }
    }

    #[inline(always)]
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_measurement = 0.0;
        self.last_setpoint = 0.0;
        self.dterm_filter.reset(0.0);
        self.ff_filter.reset(0.0);
    }

    #[inline(always)]
    pub fn integral(&self) -> f32 {
        self.integral
    }

    /// Update PID loop using Setpoint and Process Measurement.
    ///
    /// - **Proportional (P):** Corrects current tracking error.
    /// - **Integral (I):** Corrects steady-state bias with anti-windup clamp.
    /// - **Derivative (D):** Derivative-on-Measurement damping to eliminate stick derivative kick.
    /// - **FeedForward (FF):** Derivative-on-Setpoint to overcome servo mechanical latency.
    #[inline(always)]
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;

        // Proportional term
        let p_term = self.config.kp * error;

        // Integral term with anti-windup clamp
        self.integral += self.config.ki * error * dt;
        self.integral = clamp(self.integral, -self.config.max_integral, self.config.max_integral);

        // Derivative on Measurement (with sign inverted: -d(measurement)/dt)
        let raw_d_measurement = if dt > 0.0 {
            (measurement - self.last_measurement) / dt
        } else {
            0.0
        };
        self.last_measurement = measurement;
        let filtered_d = self.dterm_filter.update(raw_d_measurement);
        let d_term = -self.config.kd * filtered_d;

        // FeedForward on Setpoint: d(setpoint)/dt
        let raw_d_setpoint = if dt > 0.0 {
            (setpoint - self.last_setpoint) / dt
        } else {
            0.0
        };
        self.last_setpoint = setpoint;
        let filtered_ff = self.ff_filter.update(raw_d_setpoint);
        let ff_term = self.config.kff * filtered_ff;

        p_term + self.integral + d_term + ff_term
    }
}

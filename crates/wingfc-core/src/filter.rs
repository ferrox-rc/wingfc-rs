use core::f32::consts::PI;
use libm::{sqrtf, tanf};

/// PT1 1st-Order Low-Pass Filter
#[derive(Debug, Clone, Copy, Default)]
pub struct Pt1Filter {
    state: f32,
    k: f32,
}

impl Pt1Filter {
    pub const fn new() -> Self {
        Self { state: 0.0, k: 0.0 }
    }

    pub fn init(&mut self, cutoff_hz: f32, dt: f32) {
        if cutoff_hz <= 0.0 || dt <= 0.0 {
            self.k = 1.0;
        } else {
            let rc = 1.0 / (2.0 * PI * cutoff_hz);
            self.k = dt / (rc + dt);
        }
        self.state = 0.0;
    }

    #[inline(always)]
    pub fn update(&mut self, input: f32) -> f32 {
        self.state += self.k * (input - self.state);
        self.state
    }

    #[inline(always)]
    pub fn reset(&mut self, value: f32) {
        self.state = value;
    }

    #[inline(always)]
    pub fn state(&self) -> f32 {
        self.state
    }
}

/// 2nd-Order Biquad Butterworth Low-Pass Filter
#[derive(Debug, Clone, Copy, Default)]
pub struct BiquadFilter {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl BiquadFilter {
    pub const fn new() -> Self {
        Self {
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    pub fn init(&mut self, cutoff_hz: f32, sample_rate_hz: f32) {
        if cutoff_hz <= 0.0 || sample_rate_hz <= 0.0 {
            return;
        }
        let fr = sample_rate_hz / cutoff_hz;
        let omega = tanf(PI / fr);
        let c = 1.0 + sqrtf(2.0) * omega + omega * omega;

        self.b0 = (omega * omega) / c;
        self.b1 = 2.0 * self.b0;
        self.b2 = self.b0;
        self.a1 = (2.0 * (omega * omega - 1.0)) / c;
        self.a2 = (1.0 - sqrtf(2.0) * omega + omega * omega) / c;

        self.reset();
    }

    #[inline(always)]
    pub fn update(&mut self, input: f32) -> f32 {
        let result = self.b0 * input + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = result;
        result
    }

    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }
}

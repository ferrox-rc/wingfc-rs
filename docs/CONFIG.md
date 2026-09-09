| Parameter | Options / Range | Description |
| :--- | :--- | :--- |
| `enabled` | `true`, `false` | Enables live gain updates from transmitter potentiometers/sliders. |
| `slot_a.parameter`<br>`slot_b.parameter` | `none`, `pitch_kp`, `roll_kp`, `pitch_ki`, `roll_ki`, `pitch_kd`, `roll_kd`, `pitch_kff`, `roll_kff` | Specific PID/FeedForward gain modified by the corresponding channel. |
| `slot_a.channel`<br>`slot_b.channel` | `1` .. `16` | RC Channel number allocated to the adjustment pot. |
| `min_val` .. `max_val` | Floating point values | Gain range mapped linearly across the 1000..2000 &mu;s stick/pot sweep. |

---

### Motor Driver, Frequency & Differential Thrust
```yaml
motor:
  protocol: pwm
  pwm_frequency_hz: 400
  differential_thrust: 0.15
  min_throttle_us: 1000
  max_throttle_us: 2000
```

| Parameter | Type / Range | Default | Description |
| :--- | :--- | :--- | :--- |
| `protocol` | `pwm`, `dshot` | `pwm` | Motor ESC output protocol. `pwm` uses hardware timers; `dshot` uses digital frames (DShot300/600). |
| `pwm_frequency_hz` | `50` .. `400` Hz | `400` Hz | PWM frame frequency when protocol is `pwm`. Standard airplane ESCs require 50..60 Hz; multirotor ESCs support 400 Hz. |
| `differential_thrust` | `0.0` .. `1.0` | `0.15` | Yaw authority scaling on twin-motor airframes (`elevon_diff_thrust`). Fraction of 1000 &mu;s throttle range (e.g. 0.15 = &plusmn;150 &mu;s max split). |
| `min_throttle_us` | `900` .. `1200` &mu;s | `1000` &mu;s | Minimum throttle command pulse width. |
| `max_throttle_us` | `1800` .. `2100` &mu;s | `2000` &mu;s | Maximum full-power throttle pulse width. |

---

### Autonomous Navigation Suite

| Subsystem | Configuration | Reference Specification |
| :--- | :--- | :--- |
| **GPS Receiver Link** | NMEA parser @ 115,200 baud on Sense Plus `UARTE1` | [GPS Navigation Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/GPS_NAVIGATION.md) |
| **Fail-safe Return-to-Home** | Autonomous climb, loiter, and 120m statutory ceiling | [GPS Navigation Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/GPS_NAVIGATION.md) |

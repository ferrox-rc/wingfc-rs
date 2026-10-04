# Aircraft Configuration Guide (`wingfc.yaml`)

`wingfc-rs` allows pilots and developers to configure their aircraft hardware, airframe mixers, receiver protocols, loop timing, and PID gains through a single, human-readable YAML configuration file ([`wingfc.yaml`](../wingfc.yaml)), eliminating the need to modify Rust source code or Cargo feature flags.

The build engine (`./scripts/build.sh` or `python3 scripts/builder.py`) automatically parses, validates, and generates zero-cost compile-time Rust parameters in `crates/wingfc-firmware/src/config.rs`.

---

## 1. Hardware, Orientation & Loop Timing

```yaml
# Target MCU Board
board: xiao-sense         # Options: xiao-sense, xiao-sense-plus

# 8-Way Board Mounting Orientation
orientation: default      # Options: default, cw90, cw180, cw270, flip, flip_cw90, flip_cw180, flip_cw270

# Airframe Mixer Topology
airframe: elevon          # Options: elevon, elevon_diff_thrust, single_aileron_t_tail,
                          #          dual_aileron_t_tail, single_aileron_v_tail, dual_aileron_v_tail

# Hardware Loop Timing & IMU Data-Ready Synchronization:
#   standard: 416 Hz (Default, recommended for general flight, GPS waypoint navigation, and standard servos)
#   high:     833 Hz (Recommended for high-speed combat wings, DShot, and 333-400Hz digital servos)
loop_rate: standard       # Options: standard, high
```

| Parameter | Options / Range | Default | Description |
| :--- | :--- | :--- | :--- |
| `board` | `xiao-sense`, `xiao-sense-plus` | `xiao-sense` | Selects target board layout. Standard Sense provides 14 castellated pads; Sense Plus adds expansion pads for GPS UART (`UARTE1`) and 4 extra PWM servos (`PWM2`). |
| `orientation` | `default`, `cw90`, `cw180`, `cw270`, `flip`, `flip_cw90`, `flip_cw180`, `flip_cw270` | `default` | 3D coordinate transformation mapping sensor axes to standard aircraft body frame (X=forward, Y=right, Z=down). |
| `airframe` | `elevon`, `elevon_diff_thrust`, `single_aileron_t_tail`, `dual_aileron_t_tail`, `single_aileron_v_tail`, `dual_aileron_v_tail` | `elevon` | Pre-configured aerodynamic control surface mixer topology. |
| `loop_rate` | `standard` (416 Hz), `high` (833 Hz) | `standard` | Configures the LSM6DS3 hardware ODR and GPIOTE interrupt pacing. `standard` provides optimal CPU headroom for GPS navigation and standard servos; `high` provides minimum latency for aerobatic speedwings. |

---

## 2. Receiver Protocol & Channel Mapping

```yaml
receiver:
  protocol: crsf          # Options: crsf (420,000 baud), ibus (115,200 baud)
  failsafe_timeout_ms: 500 # Period of lost signal before failsafe engages

  channels:
    roll: 1               # Aileron / Roll stick (1-16)
    pitch: 2              # Elevator / Pitch stick (1-16)
    throttle: 3           # Throttle stick (1-16)
    yaw: 4                # Rudder / Yaw stick (1-16)
    arm: 5                # Arm switch (CH5 / AUX1)
    mode: 6               # Flight mode switch (CH6 / AUX2)
```

| Parameter | Options / Range | Default | Description |
| :--- | :--- | :--- | :--- |
| `protocol` | `crsf`, `ibus` | `crsf` | Serial RC receiver protocol. `crsf` (TBS Crossfire / ExpressLRS @ 420 kBaud); `ibus` (FlySky @ 115.2 kBaud). |
| `failsafe_timeout_ms` | `50` .. `5000` ms | `500` ms | Duration of packet loss before zero-surge failsafe engages. |
| `channels.*` | `1` .. `16` | Standard AETR | Transmitter channel index assignment for flight commands. |

---

## 3. Safety Interlocks & Arming Thresholds

```yaml
safety:
  arm_switch_threshold_us: 1800      # Pulse width above which aircraft arms
  throttle_zero_threshold_us: 1050   # Max throttle pulse permitted when arming
```

| Parameter | Type / Range | Default | Description |
| :--- | :--- | :--- | :--- |
| `arm_switch_threshold_us` | `1200` .. `1900` &mu;s | `1800` &mu;s | Channel pulse threshold triggering the ARM state transition. |
| `throttle_zero_threshold_us` | `950` .. `1200` &mu;s | `1050` &mu;s | Hardware throttle zero safety lockout to prevent accidental motor spin-up on arming. |

---

## 4. Servo Actuators & Output Mapping

```yaml
servos:
  pwm_frequency_hz: 50    # Servo frame frequency (50 to 400 Hz)

  servo1:
    min_us: 1000
    max_us: 2000
    subtrim_us: 0
    reversed: false
  # servo2 .. servo8 similarly configurable
```

| Parameter | Type / Range | Default | Description |
| :--- | :--- | :--- | :--- |
| `pwm_frequency_hz` | `50` .. `400` Hz | `50` Hz | PWM frame refresh rate. Standard analog servos require 50–60 Hz; digital servos support up to 333–400 Hz. |
| `min_us` / `max_us` | `800` .. `2200` &mu;s | `1000` / `2000` &mu;s | Mechanical travel endpoint limits. |
| `subtrim_us` | `-200` .. `200` &mu;s | `0` &mu;s | Center offset trim in microseconds. |
| `reversed` | `true`, `false` | `false` | Inverts servo travel direction. |

---

## 5. Motor Driver & Differential Thrust

```yaml
motor:
  protocol: pwm           # Options: pwm, dshot
  pwm_frequency_hz: 400   # Analog PWM ESC frequency (50 to 400 Hz)
  differential_thrust: 0.15 # Twin motor yaw authority (0.0 to 1.0)
  min_throttle_us: 1000
  max_throttle_us: 2000
```

| Parameter | Type / Range | Default | Description |
| :--- | :--- | :--- | :--- |
| `protocol` | `pwm`, `dshot` | `pwm` | Motor ESC output protocol. `pwm` uses hardware timers; `dshot` uses digital DShot300/600 frames. |
| `pwm_frequency_hz` | `50` .. `400` Hz | `400` Hz | PWM frame frequency for analog ESCs. Standard airplane ESCs require 50–60 Hz; multirotor ESCs support 400 Hz. |
| `differential_thrust` | `0.0` .. `1.0` | `0.15` | Yaw authority scaling on twin-motor airframes (`elevon_diff_thrust`). Fraction of 1000 &mu;s throttle range (e.g. 0.15 = &plusmn;150 &mu;s max split). |

---

## 6. In-Flight PID Tuning (Multi-Bank & Selector Switch)

```yaml
in_flight_tuning:
  enabled: false

  # Optional RC auxiliary channel (1-16) to select the active tuning bank (e.g. 3-POS switch or 6POS button).
  # If omitted or not configured, Bank 1 is permanently active.
  selector_channel: 5

  # Selectable Tuning Banks (up to 6 banks supported):
  # Tuned gains persist in memory across bank switches. Selecting a bank with parameter: 'none' locks/freezes gains.
  banks:
    # Bank 1: Switch LOW (900-1300 us) -> Tune Rate Proportional (Kp) Gains
    - switch_range: [900, 1300]
      slot_a:
        parameter: pitch_kp   # Options: none, pitch_kp, roll_kp, pitch_ki, roll_ki, pitch_kd, roll_kd,
                              #          pitch_kff, roll_kff, yaw_kp, yaw_ki, yaw_kd, yaw_kff
        channel: 9            # RC Channel number (1-16, e.g., 9 = CH9 / Slider 1)
        min_val: 0.05
        max_val: 0.50
      slot_b:
        parameter: roll_kp
        channel: 10           # RC Channel number (1-16, e.g., 10 = CH10 / Slider 2)
        min_val: 0.05
        max_val: 0.50

    # Bank 2: Switch MID (1301-1699 us) -> Safe Lock / Bypass (freeze current gains)
    - switch_range: [1301, 1699]
      slot_a:
        parameter: none
      slot_b:
        parameter: none

    # Bank 3: Switch HIGH (1700-2100 us) -> Tune Rate Derivative (Kd) Gains
    - switch_range: [1700, 2100]
      slot_a:
        parameter: pitch_kd
        channel: 9
        min_val: 0.005
        max_val: 0.050
      slot_b:
        parameter: roll_kd
        channel: 10
        min_val: 0.005
        max_val: 0.050
```

| Parameter | Options / Range | Description |
| :--- | :--- | :--- |
| `enabled` | `true`, `false` | Enables real-time PID gain adjustment via designated transmitter potentiometers or sliders. |
| `selector_channel` | `1` .. `16` (optional) | RC auxiliary channel (e.g., 3-position switch) used to switch between active tuning banks. If omitted, Bank 1 remains permanently active. |
| `banks` | List of up to 6 banks | Bank definitions containing `switch_range`, `slot_a`, and `slot_b`. |
| `switch_range` | `[min_us, max_us]` | RC pulse width window (800&ndash;2200 &mu;s) activating this tuning bank. |
| `slot_a.parameter`<br>`slot_b.parameter` | `none`, `pitch_kp`, `roll_kp`, `pitch_ki`, `roll_ki`, `pitch_kd`, `roll_kd`, `pitch_kff`, `roll_kff`, `yaw_kp`, `yaw_ki`, `yaw_kd`, `yaw_kff` | Target PID or FeedForward gain scaled by the adjustment channel. Set to `none` to freeze/lock current gains safely. |
| `slot_a.channel`<br>`slot_b.channel` | `1` .. `16` | RC Channel number allocated to the adjustment potentiometer or slider. |
| `min_val` .. `max_val` | Floating point values | Gain range mapped linearly across the 1000..2000 &mu;s pot sweep. |

> [!NOTE]
> **Gain Persistence & Safe Lock**: When switching from Bank 1 to Bank 2 (or adjusting another bank), previously tuned PID gains remain active in RAM. Assigning `parameter: none` creates a safe lock bank, allowing pilots to freeze their dialed-in gains without accidental bumping during flight.
> Legacy single-bank configurations using top-level `slot_a` and `slot_b` without `banks` remain fully supported for backward compatibility.

---

## 7. Autonomous Navigation Suite

| Subsystem | Configuration | Reference Specification |
| :--- | :--- | :--- |
| **GPS Receiver Link** | NMEA parser @ 115,200 baud on Sense Plus `UARTE1` | [GPS Navigation Specification](GPS_NAVIGATION.md) |
| **Fail-safe Return-to-Home** | Autonomous climb, loiter, and 120m statutory ceiling | [GPS Navigation Specification](GPS_NAVIGATION.md) |
| **External Magnetometer** | I2C Fast-Mode on `TWISPI1` (`D4`/`D5`) | [Architecture Specification](ARCHITECTURE.md) |


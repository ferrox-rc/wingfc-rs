# Aircraft Configuration Guide (`wingfc.yaml`)

`wingfc-rs` features a human-friendly YAML configuration system modeled after leading flight control ecosystems (such as Betaflight, INAV, and EdgeTX/ExpressLRS). 

Pilots and builders configure aircraft hardware, receiver protocols, motor drivers, servo refresh frequencies, per-servo calibration, PID gains, rates, and navigation features directly in [`wingfc.yaml`](file:///home/bryansouza/Repos/wingfc-rs/wingfc.yaml) without touching Rust source files, Cargo manifests, or linker scripts.

---

## 1. Quick Start Workflow

### Build Release Firmware
To configure and compile firmware for your aircraft:
```bash
./scripts/build.sh
```
This single command automatically:
1. Validates all parameters in [`wingfc.yaml`](file:///home/bryansouza/Repos/wingfc-rs/wingfc.yaml) (range checks, enum validation, hardware refresh limits, safety bounds, and airspace legality checks).
2. Generates the embedded flight controller configuration ([`crates/wingfc-firmware/src/config.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/config.rs)).
3. Maps options to the exact Cargo build features (`crsf` vs `ibus`, `dshot` vs `pwm`, `nav`).
4. Compiles an optimized release binary for the ARM Cortex-M4F target (`thumbv7em-none-eabihf`).
5. Generates `.elf`, `.bin`, and `.hex` artifacts in `dist/` with cryptographic SHA256 checksums and Flash/RAM resource metrics.

### Build & Verification Commands

| Command | Purpose & Description |
| :--- | :--- |
| `./scripts/build.sh check` | Validates YAML parameters and checks compilation in <0.5s without generating binaries. |
| `./scripts/build.sh test` | Runs host unit test suite across `wingfc-core` (~89% coverage, 17+ tests). |
| `./scripts/build.sh matrix check` | Verifies all 8 target firmware permutations against the ARM toolchain. |
| `./scripts/build.sh matrix build` | Compiles and packages all 8 release firmware binaries into `dist/`. |
| `./scripts/build.sh init` | Resets a clean default template of `wingfc.yaml` (backs up current file to `.bak`). |
| `./scripts/build.sh flash` | Auto-detects UF2 bootloader drive, J-Link SWD, or serial DFU (see [Flashing Guide](file:///home/bryansouza/Repos/wingfc-rs/docs/FLASHING.md)). |

---

## 2. Configuration Reference (`wingfc.yaml`)

### Target Board & Mounting Orientation
```yaml
board: xiao-sense
orientation: default
```

| Parameter | Options | Description |
| :--- | :--- | :--- |
| `board` | `xiao-sense`, `xiao-sense-plus` | Standard XIAO Sense (onboard 6-DoF IMU) or Sense Plus (required for GPS & Servos 5..8). |
| `orientation` | `default`, `cw90`, `cw180`, `cw270`, `flip`, `flip_cw90`, `flip_cw180`, `flip_cw270` | 8-way orthogonal IMU transform matrix, mapping sensor axes to airframe frame without trigonometric runtime cost. |

---

### Airframe Mixer Topologies
```yaml
airframe: elevon
```

The flight controller supports 6 primary airframe topologies, automatically assigning pitch, roll, and yaw stabilization across output servos and motors:

| Airframe Topology Key | Vehicle Description | Primary Servos (`PWM1`) | Motors (`PWM0`) |
| :--- | :--- | :--- | :--- |
| `elevon` | Flying wing / delta wing | **S1:** Left Elevon<br>**S2:** Right Elevon<br>**S3:** Rudder / Aux<br>**S4:** Aux | **M1:** Main Throttle<br>**M2:** Passthrough |
| `elevon_diff_thrust` | Flying wing with twin motors | **S1:** Left Elevon<br>**S2:** Right Elevon<br>**S3:** Rudder / Aux<br>**S4:** Aux | **M1:** Left Motor (Yaw -)<br>**M2:** Right Motor (Yaw +) |
| `single_aileron_t_tail` | Conventional airplane (Single Aileron) | **S1:** Ailerons (Y-cable)<br>**S2:** Elevator<br>**S3:** Rudder<br>**S4:** Aux | **M1:** Main Throttle<br>**M2:** Passthrough |
| `dual_aileron_t_tail` | Conventional airplane (Dual Ailerons) | **S1:** Left Aileron<br>**S2:** Elevator<br>**S3:** Rudder<br>**S4:** Right Aileron (-roll) | **M1:** Main Throttle<br>**M2:** Passthrough |
| `single_aileron_v_tail` | V-tail airplane (Single Aileron) | **S1:** Ailerons (Y-cable)<br>**S2:** Right V-Tail<br>**S3:** Left V-Tail<br>**S4:** Aux | **M1:** Main Throttle<br>**M2:** Passthrough |
| `dual_aileron_v_tail` | Dual aileron V-tail airplane | **S1:** Left Aileron<br>**S2:** Right V-Tail<br>**S3:** Left V-Tail<br>**S4:** Right Aileron (-roll) | **M1:** Main Throttle<br>**M2:** Passthrough |

> For pinout numbers and hardware PWM routing, see the [Pin Configuration Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/PINOUT.md).

---

### Receiver Protocol & Channel Mapping
```yaml
receiver:
  protocol: crsf
  failsafe_timeout_ms: 500
  channels:
    roll: 1      # Roll / Aileron stick (CH1)
    pitch: 2     # Pitch / Elevator stick (CH2)
    throttle: 3  # Throttle stick (CH3)
    yaw: 4       # Yaw / Rudder stick (CH4)
    arm: 5       # Arm switch (CH5 / AUX1)
    mode: 6      # Flight mode switch (CH6 / AUX2: Acro vs Angle)
```

| Parameter | Type / Range | Default | Description |
| :--- | :--- | :--- | :--- |
| `protocol` | `crsf`, `ibus` | `crsf` | Serial receiver protocol: CRSF/ELRS @ 420 kBaud or FlySky iBus @ 115.2 kBaud. |
| `failsafe_timeout_ms` | `50` .. `5000` ms | `500` ms | Maximum packet dropout duration before engaging failsafe (cuts motors, neutralizes glide). |
| `channels.roll` | `1` .. `16` | `1` | RC channel index mapped to Roll / Aileron command. |
| `channels.pitch` | `1` .. `16` | `2` | RC channel index mapped to Pitch / Elevator command. |
| `channels.throttle` | `1` .. `16` | `3` | RC channel index mapped to Throttle command. |
| `channels.yaw` | `1` .. `16` | `4` | RC channel index mapped to Yaw / Rudder command. |
| `channels.arm` | `1` .. `16` | `5` | RC channel index mapped to Arm Switch. |
| `channels.mode` | `1` .. `16` | `6` | RC channel index mapped to Flight Mode Switch (Acro vs Angle). |

> For receiver wiring diagrams and EdgeTX telemetry setup, see the [Receiver & Telemetry Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/RECEIVERS.md).

---

### Safety Interlocks & Arming Thresholds
```yaml
safety:
  arm_switch_threshold_us: 1800
  throttle_zero_threshold_us: 1050
```

| Parameter | Type / Range | Default | Description |
| :--- | :--- | :--- | :--- |
| `arm_switch_threshold_us` | `1200` .. `1900` &mu;s | `1800` &mu;s | Microsecond pulse width above which the aircraft arms. |
| `throttle_zero_threshold_us` | `1000` .. `1200` &mu;s | `1050` &mu;s | Maximum allowable throttle stick pulse permitted to arm. |

> [!CAUTION]
> ### ⚠️ Critical Safety Warning: Motor Spin-Up Hazard & Personal Injury
> - **Accidental Arming Hazard (`arm_switch_threshold_us`):** Setting this threshold too low (&lt;1400 &mu;s) introduces extreme risk of unintended arming on 2-position or 3-position toggle switches. Ensure this matches your transmitter's calibrated channel pulse (typically &approx;1000 &mu;s disarmed, &approx;2000 &mu;s armed).
> - **Violent Propeller Spin-Up Hazard (`throttle_zero_threshold_us`):** NEVER set this threshold higher than your transmitter's zero-throttle stick position. Setting `throttle_zero_threshold_us` above 1080 &mu;s may allow the flight controller to arm with the throttle raised or motor already spooling up! High-throttle arming causes immediate, violent propeller spin-up, resulting in severe lacerations or runaway aircraft. Keep this setting strictly at 1050 &mu;s unless transmitter trim calibration dictates minor adjustments.

---

### Servo Actuator Output & Calibration
```yaml
servos:
  pwm_frequency_hz: 50
  servo1:
    min_us: 1000
    max_us: 2000
    subtrim_us: 0
    reversed: false
  # ... servo2 through servo8
```

| Parameter | Type / Range | Default | Description |
| :--- | :--- | :--- | :--- |
| `pwm_frequency_hz` | `50` .. `400` Hz | `50` Hz | Servo PWM frame rate. **Standard analog servos MUST use 50 Hz** (max 60 Hz). Digital servos can accept 200..400 Hz. |
| `min_us` | `750` .. `1500` &mu;s | `1000` &mu;s | Minimum travel endpoint pulse width. |
| `max_us` | `1500` .. `2250` &mu;s | `2000` &mu;s | Maximum travel endpoint pulse width. |
| `subtrim_us` | `-500` .. `500` &mu;s | `0` &mu;s | Mechanical sub-trim offset added to calibrated center (1500 &mu;s). |
| `reversed` | `true`, `false` | `false` | Inverts servo deflection relative to flight loop demands. |

---

### In-Flight Dynamic PID Tuning
```yaml
in_flight_tuning:
  enabled: false
  slot_a:
    parameter: none     # Options: pitch_kp, roll_kp, pitch_ki, roll_ki, pitch_kd, roll_kd, pitch_kff, roll_kff
    channel: 7          # RC Channel number (1-16)
    min_val: 0.05
    max_val: 0.50
  slot_b:
    parameter: none
    channel: 8
    min_val: 0.005
    max_val: 0.050
```

| Parameter | Options / Range | Description |
| :--- | :--- | :--- |
| `enabled` | `true`, `false` | Enables live gain updates from transmitter potentiometers/sliders. |
| `slot_a.parameter`<br>`slot_b.parameter` | `none`, `pitch_kp`, `roll_kp`, `pitch_ki`, `roll_ki`, `pitch_kd`, `roll_kd`, `pitch_kff`, `roll_kff` | Specific PID/FeedForward gain modified by the corresponding channel. |
| `slot_a.channel`<br>`slot_b.channel` | `1` .. `16` | RC Channel number allocated to the adjustment pot. |
| `min_val` .. `max_val` | Floating point values | Gain range mapped linearly across the 1000..2000 &mu;s stick/pot sweep. |

---

### Motor Protocol & Navigation Suite

| Subsystem | Configuration | Reference Specification |
| :--- | :--- | :--- |
| **Motor Output** | `pwm` (50..400 Hz) or `dshot` (DShot300/600) | [System Architecture Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/ARCHITECTURE.md) |
| **Autonomous GPS** | NMEA parser, RTH guidance, 120m statutory ceiling | [GPS Navigation Specification](file:///home/bryansouza/Repos/wingfc-rs/docs/GPS_NAVIGATION.md) |

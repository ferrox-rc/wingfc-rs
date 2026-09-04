# Aircraft Configuration Guide (`wingfc.yaml`)

`wingfc-rs` features a human-friendly YAML configuration system modeled after leading flight control ecosystems (such as Betaflight, INAV, and EdgeTX/ExpressLRS). 

Pilots and builders can configure aircraft hardware, receiver protocols, motor drivers, servo refresh frequencies, PID gains, rates, and navigation features directly in [`wingfc.yaml`](file:///home/bryansouza/Repos/wingfc-rs/wingfc.yaml) without ever touching Rust source files, Cargo manifests, or linker scripts.

---

## 1. Quick Start Workflow

### Build Release Firmware
To configure and compile firmware for your aircraft:
```bash
./scripts/build.sh
```
This single command automatically:
1. Validates all parameters in [`wingfc.yaml`](file:///home/bryansouza/Repos/wingfc-rs/wingfc.yaml) (range checks, enum validation, hardware refresh limits, safety bounds, and airspace legality checks).
2. Generates the embedded flight controller configuration.
3. Automatically maps options to the exact Cargo build features (`crsf` vs `ibus`, `dshot` vs `pwm`, `nav`).
4. Compiles an optimized release binary for the ARM Cortex-M4F target (`thumbv7em-none-eabihf`).
5. Generates `.elf`, `.bin`, and `.hex` artifacts in `dist/` with cryptographic SHA256 checksums and Flash/RAM resource metrics.

### Check Configuration Syntax
To verify your settings and test compilation in under a second without generating full release artifacts:
```bash
./scripts/build.sh check
```

### Run Host Test Suite
To execute the core unit test suite on your local development machine:
```bash
./scripts/build.sh test
```

### Reset to Default Configuration
To generate or reset a fresh template of `wingfc.yaml` (existing file is automatically backed up to `wingfc.yaml.bak`):
```bash
./scripts/build.sh init
```

### Flash Firmware
To flash the compiled firmware directly to the Seeed Studio XIAO nRF52840 via a debug probe (CMSIS-DAP, J-Link, ST-Link):
```bash
./scripts/build.sh flash
```

---

## 2. Configuration Reference (`wingfc.yaml`)

### Target Board
```yaml
board: xiao-sense
```
* `xiao-sense`: Standard Seeed Studio XIAO nRF52840 Sense (6-DoF LSM6DS3TR IMU onboard).
* `xiao-sense-plus`: Seeed Studio XIAO nRF52840 Sense Plus (required for autonomous GPS navigation due to secondary UART breakout).

### Receiver Protocol
```yaml
receiver:
  protocol: crsf
  failsafe_timeout_ms: 500
```
* `protocol`:
  * `crsf`: TBS Crossfire or ExpressLRS (CRSF @ 420,000 baud over DMA UART). **Recommended.**
  * `ibus`: FlySky iBus protocol (115,200 baud).
* `failsafe_timeout_ms`: Maximum allowed duration without valid RC packets before failsafe engages (50 to 5000 ms). When triggered, motor throttle is cut to 0 and elevons neutralize to level flight.

### Servo Actuator Output Configuration
```yaml
servos:
  pwm_frequency_hz: 50
```
* `pwm_frequency_hz`: PWM frame refresh frequency (Hz) for servo channels (`D0`, `D1`, `D2`). Supported range: 50 Hz to 400 Hz.

> [!CAUTION]
> ### ⚠️ Hardware Warning: Analog Servo Burnout vs Digital Servos
> - **Standard Analog Servos (e.g. SG90, MG90S, TowerPro, plastic 9g servos):** **MUST be set to 50 Hz (maximum 60 Hz).** Supplying high-frequency PWM (>60 Hz) to an analog servo will cause violent internal feedback chattering, extreme motor heating, and **permanent burnout within minutes or seconds**.
> - **Digital Servos (e.g. KST, MKS, Savox, Blue Bird digital servos):** High-end digital servos natively support refresh rates of 200 Hz, 250 Hz, 333 Hz, or up to 400 Hz. Running digital servos at 250–400 Hz dramatically reduces control surface lag and improves gust rejection.
> - **Hardware Frequency Limit:** 400 Hz is the hard physical maximum for standard 1000..2000 &mu;s pulse widths. At 400 Hz, the total frame period is 2500 &mu;s, leaving a 500 &mu;s low gap after a 2000 &mu;s pulse. Frequencies above 400 Hz do not allow sufficient low-time for servos to detect individual pulse edges.

### Motor ESC Protocol & PWM Frequency
```yaml
motor:
  protocol: pwm
  pwm_frequency_hz: 400
  min_throttle_us: 1000
  max_throttle_us: 2000
```
* `protocol`:
  * `pwm`: Analog PWM output with configurable frame frequency. Includes automatic throttle calibration pass-through detection at boot.
  * `dshot`: Digital DShot protocol with CRC error checking. Eliminates analog calibration requirements.
* `pwm_frequency_hz`: Frame frequency for analog ESC output (50 Hz to 400 Hz).
* `min_throttle_us` / `max_throttle_us`: Physical pulse widths in microseconds for analog PWM.

> [!WARNING]
> ### ⚠️ Hardware Warning: Fixed-Wing ESC Compatibility
> - **Multirotor ESCs (BLHeli_S, BLHeli_32, AM32):** Natively support 400 Hz high-speed PWM input (or DShot).
> - **Airplane & Fixed-Wing ESCs:** Many traditional airplane ESCs (especially yellow-wrap budget ESCs or linear BEC airplane ESCs) **only support 50 Hz or 60 Hz**. If an airplane ESC is driven with 400 Hz, it will often refuse to arm, beep error codes continuously, or jitter erratically. If you experience arming issues with an analog ESC, reduce `motor.pwm_frequency_hz` to `50` or `100`.

### Autonomous Navigation Suite & Altitude Boundaries
```yaml
navigation:
  enabled: false
  barometer: false
  compass: false
  gps: false
  rth:
    min_satellites: 6
    max_hdop: 2.5
    safe_altitude_agl_m: 30.0
  hard_deck_agl_m: 15.0
  max_ceiling_agl_m: 120.0
```
* `enabled`: Master switch for the autonomous navigation suite (enables barometer, compass, and GPS).
* `barometer`: Enables the 500Hz Vertical Kalman Filter fusing vertical accelerometer data with barometric altitude.
* `compass`: Enables 9-DoF Madgwick AHRS attitude fusion for drift-free magnetic heading lock.
* `gps`: Enables NMEA GPS parsing and autonomous Return-to-Home guidance. **Requires `board: xiao-sense-plus`** to access `UARTE1` serial pins (`D9`/`D10`).
* `rth`:
  * `min_satellites`: Minimum satellites tracked before the flight controller latches the home waypoint (default 6).
  * `max_hdop`: Maximum horizontal dilution of precision required to validate home latching (default 2.5).
  * `safe_altitude_agl_m`: Safe climb altitude Above Ground Level during autonomous return (default 30.0 m).
* `hard_deck_agl_m`: Emergency altitude floor (meters Above Ground Level). If breached with a negative sink rate, the flight controller overrides pilot pitch and applies climb throttle.
* `max_ceiling_agl_m`: Statutory regulatory ceiling limiter (meters Above Ground Level).

> [!WARNING]
> ### ⚠️ Civil Aviation Airspace Legality & Regulatory Altitude Ceiling Warning
> Under national civil aviation regulations worldwide:
> - **United States (FAA):** Under 14 CFR &sect; 107.51(b) (commercial operations) and 49 U.S.C. &sect; 44809 (recreational flyers), small unmanned aircraft systems (sUAS) may **not** operate higher than **400 feet (121.9 meters) Above Ground Level (AGL)** without an active airspace authorization or approved Part 107.200 waiver.
> - **European Union (EASA):** Under Commission Implementing Regulation (EU) 2019/947 UAS.OPEN.010, the maximum flight height in the Open category is strictly **120.0 meters (400 feet) AGL**.
> - **United Kingdom (CAA):** Under CAP 722 and the Air Navigation Order, flight height is strictly capped at **120 meters (400 feet) AGL**.
> - **Australia (CASA) & Canada (Transport Canada):** Maximum permissible ceiling without waiver is **400 feet (120 meters) AGL**.
>
> Setting `max_ceiling_agl_m` above 120.0 meters (or attempting to fly above 120m AGL without prior civil aviation authority waiver/clearance) is **ILLEGAL**, poses severe mid-air collision hazards with manned aircraft, and exposes the remote pilot to civil fines and criminal prosecution. The build tool (`./scripts/build.sh`) will flag any configuration exceeding 120.0m with an alert banner.

### Flight Dynamics & Rates Tuning
```yaml
tuning:
  acro_rates:
    roll_dps: 600.0
    pitch_dps: 200.0
    yaw_dps: 100.0
  angle_limits:
    max_roll_deg: 60.0
    max_pitch_deg: 45.0
  filters:
    gyro_lpf_hz: 90.0
    dterm_lpf_hz: 40.0
    feedforward_lpf_hz: 45.0
  rate_pid:
    kp: 0.15
    ki: 0.05
    kd: 0.008
    kff: 0.04
    max_integral: 0.40
  ahrs_beta: 0.04
```
* `acro_rates`: Maximum angular rates commanded at full stick deflection in Acro / Rates mode (&deg;/s).
* `angle_limits`: Maximum bank and pitch angles commanded in Angle / Self-Leveling mode (&deg;).
* `filters`:
  * `gyro_lpf_hz`: PT1 low-pass filter cutoff frequency (Hz) for eliminating motor and propeller harmonic vibration before attitude estimation and PID loops.
  * `dterm_lpf_hz`: Cutoff frequency for PID derivative filter.
  * `feedforward_lpf_hz`: Cutoff frequency for FeedForward stick derivative filter.
* `rate_pid`:
  * `kp`: Proportional gain.
  * `ki`: Integral gain.
  * `kd`: Derivative gain.
  * `kff`: FeedForward gain (injects stick derivative directly into actuator mixer to eliminate stick lag).
  * `max_integral`: Anti-windup clamping threshold.
* `ahrs_beta`: Convergence gain for the Madgwick attitude estimation filter.

### Safety & Launch Interlocks
```yaml
safety:
  max_arm_throttle_us: 1050
  finger_delay_ms: 200
  throttle_ramp_ms: 400
```
* `max_arm_throttle_us`: Interlock threshold. Arming is refused if throttle is above this value when flipping the arm switch.
* `finger_delay_ms`: Autolaunch finger safety delay before motor starts spinning after throw detection.
* `throttle_ramp_ms`: Soft-start throttle acceleration duration during autolaunch to prevent aerodynamic torque roll.

---

## 3. Multiple Model Profiles

You can store distinct configuration files for different flying wings or gliders and pass them to the build script using `--config`:
```bash
# Build for a 600mm speed wing with 333Hz digital servos and DShot
./scripts/build.sh --config configs/speed_wing.yaml

# Build for a budget trainer with 50Hz analog servos and 50Hz ESC
./scripts/build.sh --config configs/budget_trainer.yaml
```

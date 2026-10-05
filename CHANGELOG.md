# Changelog

All notable changes to the WingFC-rs flight controller project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### 🚀 Features

- **Multi-Rate Loop Profile (`loop_rate: multirate`):** LSM6DS3TR-C sampled at 1.66 kHz and boxcar-averaged 8:1 into a fixed 208.33 Hz (`dt` = 4.8 ms) flight loop with sinc decimation anti-aliasing.
- **8:1 Boxcar Downsampler:** Host-tested `wingfc_core::downsample::BoxcarAccumulator8` with zero heap allocation.
- **Sensor Health State Machine:** Tri-state `SensorHealth` tracking (`Healthy`, `Degraded`, `Failed`) with timeout-based fault detection, consecutive sample recovery, and automatic safe mode fallback (`HeadingHold`/`AltHold` to `Angle`).
- **Granular Build Profiles (`ultralight` & `gps`):** High-level profiles (`ultralight` as sub-250g default, `gps` for autonomous navigation) decoupling granular feature flags (`baro`, `compass`, `gps`) while standardizing blackbox logging.
- **Tilt-Compensated Compass:** `wingfc_core::compass` with hard-iron/soft-iron calibration (`MagCalibration`, `MagCalibrator`) and tilt-compensated heading estimation.
- **Inter-Task Mailboxes:** `IMU_WATCH` and lock-free `ACTUATOR_LATCH` scaffolding in the firmware `bus` module for future decoupled flight loop workers.
- **External Sensors:** DPS310/BMP280 barometer drivers and a `TWISPI1` external-sensor task feeding the Vertical Kalman Filter.
- **In-Flight PID Tuning Banks:** Multi-bank, switch-selected potentiometer tuning.

### 🔧 Changes

- External sensor task now polls at 20 Hz (previously 50 Hz) on the navigation bus.
- CRSF telemetry divider now scales dynamically to ~50 Hz across all loop rates.
- QSPI flash pinout verified and documented on `P0.20`..`P0.25`, preserving Servo 7 on `P0.19`.

## [0.3.0] - 2026-09-10

### 🚀 Features

- **Active Self-Leveling Stabilization:** Validated cascaded attitude P and angular rate PID dual-loop controller in `ArmedAngle` mode.
- **Flight Mode Activation Matrix:** INAV-style multi-switch range matrix configured via `wingfc.yaml` supporting seamless Acro, Angle, Heading Hold, Alt Hold, and RTH switching.
- **Cascaded Navigation Controllers:** Heading Hold (compass-assisted banking) and Altitude Hold (barometric vertical velocity control) controllers.
- **External Compass Drivers:** Selectable QMC5883L and HMC5883L I2C drivers operating on dedicated `TWISPI1` (`D4`/`D5`).
- **System Architecture Flowcharts:** Comprehensive Mermaid.js flowcharts documenting boot sequence, 500 Hz control loop, cascaded PID dataflow, and safety state machines in [`docs/FLOWCHARTS.md`](docs/FLOWCHARTS.md).

### 🐛 Bug Fixes & Stability

- **Non-Blocking IMU EasyDMA:** Eliminated blocking `int1.wait_for_high()` timeout, enabling continuous 1.66 kHz ODR burst reads in ~300&mu;s without dropping 500 Hz control loop frames.
- **Sensor Rail HighDrive Power:** Configured `P1.08` with `OutputDrive::HighDrive` to eliminate sensor rail brownouts under active I2C/IMU load.
- **Ground Pre-Flight Stick Passthrough:** Preserved 100% mechanical servo deflection on ground when disarmed while holding motors safe.
- **UF2 Flasher Resiliency:** Handled immediate MCU reboot gracefully during virtual USB filesystem write operations.
- **Hardware Watchdog Timeout:** Increased watchdog supervision window to 2000ms to accommodate initial sensor bus stabilization.

### 🚜 Refactor

- **Castellated Pinout Allocation:** Restricted all physical signals strictly to edge castellated pads, avoiding underside test pads.
- **Embassy PWM Optimization:** Cleaned `SimplePwm` duty cycle mapping and polarity inversion across all 8 airframe servo/ESC channels.
## [0.2.0] - 2026-09-09

### ⚙️ Miscellaneous Tasks

- Chore(release): Release v0.2.0
- Ci: Gate firmware matrix build and packaging to release tags and manual dispatch
- Ci: Pin toolchain 1.98.0 and install rustfmt and clippy components in CI

### 💼 Other Changes

- Update pin mappings and upgrade UART receiver to buffered driver
- Update dependencies and embassy-nrf drivers; Improved `wingfc-firmware`
- Expand PWM driver support for multi-motor and multi-servo setups

### 📚 Documentation

- Docs(changelog): Update crate-level changelogs for v0.1.2
- Docs: Correct UART RX/TX crossover definitions and pin mapping representations

### 🚀 Features

- Add configurable angle P gain for flight loop
- Feat: Add 8-way board mounting orientation transform
- Feat: Add T-tail and V-tail airframe mixing profiles
- Feat: Add per-servo endpoints, sub-trim, and reversal support
- Feat: Add hardware watchdog timer supervision
- Feat: Add in-flight PID tuning via RC channels
- Feat: Add dual-LED visual telemetry with discrete User LED
- Feat: Add unified 8-servo sequential mapping, configurable RC channels, and updated documentation
- Feat: Make differential thrust authority configurable in wingfc.yaml and mixer
## [0.1.2] - 2026-09-06

### ⚙️ Miscellaneous Tasks

- Chore(release): Release v0.1.2

### 📚 Documentation

- Docs: Document comprehensive unit test coverage across README, ARCHITECTURE, CONFIG, and RELEASE_MATRIX

### 🚀 Features

- Feat: Add multi-method flash engine with interactive UF2 deployment, update pinouts and memory layout, and add deployment guide
## [0.1.1] - 2026-09-05

### ⚙️ Miscellaneous Tasks

- Bump version to v0.5.1 across wingfc-core and wingfc-firmware

### 🚀 Features

- Add GPS navigation subsystem, NMEA parser, RTH guidance, and documentation
- Add configurable arming and throttle safety thresholds, update documentation and release matrix
- Add GitHub Actions CI/CD pipeline and clean up clippy linter warnings
- Implement bidirectional CRSF telemetry protocol and asynchronous DMA broadcast task
- Add Receiver & Telemetry specification doc and update release matrix with measured binary sizes
- Added uf2conv to scripts;
- Feat: Integrate cargo-workspace inheritance, cargo-release automation, and git-cliff changelog
## [0.1.0] - 2026-09-04

### 💼 Other Changes

- Initial commit: High-performance Rust flight controller workspace with #![no_std] core, Madgwick AHRS, Acro Rate PID, and Embassy firmware
- Configure workspace cargo aliases and enable default host testing for wingfc-core

### 🚀 Features

- Add full FlySky iBus receiver protocol parser, 16-bit checksum verification, unified RcChannels abstraction, and unit tests
- Complete Phase 2: Dual multi-frequency PWM drivers, LSM6DS3TR I2C driver, EasyDMA UART receiver with CRSF/iBus, and gyro vibration filtering
- Implement build-time Cargo features for protocol selection (crsf vs ibus) with zero-cost dead-code elimination
- Complete Phase 3: Arming safety interlocks, RGB LED visual telemetry, zero-surge failsafe recovery, and safety unit tests
- Implement early ESC PWM startup (Step 0) and high-throttle ESC range calibration pass-through mode
- Implement and unit test FeedForward (FF) Rate PID, Hand Autolaunch, 9-DoF Madgwick AHRS, Vertical Kalman Filter, and Altitude Safety controllers
- Complete Phase 5: YAML aircraft configuration system, build automation, pinout architecture, PWM frequency tuning, and airspace legality warnings

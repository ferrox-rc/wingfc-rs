# Changelog

All notable changes to the WingFC-rs flight controller project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### 🚀 Features & Architecture

- **3-Crate Hardware Abstraction Layer (`wingfc-drivers`):**
  - Extracted MCU-agnostic external IC drivers (`LSM6DS3TR-C` IMU, `DPS310`/`BMP280` barometers, `QMC5883L`/`HMC5883L` compasses, `StatusLed` / `RgbLedDriver`) built on `embedded-hal` 1.0 and `embedded-hal-async` 1.0 into `crates/wingfc-drivers`.
  - Defined MCU-agnostic on-chip peripheral traits (`ServoOutput`, `MotorOutput`, `NorFlashStorage`) decoupling all flight loop, receiver, telemetry, external sensor, and blackbox tasks from specific microcontroller HALs.
  - Preserved `crates/wingfc-core` as 100% pure deterministic math, control theory, and byte-level protocol parsers (`#![no_std]`, zero hardware traits).
- **Seeed Studio XIAO STM32C5 Board Support Scaffolding (`board: xiao-stm32c5`):**
  - Added `boards/` module (`BoardTarget`, `xiao_nrf52840`, `xiao_stm32c5`, and `memory_stm32c5.x` linker script) targeting Cortex-M33F @ 144 MHz (`thumbv8m.main-none-eabihf`) with 16 MB XSPI NOR flash geometry.
  - Added `Lsm6ds3Driver::read_temperature_c()` (`0x20..0x21`) and `ImuHeaterController` PI thermal loop (`PA8` `TIM1_CH1` @ 45°C setpoint) for zero-thermal-drift gyro regulation.
  - Updated `scripts/builder.py` and `build.rs` to automatically select target triples and linker scripts from `wingfc.yaml`.

## [0.5.0] - 2026-10-26

### 🚀 Features

- **3-Tier Multi-Rate Decoupled Architecture (`loop_rate: multirate`):**
  - **Tier 1 (1.666 kHz IMU Worker):** Dedicated `imu_worker_task` on `TWISPI0` with `INT1` DRDY pacing and 8:1 boxcar decimation (`wingfc_core::downsample::BoxcarAccumulator8`) publishing to `IMU_WATCH`.
  - **Tier 2 (208.33 Hz Flight Loop):** Event-driven flight loop (`dt` = 4.8 ms) running VQF AHRS, PID, and airframe mixing, updating autonomous hardware `PWM0/1/2` EasyDMA registers directly.
  - **Tier 3 (50 Hz Slow Sensors, Telemetry & QSPI Blackbox):** Background navigation sensors (`baro`/`compass` on `TWISPI1`), CRSF telemetry, and 50 Hz QSPI NOR flash blackbox logging.
- **Native QSPI Blackbox Logger & Dual USB/SWD Offload:** 32-byte `BlackboxFrame` (`wingfc_core::blackbox`) with CRC-8, 256-byte EasyDMA page buffering to Puya P25Q16H 2 MB NOR flash, automatic on-demand 4 KB sector erase, and `scripts/blackbox.py` companion tool supporting both XIAO DebugMate SWD (`BBL_SWD_MAILBOX` via `probe-rs`) and native USB CDC-ACM (`BBL_INFO`, `BBL_DUMP`, `BBL_ERASE`).
- **VQF AHRS Attitude Estimator:** Replaced Madgwick with `VqfAHRS` (`tau_acc`, `tau_mag`) for decoupled roll/pitch vs. heading estimation.
- **Sensor Health State Machine:** Tri-state `SensorHealth` tracking (`Healthy`, `Degraded`, `Failed`) with timeout-based fault detection, consecutive sample recovery, and automatic safe mode fallback (`HeadingHold`/`AltHold` to `Angle`).
- **Granular Build Profiles (`ultralight` & `gps`):** High-level profiles (`ultralight` as sub-250g default, `gps` for autonomous navigation) decoupling granular feature flags (`baro`, `compass`, `gps`).
- **Tilt-Compensated Compass:** `wingfc_core::compass` with hard-iron/soft-iron calibration (`MagCalibration`, `MagCalibrator`) and tilt-compensated heading estimation.
- **External Sensors:** DPS310/BMP280 barometer drivers and a 50 Hz `TWISPI1` external-sensor task feeding the Vertical Kalman Filter.
- **In-Flight PID Tuning Banks:** Multi-bank, switch-selected potentiometer tuning.

### 🔧 Changes & Fixes

- Auto-regenerate `crates/wingfc-firmware/src/config.rs` from `wingfc.yaml` inside `build.rs` during `cargo build` / `cargo run`.
- Distinguish bench disarmed ground state (`ArmingStatus::Disarmed`) from in-flight `ArmingStatus::Failsafe` when RC transmitter is off before arming.
- Enable 64 MHz external crystal (`HfclkSource::ExternalXtal`) and set `DEFMT_LOG = "info"` by default in `.cargo/config.toml`.
- Documented and verified QSPI flash pinout on `P0.20`..`P0.25`, preserving Servo 7 on `P0.19`.

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

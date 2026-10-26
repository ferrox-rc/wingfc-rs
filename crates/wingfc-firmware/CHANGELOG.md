# Changelog

All notable changes to the WingFC-rs flight controller project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.5.0] - 2026-10-26

### 🚀 Features

- **3-Tier Multi-Rate Decoupled Architecture:**
  - Tier 1: `imu_worker_task` sampling LSM6DS3TR-C at 1.666 kHz on `INT1` DRDY with 8:1 boxcar decimation into `IMU_WATCH`.
  - Tier 2: 208.33 Hz (`dt` = 4.8 ms) flight control loop updating autonomous hardware `PWM0/1/2` EasyDMA registers directly.
  - Tier 3: 50 Hz external sensors (`TWISPI1`), telemetry, and QSPI NOR flash blackbox logging.
- **QSPI Blackbox & Dual Offload (`tasks::blackbox`, `drivers::qspi`):** 32 MHz Quad-IO Puya P25Q16H driver, 50 Hz page-buffered flight logging, on-demand sector erase, and disarmed log offload over both XIAO DebugMate SWD (`BBL_SWD_MAILBOX`) and native USB CDC-ACM (`usb_rx_task`).
- **Build Profiles (`ultralight` & `gps`) & Auto-Config (`build.rs`):** Automatic regeneration of `src/config.rs` from `wingfc.yaml` during `cargo build` / `cargo run`.

### 🔧 Changes

- Enabled 64 MHz external crystal (`HfclkSource::ExternalXtal`) and `DEFMT_LOG = "info"`.
- Documented and verified QSPI flash pinout (`P0.20`..`P0.25`), preserving Servo 7 on `P0.19`.
- External sensor bus and blackbox tasks unified at 50 Hz (20 ms period).

## [0.4.0] - 2026-09-25

### 💼 Other Changes

- Merge pull request #1 from BryanSouza91/dev

### 🚀 Features

- Feat(flight-loop): Implement IMU INT1 DRDY event pacing, dynamic DT, and selectable loop rates
## [0.3.1] - 2026-09-11

### ⚙️ Miscellaneous Tasks

- Chore(release): Release v0.3.1

### ⚡ Performance

- Perf(optimization): Eliminate divisions, cache Euler angles, and optimize mixer

### 💼 Other Changes

- Tune(rates): Restore standard angle_p_gain to 4.0 in wingfc.yaml
## [0.3.0] - 2026-09-10

### ⚙️ Miscellaneous Tasks

- Chore(release): Release v0.3.0

### 🎨 Styling

- Style: Apply rustfmt formatting and add auto-rustfmt to builder.py

### 🐛 Bug Fixes

- Fix(stabilization): Restore active attitude stabilization, non-blocking IMU reads, and complete system flowcharts

### 💼 Other Changes

- Clean up imports and formatting in flight loop task

### 🚀 Features

- Feat(compass): Add selectable external QMC5883L and HMC5883L I2C drivers on TWISPI1
- Feat(flight-modes): Add INAV-style mode matrix, cascaded Heading Hold, and Altitude Hold controllers

### 🚜 Refactor

- Refactor(firmware): Restrict board pin mappings strictly to physical castellated edge pads and allocate TWISPI1 for external I2C
## [0.2.0] - 2026-09-09

### ⚙️ Miscellaneous Tasks

- Chore(release): Release v0.2.0

### 💼 Other Changes

- Update pin mappings and upgrade UART receiver to buffered driver
- Update dependencies and embassy-nrf drivers; Improved `wingfc-firmware`
- Expand PWM driver support for multi-motor and multi-servo setups

### 📚 Documentation

- Docs(changelog): Update crate-level changelogs for v0.1.2
- Docs: Correct UART RX/TX crossover definitions and pin mapping representations

### 🚀 Features

- Feat: Add multi-method flash engine with interactive UF2 deployment, update pinouts and memory layout, and add deployment guide
- Add configurable angle P gain for flight loop
- Feat: Add 8-way board mounting orientation transform
- Feat: Add T-tail and V-tail airframe mixing profiles
- Feat: Add per-servo endpoints, sub-trim, and reversal support
- Feat: Add hardware watchdog timer supervision
- Feat: Add in-flight PID tuning via RC channels
- Feat: Add dual-LED visual telemetry with discrete User LED
- Feat: Add unified 8-servo sequential mapping, configurable RC channels, and updated documentation
- Feat: Make differential thrust authority configurable in wingfc.yaml and mixer
## [0.1.1] - 2026-09-05

### ⚙️ Miscellaneous Tasks

- Bump version to v0.5.1 across wingfc-core and wingfc-firmware

### 🚀 Features

- Add GPS navigation subsystem, NMEA parser, RTH guidance, and documentation
- Add configurable arming and throttle safety thresholds, update documentation and release matrix
- Implement bidirectional CRSF telemetry protocol and asynchronous DMA broadcast task
- Feat: Integrate cargo-workspace inheritance, cargo-release automation, and git-cliff changelog
## [0.1.0] - 2026-09-04

### 💼 Other Changes

- Initial commit: High-performance Rust flight controller workspace with #![no_std] core, Madgwick AHRS, Acro Rate PID, and Embassy firmware

### 🚀 Features

- Complete Phase 2: Dual multi-frequency PWM drivers, LSM6DS3TR I2C driver, EasyDMA UART receiver with CRSF/iBus, and gyro vibration filtering
- Implement build-time Cargo features for protocol selection (crsf vs ibus) with zero-cost dead-code elimination
- Complete Phase 3: Arming safety interlocks, RGB LED visual telemetry, zero-surge failsafe recovery, and safety unit tests
- Implement early ESC PWM startup (Step 0) and high-throttle ESC range calibration pass-through mode
- Implement and unit test FeedForward (FF) Rate PID, Hand Autolaunch, 9-DoF Madgwick AHRS, Vertical Kalman Filter, and Altitude Safety controllers
- Complete Phase 5: YAML aircraft configuration system, build automation, pinout architecture, PWM frequency tuning, and airspace legality warnings

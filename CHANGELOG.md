# Changelog

All notable changes to the WingFC-rs flight controller project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.1] - 2026-09-05

### ⚙️ Miscellaneous Tasks

- Transition to cargo-workspace inheritance at v0.1.1 across all crates

### 🚀 Features

- Add GPS navigation subsystem, NMEA parser, RTH guidance, and documentation
- Add configurable arming and throttle safety thresholds, update documentation and release matrix
- Add GitHub Actions CI/CD pipeline and clean up clippy linter warnings
- Implement bidirectional CRSF telemetry protocol and asynchronous DMA broadcast task
- Add Receiver & Telemetry specification doc and update release matrix with measured binary sizes
- Added uf2conv to scripts;
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

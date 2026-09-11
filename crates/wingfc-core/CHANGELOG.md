# Changelog

All notable changes to the WingFC-rs flight controller project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.1] - 2026-09-11

### ⚡ Performance

- Perf(optimization): Eliminate divisions, cache Euler angles, and optimize mixer
## [0.3.0] - 2026-09-10

### ⚙️ Miscellaneous Tasks

- Chore(release): Release v0.3.0

### 🎨 Styling

- Style: Apply rustfmt formatting and add auto-rustfmt to builder.py

### 🐛 Bug Fixes

- Fix(stabilization): Restore active attitude stabilization, non-blocking IMU reads, and complete system flowcharts

### 🚀 Features

- Feat(compass): Add selectable external QMC5883L and HMC5883L I2C drivers on TWISPI1
- Feat(flight-modes): Add INAV-style mode matrix, cascaded Heading Hold, and Altitude Hold controllers

### 🚜 Refactor

- Refactor(firmware): Restrict board pin mappings strictly to physical castellated edge pads and allocate TWISPI1 for external I2C
## [0.2.0] - 2026-09-09

### ⚙️ Miscellaneous Tasks

- Chore(release): Release v0.2.0

### 💼 Other Changes

- Update dependencies and embassy-nrf drivers; Improved `wingfc-firmware`

### 📚 Documentation

- Docs: Document comprehensive unit test coverage across README, ARCHITECTURE, CONFIG, and RELEASE_MATRIX
- Docs(changelog): Update crate-level changelogs for v0.1.2

### 🚀 Features

- Feat: Add 8-way board mounting orientation transform
- Feat: Add T-tail and V-tail airframe mixing profiles
- Feat: Add in-flight PID tuning via RC channels
- Feat: Add unified 8-servo sequential mapping, configurable RC channels, and updated documentation
- Feat: Make differential thrust authority configurable in wingfc.yaml and mixer
## [0.1.1] - 2026-09-05

### ⚙️ Miscellaneous Tasks

- Bump version to v0.5.1 across wingfc-core and wingfc-firmware

### 🚀 Features

- Add GPS navigation subsystem, NMEA parser, RTH guidance, and documentation
- Add configurable arming and throttle safety thresholds, update documentation and release matrix
- Add GitHub Actions CI/CD pipeline and clean up clippy linter warnings
- Implement bidirectional CRSF telemetry protocol and asynchronous DMA broadcast task
- Feat: Integrate cargo-workspace inheritance, cargo-release automation, and git-cliff changelog
## [0.1.0] - 2026-09-04

### 💼 Other Changes

- Initial commit: High-performance Rust flight controller workspace with #![no_std] core, Madgwick AHRS, Acro Rate PID, and Embassy firmware
- Configure workspace cargo aliases and enable default host testing for wingfc-core

### 🚀 Features

- Add full FlySky iBus receiver protocol parser, 16-bit checksum verification, unified RcChannels abstraction, and unit tests
- Complete Phase 3: Arming safety interlocks, RGB LED visual telemetry, zero-surge failsafe recovery, and safety unit tests
- Implement and unit test FeedForward (FF) Rate PID, Hand Autolaunch, 9-DoF Madgwick AHRS, Vertical Kalman Filter, and Altitude Safety controllers
- Complete Phase 5: YAML aircraft configuration system, build automation, pinout architecture, PWM frequency tuning, and airspace legality warnings

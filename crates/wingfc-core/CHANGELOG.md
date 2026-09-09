# Changelog

All notable changes to the WingFC-rs flight controller project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2026-09-08

### 🚀 Features
- Complete multi-airframe mixing suite (Elevon, Diff Thrust, T-Tail, V-Tail)
- Expand to unified 8-servo sequential architecture (`[ServoConfig; 8]`, `servo1_us`..`servo8_us`)
- Add per-servo limits, sub-trim offset, and direction reversal support
- Add 8-way board mounting orientation transform matrix
- Add in-flight dynamic PID potentiometer tuning via RC channels
- Add configurable outer angle P-gain for attitude control
- Add configurable RC channel mapping (1..16)

## [0.1.2] - 2026-09-06

### 📚 Documentation

- Docs: Document comprehensive unit test coverage across README, ARCHITECTURE, CONFIG, and RELEASE_MATRIX

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

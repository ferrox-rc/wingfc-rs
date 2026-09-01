# WingFC-rs

High-performance, zero-latency, bare-metal Rust flight controller firmware for sub-250g FPV flying wings, acrobatic wings, and UAVs.

## Key Features
- **Deterministic Hard Real-Time Execution:** Zero garbage collector, zero heap allocation, sub-microsecond jitter.
- **Acrobatic & Rates Mode:** Direct angular velocity tracking with PT1 & Biquad vibration filtering.
- **Madgwick Quaternion AHRS:** Full 3D attitude estimation without gimbal lock and with dynamic G-rejection.
- **DMA-Driven I/O:** Embassy-based async UART (CRSF / ELRS), SPI (DShot), and hardware PWM.
- **Next-Gen Telemetry:** Zero-overhead binary logging over RTT with `defmt`.

## Workspace Structure
- `crates/wingfc-core`: Pure `#![no_std]` algorithms (AHRS, PID, Airframe Mixers, Protocols) testable directly on host PC with `cargo test`.
- `crates/wingfc-firmware`: Target firmware for Seeed Studio XIAO nRF52840 Sense (`thumbv7em-none-eabihf`).

## Building & Testing

### Run Core Unit Tests on Host:
```bash
cargo test -p wingfc-core
```

### Build Embedded Firmware for nRF52840:
```bash
cargo build --release -p wingfc-firmware
```

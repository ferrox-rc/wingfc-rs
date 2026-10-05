# WingFC-rs System Architecture & Design Specification

`wingfc-rs` is a bare-metal, high-performance, `#![no_std]` flight controller firmware and flight dynamics core written entirely in Rust for the ARM Cortex-M4F architecture (specifically targetting the Nordic nRF52840 SoC on the Seeed Studio XIAO nRF52840 Sense / Sense Plus, and custom derivative PCBs).

---

## 1. System Design Principles

1. **Deterministic Execution:** Zero heap allocations (`no_std`), zero garbage collection pauses, and hard real-time execution at 500Hz (2000&mu;s period).
2. **Hardware Watchdog Supervision:** 2000ms hardware watchdog timer (`WDT`) supervised directly by the 500 Hz flight loop.
3. **Zero-Cost Abstractions:** Build-time Cargo feature flags eliminate inactive protocols, algorithms, and drivers at compile time via dead-code elimination.
4. **Defense-in-Depth Safety:** Multi-layered arming interlocks, zero-surge failsafe recovery, and physical separation between core logic (`wingfc-core`) and peripheral drivers (`wingfc-firmware`).
5. **Modularity for Rapid Testing:** Pure flight control algorithms are isolated in `wingfc-core` and fully testable on host platforms via standard `cargo test` (achieving 88.4% line coverage and 89.8% region coverage across 21 unit tests).
6. **Zero-Code User Configuration:** End users configure aircraft hardware, protocols, flight dynamics, per-servo limits, and PID tunes using a friendly configuration file ([`wingfc.yaml`](../wingfc.yaml)) without touching Rust source code or build files (see the [Aircraft Configuration Guide](CONFIG.md)).
7. **Visual Architectural Flowcharts:** Complete Mermaid.js state and dataflow diagrams are available in the [System Architecture & Control Flowcharts](FLOWCHARTS.md).

---

## 2. Hardware Architecture & Peripheral Allocation

Physical pinout allocations, wiring isolation guidelines, electrical specs, and peripheral conflict matrices are detailed in the [Pin Configuration & Hardware Allocation Specification](PINOUT.md).

### Summary of Assigned On-Chip Blocks:
* **`PWM0` (ESCs 1 & 2):** Drives Motor 1 (`D3`/`P0.29`) and Motor 2 (`D8`/`P1.13`) for single-motor or differential-thrust twins. Supports 50..400 Hz analog PWM or digital DShot frames.
* **`PWM1` (Primary Servos 1..4):** Drives standard flight control surfaces (`D0`/`P0.02`, `D1`/`P0.03`, `D2`/`P0.28`, `D9`/`P1.14`).
* **`PWM2` (Secondary Servos 5..8):** Available on Seeed Studio XIAO nRF52840 Sense Plus (`D10`/`P1.15`, `D11`/`P0.15`, `D12`/`P0.19`, `D13`/`P1.01`) for auxiliary gear, pan/tilt, or extra control surfaces.
* **`TWISPI0` (Internal IMU):** Dedicated 400 kHz Fast-Mode EasyDMA I2C bus communicating with the onboard ST LSM6DS3TR-C 6-DoF sensor (`P0.07` SDA, `P0.27` SCL, `P1.08` Power Rail with `HighDrive`).
* **`TWISPI1` (External Sensors):** Dedicated 400 kHz I2C bus on castellated pads (`D4`/`P0.04` SDA, `D5`/`P0.05` SCL) for external QMC5883L/HMC5883L magnetometers and BMP280/SPL06 barometers.
* **`UARTE0` (RC Receiver & Telemetry):** Buffered EasyDMA UART engine servicing high-speed RC protocols (CRSF/ELRS @ 420 kBaud or FlySky iBus @ 115.2 kBaud) and bidirectional telemetry transmission (see the [Receiver & Telemetry Specification](RECEIVERS.md)).
* **`UARTE1` (Autonomous GPS):** Secondary EasyDMA serial engine on Sense Plus (`D14`/`P0.09` RX, `D15`/`P0.10` TX) running the zero-allocation NMEA navigation parser (see the [GPS Navigation Specification](GPS_NAVIGATION.md)).
* **Visual Telemetry:** Coordinated dual-LED system featuring the 3-color RGB LED (`P0.26`, `P0.30`, `P0.06`) and discrete User LED (`P0.17`).

---

### 3. Hardware Event-Driven Flight Loop (416 Hz / 833 Hz)

Execution runs on Embassy's cooperative async executor with **hardware DRDY interrupt pacing** via GPIOTE on `P0.11` (`pin_imu_int1`), eliminating beat-frequency clock aliasing and phase jitter between the MCU and sensor crystal domains:

```
                  +---------------------------------------------------+
                  |  LSM6DS3 INT1 Hardware Data-Ready Interrupt (WFE) |
                  |    (416 Hz Standard / 833 Hz High-Speed Profile)  |
                  +---------------------------------------------------+
                                            |
                                            v
                                   [Pet Hardware WDT]
                                            |
                                            v
                             [Measure Dynamic dt (Instant)]
                                            |
                                            v
                                  [Drain Latest RC Packet]
                                            |
                                            v
                                 [Evaluate Failsafe Timeout]
                                            |
                                            v
                              [Update Arming Safety State Machine]
                                            |
                                            v
                              [Apply In-Flight Pot PID Tuning]
                                            |
                                            v
                        [Burst-Read LSM6DS3TR 12B Gyro/Accel via I2C]
                               (Clears DRDY on Sensor)
                                            |
                                            v
                            [Apply 8-Way Board Mounting Matrix]
                                            |
                                            v
                            [PT1 Gyro Harmonic Vibration Filter]
                                            |
                                            v
                        [Madgwick AHRS Orientation Estimation (dt)]
                                            |
                                            v
                        [Inner/Outer PID Loop (Rate / Angle / FF) (dt)]
                                            |
                                            v
                                  [Multi-Airframe Mixer]
                                            |
                                            v
                        [Per-Servo Calibration (Min, Max, Trim, Rev)]
                                            |
                                            v
                        [Output Actuation: Servos (PWM1/2) & ESC (PWM0)]
                                            |
                                            v
                         [Transmit CRSF Telemetry Snapshot (~50 Hz)]
```

### Control Pipeline Steps:
1. **Hardware DRDY Pacing & WDT:** The loop suspends in low-power `WFE` sleep until the LSM6DS3 asserts its data-ready interrupt on `P0.11`. A safety timeout (`3.0 * NOMINAL_DT`) guards against sensor detachment, while the 2000ms hardware watchdog is petted every cycle (`wdt_handle.pet()`).
2. **Dynamic DT Calculation:** Time elapsed since the previous sample is measured using `Instant::now()` and clamped within `[0.5 * NOMINAL_DT, 2.0 * NOMINAL_DT]` for numerical stability, compensating for physical sensor oscillator drift.
3. **RC Input & Channel Mapping:** Non-blocking reception from `UARTE0`. Channels are mapped according to [`wingfc.yaml`](../wingfc.yaml) (`roll`, `pitch`, `throttle`, `yaw`, `arm`, `mode`) using single-cycle hoisted scale factors.
4. **Failsafe Monitoring:** If no valid RC packet is received within `failsafe_timeout_ms`, the aircraft enters `Failsafe` state (throttle cut to zero, surfaces trimmed for level glide).
5. **Arming Safety State Machine:** Enforces arm-switch thresholds, zero-throttle interlocks, and boot-time switch positions.
6. **In-Flight Dynamic Tuning:** Real-time adjustment of selected PID gains via designated transmitter sliders or pots.
7. **IMU Sampling & Transformation:** Accel and gyro vectors are burst-read over 400 kHz Fast-Mode EasyDMA I2C. Gyro bias is subtracted in the raw sensor frame before the 8-way mounting orientation matrix is applied.
8. **Attitude Estimation & Control Loop:** The Madgwick AHRS and rate PIDs integrate with the measured dynamic `dt`. Transcendentals (`atan2f`/`asinf`) are evaluated lazily only when attitude stabilization or telemetry transmission requires them.
9. **Airframe Mixing & Output Dispatch:** Mixer evaluates only active servo channels and precomputed neutrals. `PWM0` updates ESC pulses; `PWM1` and `PWM2` update servo pulses.

---

## 4. Multi-Rate Decoupled Architecture (`loop_rate: multirate`)

Fixed-wing airframes have 10–25 Hz control bandwidth, so a monolithic loop synchronized to a fast IMU ODR wastes CPU and lets motor vibration alias into the control math. The `multirate` profile separates the sensing, control, and navigation rates:

| Tier | Rate | Bus / Peripheral | Role |
| :--- | :--- | :--- | :--- |
| **1. IMU acquisition** | 1666.6 Hz | `TWISPI0` (dedicated) | LSM6DS3TR-C ODR; each DRDY burst-read is accumulated |
| **2. Flight loop** | 208.33 Hz (`dt` = 4.8 ms) | CPU | 8:1 boxcar average → AHRS → PID → mixer |
| **3. Actuation** | Servo/ESC PWM rate | `PWM0/1/2` | Hardware timers hold the latest pulse widths |
| **4. Slow navigation** | 20 Hz | `TWISPI1` (external) | Barometer + compass → Vertical Kalman Filter / heading |

**Why 8:1 at 1666 Hz → 208.33 Hz?**
* An 8-sample boxcar average is a sinc low-pass with nulls at multiples of 208.33 Hz, attenuating motor/prop harmonics before they reach the PID derivative term.
* Exactly 8 samples per step yields a rock-solid `dt` of 4.8 ms, avoiding the clock aliasing of non-power-of-two loop rates against the LSM6DS3 binary ODRs.
* 500 Hz (2000 µs) was rejected: it saturates servo/ESC PWM frames (a 2 ms pulse in a 2 ms period) and aliases against the sensor ODRs.

### Implementation
* [`wingfc_core::downsample::BoxcarAccumulator8`](../crates/wingfc-core/src/downsample.rs) performs the decimation (host-tested).
* The IMU driver ([`drivers/imu.rs`](../crates/wingfc-firmware/src/drivers/imu.rs)) waits for 8 DRDY interrupts per `wait_data_ready()` call and `read()` returns the averaged frame. Bias subtraction and the mounting matrix are applied per raw sample, before averaging.
* The external sensor task polls the barometer and compass at 20 Hz on `TWISPI1`, keeping the IMU bus free of contention.
* [`wingfc_core::compass`](../crates/wingfc-core/src/compass.rs) provides hard-iron/soft-iron calibration (`MagCalibration`, `MagCalibrator`) and tilt-compensated heading:

  ```
  Xh = mx*cos(theta) + my*sin(phi)*sin(theta) + mz*cos(phi)*sin(theta)
  Yh = my*cos(phi) - mz*sin(phi)
  heading = atan2(-Yh, Xh) + declination
  ```

* [`bus.rs`](../crates/wingfc-firmware/src/bus.rs) defines the `IMU_WATCH` and lock-free `ACTUATOR_LATCH` mailboxes reserved for the planned split of the IMU worker from the flight loop.

# Implementation Plan: Multi-Rate Decoupled Architecture (`wingfc-rs`)

Architectural migration of `wingfc-rs` on Seeed Studio XIAO nRF52840 (Cortex-M4F @ 64 MHz) from a monolithic IMU-synchronized loop to an Embassy multi-rate decoupled async architecture.

## User Review Required

> [!IMPORTANT]
> **Key Architecture Decisions from the Design Blueprint**:
> 1. **IMU Sampling & Decimation (1666.6 Hz $\to$ 208.33 Hz)**:
>    - Hardware ODR set to 1666.6 Hz on dedicated `TWIM0`.
>    - 8:1 sinc/boxcar averaging accumulator yields decimated samples at **208.33 Hz** ($dt = 4.8\text{ ms}$).
>    - Completely eliminates clock aliasing, motor vibration aliasing, and servo PWM saturation (leaves 2800 µs low time on 2000 µs PWM pulses).
> 2. **Decoupled Actuation (RAM Latching)**:
>    - Flight loop runs at 208.33 Hz and writes actuator duty cycles to a lock-free RAM structure.
>    - Actuator hardware timers output at native rates: 50 Hz for analog servos, 208 Hz for digital servos, or DShot/OneShot for ESCs.
> 3. **Slow Sensors & Navigation (20 Hz)**:
>    - Moved to dedicated `TWIM1` bus (Barometer + Compass) + `UARTE1` (GPS), completely isolating IMU from bus delays.
> 4. **Storage & Logging**:
>    - High-speed 2MB NOR flash (P25Q16H) via hardware QSPI for wear-leveled configuration and 50 Hz blackbox logging.
> 5. **Build Profiles**:
>    - `ultralight` (sub-250g micro wings: zero-alloc, IMU + RC + Servo mixing only).
>    - `full` (standard/autonomous: full navigation, QSPI blackbox, BLE telemetry, mission engine).

---

## Architecture Overview & Dataflow

```
+-----------------------------------------------------------------------------+
| TIER 1: FAST IMU WORKER (TWIM0, High Priority)                              |
| - LSM6DS3TR-C sampled at 1666.6 Hz (ODR_XL=1.66kHz, ODR_G=1.66kHz)          |
| - 8:1 Sinc/Boxcar Downsampling Accumulator                                  |
| - Produces filtered snapshot at 208.33 Hz (dt = 4.8 ms)                     |
+------------------------------------+----------------------------------------+
                                     |
                                     v [IMU_WATCH (208.33 Hz)]
+-----------------------------------------------------------------------------+
| TIER 2: DETERMINISTIC FLIGHT LOOP (208.33 Hz, Medium-High Priority)        |
| - Consumes IMU_WATCH sample (fixed dt = 0.0048 s)                           |
| - Samples RC_WATCH (latest packet, fail-safe guarded)                       |
| - Samples SLOW_SENSORS_WATCH (latest baro/mag/GPS, non-blocking)            |
| - AHRS: 6-DoF/9-DoF Madgwick filter update                                  |
| - Controllers: Angle / Rate / Altitude Hold / Heading Hold PID               |
| - Airframe Mixer: Elevon / V-Tail / Standard Wing                          |
| - Writes Actuator Target Pulses to ACTUATOR_LATCH in RAM                    |
+------------------+------------------------------------+---------------------+
                   |                                    |
                   v [ACTUATOR_LATCH]                   v [TELEMETRY_BUS]
+------------------------------------+   +------------------------------------+
| TIER 3: ACTUATION DRIVERS          |   | LOGGING & TELEMETRY                |
| - Analog Servos @ 50 Hz (PWM Timer)|   | - QSPI Blackbox Log @ 50 Hz        |
| - Digital Servos @ 208 Hz (PWM)    |   | - BLE Telemetry (NUS) @ 10 Hz      |
+------------------------------------+   +------------------------------------+

+-----------------------------------------------------------------------------+
| TIER 4: SLOW NAVIGATION SENSORS & RC INGESTION (Async Tasks)                |
| - RC Task (UARTE0 EasyDMA): Packet-driven (CRSF/iBUS) -> publishes RC_WATCH |
| - Nav Sensors Task (TWIM1 @ 20 Hz): DPS310/BMP280 Baro + QMC/HMC Compass    |
| - GPS Task (UARTE1 EasyDMA): NMEA / UBX parser @ 10 Hz                      |
| -> Publishes SLOW_SENSORS_WATCH                                             |
+-----------------------------------------------------------------------------+
```

---

## Phased Implementation Plan

### Phase 1: Core DSP Downsampler & Multi-Rate Telemetry Bus Types
**Goal:** Establish mathematically sound 8:1 downsampling and lock-free thread-safe inter-task communication primitives in `wingfc-core` and `wingfc-firmware`.

#### Files:
- [NEW] [`crates/wingfc-core/src/dsp/downsample.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-core/src/dsp/downsample.rs):
  - `BoxcarAccumulator8`: Accumulates 8 samples of 3D accel and 3D gyro.
  - Computes exact mean upon the 8th sample, scales by sensor sensitivity, resets accumulator.
  - Unit tests verifying integer/float precision, sinc filter attenuation, and exact $dt=0.0048$ emission.
- [NEW] [`crates/wingfc-firmware/src/bus/mod.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/bus/mod.rs) & [`crates/wingfc-firmware/src/bus/channels.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/bus/channels.rs):
  - `IMU_WATCH`: `Watch<CriticalSectionRawMutex, ImuDecimatedSample, 2>`
  - `RC_WATCH`: `Watch<CriticalSectionRawMutex, RcChannels, 2>`
  - `SLOW_SENSORS_WATCH`: `Watch<CriticalSectionRawMutex, SlowSensorsSnapshot, 2>`
  - `ACTUATOR_LATCH`: Lock-free `AtomicU16` arrays or double-buffered structure for servo duty cycles.

---

### Phase 2: Tier 1 IMU Worker Task (1666.6 Hz $\to$ 208.33 Hz)
**Goal:** Configure LSM6DS3TR-C at 1.66 kHz ODR over dedicated `TWIM0` EasyDMA and publish decimated frames to `IMU_WATCH`.

#### Files:
- [MODIFY] [`crates/wingfc-firmware/src/drivers/imu.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/drivers/imu.rs):
  - Configure LSM6DS3TR-C: `CTRL1_XL` (`0x80` for 1.66 kHz, $\pm 8g$), `CTRL2_G` (`0x80` for 1.66 kHz, $\pm 2000^\circ/\text{s}$).
  - Enable DRDY interrupt on `INT1` or burst DMA polling mode at 1666.6 Hz.
- [MODIFY] [`crates/wingfc-firmware/src/tasks/flight_loop.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/tasks/flight_loop.rs):
  - Separate raw IMU reader (`imu_worker_task`) from the flight loop.
  - Accumulate 8 raw reads in `BoxcarAccumulator8`, publish to `IMU_WATCH` every 4.8 ms.

---

### Phase 3: Tier 2 Deterministic Flight Loop & Actuation RAM Latching
**Goal:** Convert flight loop to consume `IMU_WATCH` at exactly 208.33 Hz, calculate PID mixer, and latch outputs into RAM decoupled from PWM output timers.

#### Files:
- [MODIFY] [`crates/wingfc-firmware/src/tasks/flight_loop.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/tasks/flight_loop.rs):
  - Main loop waits on `imu_receiver.changed().await`.
  - Non-blocking read of `rc_receiver.try_get()` with failsafe timeout tracking.
  - Non-blocking read of `slow_sensors_receiver.try_get()` for baro/mag navigation corrections.
  - Step inner rate/attitude/alt-hold controllers with exact $dt = 0.0048$.
  - Latch mixed servo/motor microsecond pulse targets to `ACTUATOR_LATCH`.
- [MODIFY] [`crates/wingfc-firmware/src/drivers/pwm.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/drivers/pwm.rs):
  - PWM timer peripheral task continuously refreshes compare registers from `ACTUATOR_LATCH` at independent rates:
    - 50 Hz (20 ms period) for analog servos.
    - 208 Hz (4.8 ms period) for digital servos.

---

### Phase 4: Tier 4 Slow Navigation Sensors & Tilt-Compensated Compass
**Goal:** Implement 20 Hz navigation task on `TWIM1` with tilt-compensated horizontal plane magnetic projection.

#### Files:
- [MODIFY] [`crates/wingfc-firmware/src/tasks/sensors_ext.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/tasks/sensors_ext.rs):
  - Rate-limit Barometer (DPS310/BMP280) and Compass (QMC5883L) polling to 20 Hz (50 ms).
  - Feed altitude and temperature into `SlowSensorsSnapshot`.
- [MODIFY] [`crates/wingfc-core/src/navigation/compass.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-core/src/navigation/compass.rs):
  - Implement tilt compensation using roll ($\phi$) and pitch ($\theta$) from attitude quaternion:
    $$X_h = X \cos\theta + Y \sin\phi \sin\theta + Z \cos\phi \sin\theta$$
    $$Y_h = Y \cos\phi - Z \sin\phi$$
    $$\text{Yaw}_{\text{mag}} = \text{atan2}(-Y_h, X_h) + \text{declination}$$
  - Hard-iron offset and soft-iron $3\times 3$ matrix calibration corrections.

---

### Phase 5: QSPI Flash Driver & Blackbox Logging Integration
**Goal:** Implement hardware QSPI driver for P25Q16H 2MB NOR flash on XIAO Sense and stream binary blackbox flight logs at 50 Hz.

#### Files:
- [NEW] [`crates/wingfc-firmware/src/drivers/qspi.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/drivers/qspi.rs):
  - Embassy nRF QSPI peripheral driver (`embassy_nrf::qspi::Qspi`).
  - Read JEDEC ID (`0x85 0x60 0x15` for P25Q16H).
  - Page program (256 bytes) and Sector erase (4 KB).
- [NEW] [`crates/wingfc-firmware/src/tasks/blackbox.rs`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/src/tasks/blackbox.rs):
  - Dedicated low-priority logging task receiving telemetry records via channel at 50 Hz.
  - Circular buffer writes with wear leveling for config parameters and sequential chunk logging.

---

### Phase 6: Build Profiles (`ultralight` vs `full`) & Matrix Verification
**Goal:** Support resource-constrained micro-wings vs fully autonomous long-range airframes via Cargo features.

#### Files:
- [MODIFY] [`crates/wingfc-firmware/Cargo.toml`](file:///home/bryansouza/Repos/wingfc-rs/crates/wingfc-firmware/Cargo.toml):
  - `profile-ultralight`: Disables QSPI, GPS, Compass, Baro. Leaves pure 208.33 Hz IMU + RC + Actuator mixing with minimal flash/RAM footprint.
  - `profile-full` (default): Enables all navigation sensors, QSPI logging, BLE telemetry, and failsafe RTL.
- [MODIFY] [`scripts/build.sh`](file:///home/bryansouza/Repos/wingfc-rs/scripts/build.sh):
  - Add matrix check for both `profile-ultralight` and `profile-full` across all target variants.

---

## Verification Plan

### Automated Tests
1. **Core DSP Tests (`wingfc-core`)**:
   ```bash
   cargo test -p wingfc-core -- dsp
   ```
   - Verify 8:1 decimation math, no overflow on int16 accumulated values, correct float scaling.
2. **Tilt-Compensated Compass Tests (`wingfc-core`)**:
   ```bash
   cargo test -p wingfc-core -- compass
   ```
   - Verify horizontal projection across pitch and roll angles $\pm 45^\circ$.
3. **Firmware Matrix Build**:
   ```bash
   ./scripts/build.sh matrix check
   ```
   - Verify all combinations (`release`, `debug`, `xiao-ble`, `xiao-sense`, `ultralight`, `full`) compile cleanly with zero warnings.

### Manual / Hardware Verification
- Run logic analyzer or oscilloscope on PWM pins to verify jitter-free 50 Hz / 208 Hz pulse widths while `TWIM0` runs at 1666.6 Hz.
- Inspect serial/BLE telemetry to confirm steady 208.33 Hz cycle time without timing jitter.

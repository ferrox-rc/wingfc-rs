# WingFC-rs Release Matrix & Build Specification

This document defines the production release matrix, build-time feature combinations, memory consumption profiles, and artifact distribution standards for `wingfc-rs`.

---

## 1. Production Release Targets (8 Configurations)

Firmware releases compose into 8 discrete, mutually exclusive production binaries based on three core dimensions:
1. **Receiver Protocol (Required Either/Or):**
   - `crsf` (Crossfire / ExpressLRS @ 420 kBaud with bidirectional telemetry: Attitude, Flight Mode, Battery, GPS/Baro).
   - `ibus` (FlySky iBus @ 115.2 kBaud unidirectional RC control).
2. **Motor Output Protocol:** `dshot` (Digital packet with CRC) or `pwm` (Configurable 50Hz&ndash;400Hz analog PWM).
3. **Navigation Sensor Suite:** `nav` (fusing Barometer Vertical Kalman Filter, 9-DoF Compass AHRS, and GPS Return-to-Home) or baseline (6-DoF IMU).

| Artifact Target Name | Receiver | Motor | Navigation (`nav`) | Features Flag | Flash Size | RAM Size |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `wingfc-firmware-crsf-pwm` | CRSF + Telemetry | PWM 50&ndash;400Hz | Baseline (6-DoF) | `--features crsf` | 36.0 KB (3.5%) | 5.7 KB (2.2%) |
| `wingfc-firmware-crsf-pwm-nav` | CRSF + Telemetry | PWM 50&ndash;400Hz | **Nav (Baro + Compass + GPS)** | `--features crsf,nav` | 37.1 KB (3.6%) | 5.8 KB (2.2%) |
| `wingfc-firmware-crsf-dshot` | CRSF + Telemetry | DShot | Baseline (6-DoF) | `--features crsf,dshot` | 34.7 KB (3.4%) | 5.7 KB (2.2%) |
| `wingfc-firmware-crsf-dshot-nav` | CRSF + Telemetry | DShot | **Nav (Baro + Compass + GPS)** | `--features crsf,dshot,nav` | 35.9 KB (3.5%) | 5.8 KB (2.2%) |
| `wingfc-firmware-ibus-pwm` | iBus | PWM 50&ndash;400Hz | Baseline (6-DoF) | `--features ibus` | 32.4 KB (3.2%) | 5.7 KB (2.2%) |
| `wingfc-firmware-ibus-pwm-nav` | iBus | PWM 50&ndash;400Hz | **Nav (Baro + Compass + GPS)** | `--features ibus,nav` | 32.4 KB (3.2%) | 5.7 KB (2.2%) |
| `wingfc-firmware-ibus-dshot` | iBus | DShot | Baseline (6-DoF) | `--features ibus,dshot` | 31.1 KB (3.0%) | 5.7 KB (2.2%) |
| `wingfc-firmware-ibus-dshot-nav` | iBus | DShot | **Nav (Baro + Compass + GPS)** | `--features ibus,dshot,nav` | 31.1 KB (3.0%) | 5.7 KB (2.2%) |

> **Resource Footprint & Overhead:** Total nRF52840 resources are 1,024 KB Flash and 256 KB RAM. The entire firmware consumes less than **3.7% of Flash** and **2.3% of RAM**, leaving &gt;96% of chip resources available for future flight scripting, complex autonomous missions, and extended telemetry buffers.
>
> CRSF builds incorporate the asynchronous EasyDMA telemetry broadcast engine (transmitting attitude, flight mode, battery, and GPS frames @ 50 Hz over pin `D6` / `P1.11`) requiring only &approx; 3.6 KB additional flash.
>
> DShot builds consume &approx; 1.2&ndash;1.3 KB less flash than analog PWM builds because legacy analog ESC throttle calibration routines and pass-through tables are eliminated at compile time.

---

## 2. Release Artifact Deliverables

Every release produces three file formats per target in the `dist/` directory, plus a cryptographic manifest:

1. **`wingfc-firmware-<variant>.elf`:** ELF binary containing full DWARF debug symbols (used for GDB debugging, probe-rs, and size inspection).
2. **`wingfc-firmware-<variant>.bin`:** Raw binary image for direct memory flashing via J-Link, OpenOCD, or DAPLink.
3. **`wingfc-firmware-<variant>.hex`:** Intel HEX formatted record file for bootloader utilities and GUI flashers.
4. **`SHA256SUMS`:** Standard `sha256sum` manifest containing SHA-256 digests for all release binaries.

---

## 3. Build & Test Commands

The release matrix can be verified or built using either the unified `./scripts/build.sh matrix` command or the direct script `./scripts/build_matrix.sh`:

### Rapid Verification (`check`)
Runs host unit tests for base, barometer, compass, GPS, and full navigation suite, followed by target verification of all 8 release configurations:
```bash
# Using unified build tool
./scripts/build.sh matrix check

# Or using matrix script directly
./scripts/build_matrix.sh check
```
*Execution time:* &approx; 0.8&ndash;1.5 seconds.

#### Test Coverage Metrics (`wingfc-core`)
Host unit tests achieve **88.4% Line Coverage**, **89.8% Region Coverage**, and **89.4% Function Coverage** measured via `cargo-llvm-cov`:

| Subsystem / Module | Source File | Line Coverage | Function Coverage | Region Coverage |
| :--- | :--- | :---: | :---: | :---: |
| **Altitude Safety & Terrain** | `altitude_safety.rs` | **100.0%** | **100.0%** | **100.0%** |
| **Airframe Mixer** | `mixer.rs` | **100.0%** | **100.0%** | **100.0%** |
| **Vertical Kalman Filter** | `vertical_kf.rs` | **97.9%** | **100.0%** | **98.1%** |
| **Hand Autolaunch** | `autolaunch.rs` | **97.8%** | **100.0%** | **96.9%** |
| **Signal Filtering** | `filter.rs` | **96.9%** | **100.0%** | **97.1%** |
| **CRSF Protocol & Telemetry** | `protocol/crsf.rs` | **95.7%** | **91.7%** | **96.3%** |
| **Math & Quaternion Library** | `math.rs` | **94.1%** | **94.7%** | **95.6%** |
| **Arming Safety Interlocks** | `safety.rs` | **88.1%** | **75.0%** | **85.4%** |
| **Rate PID & FeedForward** | `pid.rs` | **84.5%** | **80.0%** | **82.8%** |
| **FlySky iBus Protocol** | `protocol/ibus.rs` | **82.3%** | **71.4%** | **85.7%** |
| **GPS & NMEA Navigation** | `gps.rs` | **79.6%** | **85.7%** | **84.8%** |
| **Madgwick 9-DoF AHRS** | `ahrs.rs` | **79.5%** | **100.0%** | **84.3%** |
| **DShot Motor Protocol** | `dshot.rs` | **76.9%** | **66.7%** | **82.9%** |
| **Total `wingfc-core` Suite** | **All Modules** | **88.4%** | **89.4%** | **89.8%** |

### Automated Release Build (`build`)
Compiles, converts, hashes, and outputs all release binaries to `dist/`:
```bash
# Using unified build tool
./scripts/build.sh matrix build

# Or using matrix script directly
./scripts/build_matrix.sh build
```

### Granular 16-Variant Mode (`--granular`)
For developmental testing with independent Barometer and Compass toggles:
```bash
./scripts/build.sh matrix check -- --granular
./scripts/build_matrix.sh check --granular
```

---

## 4. Manual Cargo Build Commands

Individual release variants can also be built directly with `cargo`:

```bash
# Target: Cortex-M4F with Hardware FPU
TARGET="thumbv7em-none-eabihf"

# CRSF + DShot + Nav (Baro + Compass + GPS)
cargo build --release -p wingfc-firmware --target $TARGET --no-default-features --features crsf,dshot,nav

# CRSF + PWM (Baseline)
cargo build --release -p wingfc-firmware --target $TARGET --no-default-features --features crsf

# iBus + DShot + Nav (Baro + Compass + GPS)
cargo build --release -p wingfc-firmware --target $TARGET --no-default-features --features ibus,dshot,nav
```

---

## 5. Flashing Instructions

*For complete details on memory offsets, binary formats, and all automated flashing methods, see the [Firmware Deployment & Flashing Guide](FLASHING.md).*

### Using `probe-rs` (Recommended for SWD):
```bash
probe-rs run --chip nRF52840_xxAA dist/wingfc-firmware-crsf-dshot-nav.elf
```

### Flashing pre-built `.hex` with `nrfjprog`:
```bash
# Flash Intel HEX using sector erase (preserves Nordic SoftDevice and UF2 bootloader)
nrfjprog --program dist/wingfc-firmware-crsf-dshot-nav.hex --sectorerase --reset
```

### Flashing via UF2 Bootloader (Seeed XIAO):
The Seeed Studio XIAO nRF52840 Sense includes a factory UF2 bootloader that expects application images linked at base address `0x00027000`.

1. **Convert from `.hex` (Recommended):**
   Intel HEX records already contain the embedded `0x00027000` base address from `memory.x`, so omit `-b`:
   ```bash
   python3 scripts/uf2conv/uf2conv.py -c -f 0xADA52840 dist/wingfc-firmware-crsf-dshot-nav.hex -o firmware.uf2
   ```

2. **Convert from `.bin` (Requires `-b 0x27000`):**
   Raw binary files lack address metadata, so you must explicitly specify the application start offset:
   ```bash
   python3 scripts/uf2conv/uf2conv.py -c -b 0x27000 -f 0xADA52840 dist/wingfc-firmware-crsf-dshot-nav.bin -o firmware.uf2
   ```

3. **Verify UF2 Header & Target Address:**
   ```bash
   python3 scripts/uf2conv/uf2conv.py -i firmware.uf2
   ```
   *(Ensure output reports `Target Address is 0x00027000`)*

4. **Deploy:**
   Double-tap the reset button on the board so it mounts as a USB mass storage drive (`XIAO-SENSE`), then copy `firmware.uf2` to the drive.


---

## 6. Automated Workspace Versioning & Release Pipeline

The project utilizes Cargo Workspace inheritance, `cargo-release`, and `git-cliff` to provide a fully automated, reproducible semver release workflow.

### Architecture
- **Workspace Inheritance (`[workspace.package]`):** Root `Cargo.toml` centrally manages versioning, license, repository URLs, and shared workspace dependencies. Sub-crates (`wingfc-core` and `wingfc-firmware`) inherit these values automatically with `version.workspace = true`.
- **`cargo-release` Integration:** Manages atomic version bumps across all workspace crates, updates `Cargo.lock`, executes pre-release changelog hooks, creates consolidated git commits, and generates release tags (`vX.Y.Z`).
- **`git-cliff` Changelog Engine:** Parses conventional commits and feature additions according to [`cliff.toml`](../cliff.toml), autogenerating structured, categorized entries in [`CHANGELOG.md`](../CHANGELOG.md) for each release.

### Triggering a Release

#### Local Developer CLI
Ensure your git working tree is clean, then run:
```bash
# Automated patch release (e.g., v0.1.1 -> v0.1.2)
./scripts/build.sh release patch

# Or specify level directly (patch | minor | major | explicit version)
./scripts/release.sh minor
./scripts/release.sh 0.2.0
```

#### GitHub Actions Automation
Releases can be dispatched directly through GitHub Actions:
1. Navigate to **Actions** &rarr; **Automated Release (cargo-release)**.
2. Click **Run workflow** and select the desired release level (`patch`, `minor`, `major`).
3. The workflow increments versions across the workspace, regenerates `CHANGELOG.md`, creates git tag `vX.Y.Z`, builds all 8 Cortex-M4F firmware binaries, and publishes a new GitHub Release with the autogenerated release notes and checksums.

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
> CRSF builds incorporate the asynchronous EasyDMA telemetry broadcast engine (transmitting attitude, flight mode, battery, and GPS frames @ 50 Hz over pin `D7` / `P0.11`) requiring only &approx; 3.6 KB additional flash.
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

### Using `probe-rs` (Recommended for SWD):
```bash
probe-rs run --chip nRF52840_xxAA dist/wingfc-firmware-crsf-dshot-nav.elf
```

### Flashing pre-built `.hex` with `nrfjprog`:
```bash
nrfjprog --program dist/wingfc-firmware-crsf-dshot-nav.hex --chiperase --reset
```

### Flashing via UF2 Bootloader (Seeed XIAO):
Convert `.hex` to `.uf2` using `uf2conv.py`:
```bash
python3 scripts/uf2conv/uf2conv.py -c -b 0x1000 -f 0xADA52840 dist/wingfc-firmware-crsf-dshot-nav.hex -o firmware.uf2
```
Copy `firmware.uf2` to the mounted USB drive (`XIAO-SENSE`).


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

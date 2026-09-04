# WingFC-rs Release Matrix & Build Specification

This document defines the production release matrix, build-time feature combinations, memory consumption profiles, and artifact distribution standards for `wingfc-rs`.

---

## 1. Production Release Targets (8 Configurations)

Firmware releases compose into 8 discrete, mutually exclusive production binaries based on three core dimensions:
1. **Receiver Protocol (Required Either/Or):** `crsf` (CRSF / ELRS @ 420kBaud) or `ibus` (FlySky iBus @ 115.2kBaud).
2. **Motor Output Protocol:** `dshot` (Digital packet with CRC) or `pwm` (Configurable 50Hz&ndash;400Hz analog PWM).
3. **Navigation Sensor Suite:** `nav` (fusing Barometer Vertical Kalman Filter, 9-DoF Compass AHRS, and GPS Return-to-Home) or baseline (6-DoF IMU).

| Artifact Target Name | Receiver | Motor | Navigation (`nav`) | Features Flag | Flash Size | RAM Size |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `wingfc-firmware-crsf-pwm` | CRSF | PWM 50&ndash;400Hz | Baseline (6-DoF) | `--features crsf` | 32.8 KB (3.2%) | 5.7 KB (2.2%) |
| `wingfc-firmware-crsf-pwm-nav` | CRSF | PWM 50&ndash;400Hz | **Nav (Baro + Compass + GPS)** | `--features crsf,nav` | 32.8 KB (3.2%) | 5.7 KB (2.2%) |
| `wingfc-firmware-crsf-dshot` | CRSF | DShot | Baseline (6-DoF) | `--features crsf,dshot` | 31.5 KB (3.1%) | 5.7 KB (2.2%) |
| `wingfc-firmware-crsf-dshot-nav` | CRSF | DShot | **Nav (Baro + Compass + GPS)** | `--features crsf,dshot,nav` | 31.5 KB (3.1%) | 5.7 KB (2.2%) |
| `wingfc-firmware-ibus-pwm` | iBus | PWM 50&ndash;400Hz | Baseline (6-DoF) | `--features ibus` | 32.2 KB (3.1%) | 5.7 KB (2.2%) |
| `wingfc-firmware-ibus-pwm-nav` | iBus | PWM 50&ndash;400Hz | **Nav (Baro + Compass + GPS)** | `--features ibus,nav` | 32.2 KB (3.1%) | 5.7 KB (2.2%) |
| `wingfc-firmware-ibus-dshot` | iBus | DShot | Baseline (6-DoF) | `--features ibus,dshot` | 30.8 KB (3.0%) | 5.7 KB (2.2%) |
| `wingfc-firmware-ibus-dshot-nav` | iBus | DShot | **Nav (Baro + Compass + GPS)** | `--features ibus,dshot,nav` | 30.8 KB (3.0%) | 5.7 KB (2.2%) |

> **Resource Footprint & Overhead:** Total nRF52840 resources are 1,024 KB Flash and 256 KB RAM. The entire firmware consumes less than **3.3% of Flash** and **2.3% of RAM**, leaving &gt;96% of chip resources available for future flight scripting, complex autonomous missions, and extended telemetry buffers.
>
> DShot builds consume &approx; 1.3 KB less flash than analog PWM builds because legacy analog ESC throttle calibration routines and pass-through tables are eliminated at compile time.

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
*Execution time:* &approx; 1.2&ndash;2.0 seconds.

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
python3 scripts/uf2conv.py -f 0xADA52840 -c -o wingfc.uf2 dist/wingfc-firmware-crsf-dshot-nav.hex
```
Then drag and drop `wingfc.uf2` into the mounted XIAO drive.

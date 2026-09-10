# Contributing to WingFC-rs

Thank you for your interest in contributing to **WingFC-rs**! This project provides bare-metal, high-performance, `#![no_std]` Rust flight controller firmware for sub-250g FPV flying wings, aerobatic aircraft, and autonomous UAVs.

To maintain hard real-time determinism, strict safety standards, and clean release engineering, please review these contributing guidelines before opening issues or pull requests.

---

## 1. Branching Strategy & Workflow

WingFC-rs follows a strict branch separation model:

* **`main` (Production / Release Branch):**
  * Contains only tested, validated release code.
  * Direct commits to `main` are strictly avoided.
  * Tagged releases (e.g., `v0.3.0`, `v0.4.0`) are cut exclusively from `main`.
* **`dev` (Active Integration Branch):**
  * The primary integration and working branch.
  * All feature work, bug fixes, and hardware testing branch from and merge back into `dev`.
* **Feature & Bugfix Branches (`feat/*`, `fix/*`):**
  * Always branch off `dev`:
    ```bash
    git checkout dev
    git pull
    git checkout -b feat/my-new-feature
    # or
    git checkout -b fix/issue-description
    ```
  * Submit pull requests targeting `dev`, never `main`.

---

## 2. Workspace Structure & Architectural Boundaries

```
wingfc-rs/
├── crates/
│   ├── wingfc-core/        # Pure #![no_std] algorithms (AHRS, PID, Mixers, Protocols)
│   └── wingfc-firmware/    # Bare-metal Embassy-nRF firmware for Seeed Studio XIAO nRF52840
├── docs/                   # Technical, pinout, flashing, and architectural specifications
├── scripts/                # Automated build, check, and matrix packaging engines
└── wingfc.yaml             # User-facing aircraft configuration file
```

### Critical Architectural Rules:
1. **`wingfc-core` is Pure & Portable:**
   * Must remain completely `#![no_std]` with zero hardware or HAL dependencies (`embassy-*`, `nrf-*`, etc.).
   * All flight control algorithms (Madgwick AHRS, PID + FeedForward, Airframe Mixers, RC protocol parsers, NMEA parser) live here.
   * Everything in `wingfc-core` must be testable directly on host machines via `cargo test`.
2. **`wingfc-firmware` Isolates Hardware Drivers:**
   * Handles physical SoC peripheral allocations (PWM, TWIM, UARTE, GPIOTE, Watchdog).
   * Relies on the **Embassy-nRF** asynchronous HAL. Direct PAC register writes are discouraged unless required for hardware timing constraints.
3. **Strict Physical Edge Pad Allocation:**
   * Only castellated edge pads (`D0`..`D10` on Standard XIAO Sense; `D0`..`D19` on XIAO Sense Plus) may be allocated.
   * Never map functional signals to underside test pads or solder balls (see [`docs/PINOUT.md`](docs/PINOUT.md)).

---

## 3. Real-Time Performance & Determinism Constraints

Flight safety depends on microsecond determinism. Every contribution to the flight loop must respect the following:

* **Hard Real-Time Loop Budget (500 Hz / 2000 &mu;s):**
  * The primary control loop ticks every 2.0 ms. The entire cycle (IMU burst read + AHRS + PID + Mixer + PWM dispatch) must complete in under **600 &mu;s**.
* **Zero Allocations & Zero Panics:**
  * No dynamic memory allocation (`alloc`, heap vectors, `String`). All buffers must be static, stack-allocated, or fixed-capacity arrays.
  * Flight paths must never panic. Avoid unwraps on critical paths; handle errors gracefully or enter safe fallback states.
* **Non-Blocking I/O Only:**
  * Never introduce blocking delay loops (`Timer::after()`, `std::thread::sleep`, spin-locks) inside `flight_loop.step()`.
  * Sensor reads must utilize hardware EasyDMA or asynchronous interrupts.

---

## 4. Development & Testing Requirements

Before opening a pull request or pushing to `dev`, complete all verification steps:

### Step 1: Run Host Unit Tests
All core algorithms must pass unit tests with high test coverage:
```bash
# Run unit tests
cargo test --package wingfc-core

# Run all test feature permutations
./scripts/build.sh test
```

### Step 2: Validate User Configuration Engine
Verify that configuration parsing and code generation function correctly:
```bash
./scripts/build.sh check
```

### Step 3: Verify the Full 8-Permutation Release Matrix
All 8 build matrix permutations must compile with 0 warnings or errors:
```bash
./scripts/build_matrix.sh
```
The build matrix verifies:
* `crsf-pwm`, `crsf-pwm-nav`, `crsf-dshot`, `crsf-dshot-nav`
* `ibus-pwm`, `ibus-pwm-nav`, `ibus-dshot`, `ibus-dshot-nav`

### Step 4: Code Formatting & Linting
Ensure code adheres to standard Rust formatting:
```bash
cargo fmt --all -- --check
cargo clippy --package wingfc-core --all-targets -- -D warnings
```

---

## 5. Commit Guidelines

We use [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) to automate changelog generation and version tagging:

Format: `<type>(<scope>): <concise description>`

Common Types:
* `feat`: A new feature or capability (e.g., `feat(compass): Add support for IST8310 magnetometer`)
* `fix`: A bug fix (e.g., `fix(stabilization): Eliminate blocking wait on IMU DRDY pin`)
* `docs`: Documentation updates (e.g., `docs(pinout): Document HighDrive requirement on P1.08`)
* `refactor`: Code refactoring without behavioral change (e.g., `refactor(mixer): Simplify elevon differential mixing`)
* `test`: Adding or improving tests (e.g., `test(ahrs): Add 30-degree roll tilt convergence test`)
* `chore`: Maintenance tasks, dependencies, release preparation

---

## 6. Pull Request Process

1. **Keep Pull Requests Focused:** Avoid combining unrelated features, refactors, and styling changes into a single PR.
2. **Update Documentation:** If you add new parameters to [`wingfc.yaml`](wingfc.yaml), update:
   * [`docs/CONFIG.md`](docs/CONFIG.md)
   * [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) (if architectural changes are involved)
   * [`docs/FLOWCHARTS.md`](docs/FLOWCHARTS.md) (if dataflow or state transitions are modified)
3. **PR Description:** Clearly describe:
   * The problem being solved or feature being added.
   * What physical bench testing was performed (board model, receiver, servos, transmitter).
   * Confirm that `./scripts/build_matrix.sh` passed cleanly.

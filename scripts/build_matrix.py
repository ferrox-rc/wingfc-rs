#!/usr/bin/env python3
"""
WingFC-rs Build & Test Matrix Automation

Tests and builds all 16 permutations of wingfc-firmware features:
- Receiver protocol (mutually exclusive): crsf OR ibus
- ESC protocol: dshot (ON) or PWM 400Hz (OFF)
- Barometer: baro (ON/OFF)
- Magnetometer / Compass: compass (ON/OFF)

Usage:
  python3 scripts/build_matrix.py check      # Check compilation of all 16 variants + host tests
  python3 scripts/build_matrix.py build      # Build all 16 release binaries and package to dist/
  python3 scripts/build_matrix.py test-core  # Run all host unit test combinations
"""

import argparse
import hashlib
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

TARGET = "thumbv7em-none-eabihf"
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent

# Hardware limits for nRF52840 (Seeed XIAO nRF52840 Sense)
FLASH_TOTAL_BYTES = 1024 * 1024  # 1 MB Flash
RAM_TOTAL_BYTES = 256 * 1024    # 256 KB RAM

def get_all_permutations(granular=False):
    """
    Generates discrete firmware permutations.
    Default: 8 release configurations combining baro + compass into 'nav':
      (crsf/ibus) x (pwm/dshot) x (standard/nav)
    Granular: 16 configurations with independent baro and compass.
    """
    permutations = []
    protocols = ["crsf", "ibus"]
    dshot_opts = [False, True]

    if not granular:
        nav_opts = [False, True]
        for proto in protocols:
            for dshot in dshot_opts:
                for nav in nav_opts:
                    features = [proto]
                    name_parts = [proto]

                    if dshot:
                        features.append("dshot")
                        name_parts.append("dshot")
                    else:
                        name_parts.append("pwm")

                    if nav:
                        features.append("nav")
                        name_parts.append("nav")

                    name = "-".join(name_parts)
                    permutations.append({
                        "name": name,
                        "features": features,
                        "features_str": ",".join(features),
                    })
    else:
        baro_opts = [False, True]
        compass_opts = [False, True]
        for proto in protocols:
            for dshot in dshot_opts:
                for baro in baro_opts:
                    for compass in compass_opts:
                        features = [proto]
                        name_parts = [proto]

                        if dshot:
                            features.append("dshot")
                            name_parts.append("dshot")
                        else:
                            name_parts.append("pwm")

                        if baro:
                            features.append("baro")
                            name_parts.append("baro")

                        if compass:
                            features.append("compass")
                            name_parts.append("compass")

                        name = "-".join(name_parts)
                        permutations.append({
                            "name": name,
                            "features": features,
                            "features_str": ",".join(features),
                        })

    return permutations


def run_cmd(cmd, cwd=WORKSPACE_ROOT, capture=False):
    """Executes a subprocess command."""
    if capture:
        res = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return res.returncode == 0, res.stdout, res.stderr
    else:
        res = subprocess.run(cmd, cwd=cwd)
        return res.returncode == 0


def run_host_core_tests():
    """Runs wingfc-core tests across feature configurations on the host."""
    print("=" * 70)
    print("STEP 1: HOST CORE UNIT TESTS (wingfc-core)")
    print("=" * 70)

    configs = [
        ("Base (no nav)", []),
        ("Barometer only", ["baro"]),
        ("Compass only", ["compass"]),
        ("Nav Suite (nav: baro + compass)", ["nav"]),
    ]

    all_passed = True
    for desc, features in configs:
        cmd = ["cargo", "test", "-p", "wingfc-core"]
        if features:
            cmd.extend(["--features", ",".join(features)])
        print(f"-> Testing: {desc} (features: {features if features else 'default'})... ", end="", flush=True)
        ok, stdout, stderr = run_cmd(cmd, capture=True)
        if ok:
            print("PASS")
        else:
            print("FAIL")
            print(stderr)
            all_passed = False

    return all_passed


def check_matrix(permutations):
    """Checks compilation for all target firmware permutations."""
    count = len(permutations)
    print("\n" + "=" * 70)
    print(f"STEP 2: FIRMWARE MATRIX CHECK ({count} Permutations for {TARGET})")
    print("=" * 70)

    results = []
    start_total = time.time()

    for idx, perm in enumerate(permutations, 1):
        name = perm["name"]
        feat = perm["features_str"]
        cmd = [
            "cargo", "check", "-p", "wingfc-firmware",
            "--target", TARGET,
            "--no-default-features",
            "--features", feat,
        ]

        t0 = time.time()
        print(f"[{idx:02d}/{count:02d}] Checking {name:<32} (features: {feat:<28})... ", end="", flush=True)
        ok, stdout, stderr = run_cmd(cmd, capture=True)
        elapsed = time.time() - t0

        if ok:
            print(f"PASS ({elapsed:.2f}s)")
            results.append((name, feat, True, elapsed, ""))
        else:
            print(f"FAIL ({elapsed:.2f}s)")
            results.append((name, feat, False, elapsed, stderr))

    total_elapsed = time.time() - start_total
    all_ok = all(r[2] for r in results)

    print("\n" + "-" * 70)
    print(f"Matrix Check Summary: {'ALL ' + str(count) + ' PASSED' if all_ok else 'SOME FAILED'} in {total_elapsed:.1f}s")
    print("-" * 70)
    return all_ok


def parse_llvm_size(elf_path):
    """Parses text, data, bss from llvm-size / size."""
    size_bin = shutil.which("llvm-size") or shutil.which("size")
    if not size_bin:
        return None, None, None

    ok, out, _ = run_cmd([size_bin, str(elf_path)], capture=True)
    if not ok:
        return None, None, None

    lines = out.strip().splitlines()
    if len(lines) >= 2:
        parts = lines[1].split()
        if len(parts) >= 3:
            try:
                text = int(parts[0])
                data = int(parts[1])
                bss = int(parts[2])
                return text, data, bss
            except ValueError:
                pass
    return None, None, None


def build_matrix(permutations, out_dir):
    """Builds and packages all release binaries into out_dir."""
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    objcopy_bin = shutil.which("llvm-objcopy") or shutil.which("rust-objcopy") or shutil.which("arm-none-eabi-objcopy")

    count = len(permutations)
    print("\n" + "=" * 80)
    print(f"BUILDING RELEASE BINARIES FOR ALL {count} PERMUTATIONS ({TARGET})")
    print(f"Output directory: {out_dir}")
    print("=" * 80)

    results = []
    built_elf = WORKSPACE_ROOT / "target" / TARGET / "release" / "wingfc-firmware"
    hashes = {}

    for idx, perm in enumerate(permutations, 1):
        name = perm["name"]
        feat = perm["features_str"]
        cmd = [
            "cargo", "build", "--release", "-p", "wingfc-firmware",
            "--target", TARGET,
            "--no-default-features",
            "--features", feat,
        ]

        t0 = time.time()
        print(f"[{idx:02d}/{count:02d}] Building {name:<32} (features: {feat:<28})... ", end="", flush=True)
        ok, stdout, stderr = run_cmd(cmd, capture=True)
        elapsed = time.time() - t0

        if not ok or not built_elf.exists():
            print(f"FAIL ({elapsed:.2f}s)")
            results.append({"name": name, "feat": feat, "ok": False, "err": stderr})
            continue

        # Target file base
        base_name = f"wingfc-firmware-{name}"
        dest_elf = out_dir / f"{base_name}.elf"
        dest_bin = out_dir / f"{base_name}.bin"
        dest_hex = out_dir / f"{base_name}.hex"

        shutil.copy2(built_elf, dest_elf)

        # Generate .bin and .hex if objcopy available
        if objcopy_bin:
            run_cmd([objcopy_bin, "-O", "binary", str(dest_elf), str(dest_bin)], capture=True)
            run_cmd([objcopy_bin, "-O", "ihex", str(dest_elf), str(dest_hex)], capture=True)

        # Compute SHA256 of artifacts
        for p in [dest_elf, dest_bin, dest_hex]:
            if p.exists():
                h = hashlib.sha256(p.read_bytes()).hexdigest()
                hashes[p.name] = h

        # Calculate memory consumption
        text, data, bss = parse_llvm_size(dest_elf)
        flash_used = (text + data) if text is not None else dest_bin.stat().st_size if dest_bin.exists() else 0
        ram_used = (data + bss) if data is not None else 0

        flash_pct = (flash_used / FLASH_TOTAL_BYTES) * 100.0
        ram_pct = (ram_used / RAM_TOTAL_BYTES) * 100.0

        print(f"DONE in {elapsed:.1f}s | Flash: {flash_used / 1024:.1f} KB ({flash_pct:.1f}%) | RAM: {ram_used / 1024:.1f} KB ({ram_pct:.1f}%)")

        results.append({
            "name": name,
            "feat": feat,
            "ok": True,
            "elapsed": elapsed,
            "flash_bytes": flash_used,
            "flash_pct": flash_pct,
            "ram_bytes": ram_used,
            "ram_pct": ram_pct,
        })

    # Write SHA256SUMS file
    sums_file = out_dir / "SHA256SUMS"
    with open(sums_file, "w") as f:
        for fname in sorted(hashes.keys()):
            f.write(f"{hashes[fname]}  {fname}\n")
    print(f"\n-> Checksums written to: {sums_file}")

    # Print summary table
    print("\n" + "=" * 92)
    print(f"{'Variant Name':<32} {'Features':<28} {'Flash (KB)':<12} {'RAM (KB)':<10} {'Status'}")
    print("=" * 92)
    for r in results:
        if r["ok"]:
            f_str = f"{r['flash_bytes'] / 1024:.1f} ({r['flash_pct']:.1f}%)"
            ram_str = f"{r['ram_bytes'] / 1024:.1f} ({r['ram_pct']:.1f}%)"
            print(f"{r['name']:<32} {r['feat']:<28} {f_str:<12} {ram_str:<10} OK")
        else:
            print(f"{r['name']:<32} {r['feat']:<28} {'-':<12} {'-':<10} FAILED")
    print("=" * 92)

    return all(r["ok"] for r in results)


def main():
    parser = argparse.ArgumentParser(description="WingFC-rs Matrix Test & Release Builder")
    parser.add_argument(
        "action",
        nargs="?",
        default="all",
        choices=["all", "check", "build", "test-core"],
        help="Action to perform: 'check' (compile check matrix), 'build' (produce release binaries), 'test-core' (host unit tests), 'all' (tests + check + build)",
    )
    parser.add_argument(
        "--out-dir",
        default=str(WORKSPACE_ROOT / "dist"),
        help="Output directory for built release artifacts (default: dist/)",
    )
    parser.add_argument(
        "--granular",
        action="store_true",
        help="Build all 16 granular permutations with independent baro/compass instead of the 8 combined nav permutations",
    )

    args = parser.parse_args()
    permutations = get_all_permutations(granular=args.granular)

    if args.action == "test-core":
        ok = run_host_core_tests()
        sys.exit(0 if ok else 1)

    elif args.action == "check":
        ok_core = run_host_core_tests()
        ok_matrix = check_matrix(permutations)
        sys.exit(0 if (ok_core and ok_matrix) else 1)

    elif args.action == "build":
        ok = build_matrix(permutations, args.out_dir)
        sys.exit(0 if ok else 1)

    elif args.action == "all":
        ok_core = run_host_core_tests()
        if not ok_core:
            print("Core tests failed! Aborting build.")
            sys.exit(1)
        ok_build = build_matrix(permutations, args.out_dir)
        sys.exit(0 if ok_build else 1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
WingFC Blackbox Log Offload & Analysis Tool.

Communicates with WingFC-rs flight controller over either:
  1. XIAO DebugMate SWD (`probe-rs` via in-RAM `BBL_SWD_MAILBOX`), or
  2. Native USB CDC-ACM serial port
to inspect, download, and decode high-rate 50 Hz binary flight logs from onboard QSPI NOR flash.

Automatically formats `.bbl` / `.bfl` output files into the standard Betaflight Blackbox v2
container format so they load directly into the online Betaflight Blackbox Explorer.
"""

import argparse
import binascii
import csv
import math
import os
import struct
import subprocess
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

FRAME_SIZE = 32
MAGIC_BYTE = 0xBB
VERSION_BYTE = 0x01

LOG_START_ADDR = 0x004000
SWD_MAILBOX_MAGIC = 0x57464342
SWD_CMD_NONE = 0
SWD_CMD_ERASE_USED = 1
SWD_CMD_ERASE_ALL = 2
SWD_CMD_READ_CHUNK = 3

SWD_STATUS_IDLE = 0
SWD_STATUS_BUSY = 1
SWD_STATUS_ERR_ARMED = 2
SWD_STATUS_ERR_IO = 3

SWD_BUF_BYTES = 4096

# Betaflight Blackbox v2 schema definition:
# (name, signed, I_predictor, I_encoding, P_predictor, P_encoding)
# Encodings: 0 = SIGNED_VB, 1 = UNSIGNED_VB, 9 = NULL
# Predictors: 0 = PREDICTOR_0, 1 = PREDICTOR_PREVIOUS, 6 = PREDICTOR_INC
BETAFLIGHT_BBL_FIELDS = [
    ("loopIteration", 0, 0, 1, 6, 9),
    ("time",          0, 0, 1, 1, 0),
    ("axisP[0]",      1, 0, 0, 1, 0),
    ("axisP[1]",      1, 0, 0, 1, 0),
    ("axisP[2]",      1, 0, 0, 1, 0),
    ("axisI[0]",      1, 0, 0, 1, 0),
    ("axisI[1]",      1, 0, 0, 1, 0),
    ("axisI[2]",      1, 0, 0, 1, 0),
    ("axisD[0]",      1, 0, 0, 1, 0),
    ("axisD[1]",      1, 0, 0, 1, 0),
    ("rcCommand[0]",  1, 0, 0, 1, 0),
    ("rcCommand[1]",  1, 0, 0, 1, 0),
    ("rcCommand[2]",  1, 0, 0, 1, 0),
    ("rcCommand[3]",  0, 0, 1, 1, 0),
    ("gyroADC[0]",    1, 0, 0, 1, 0),
    ("gyroADC[1]",    1, 0, 0, 1, 0),
    ("gyroADC[2]",    1, 0, 0, 1, 0),
    ("accSmooth[0]",  1, 0, 0, 1, 0),
    ("accSmooth[1]",  1, 0, 0, 1, 0),
    ("accSmooth[2]",  1, 0, 0, 1, 0),
    ("motor[0]",      0, 0, 1, 1, 0),
    ("motor[1]",      0, 0, 1, 1, 0),
    ("servo[0]",      0, 0, 1, 1, 0),
    ("servo[1]",      0, 0, 1, 1, 0),
    ("debug[0]",      1, 0, 0, 1, 0),
    ("debug[1]",      1, 0, 0, 1, 0),
    ("debug[2]",      1, 0, 0, 1, 0),
    ("debug[3]",      1, 0, 0, 1, 0),
]


def crc8_itu(data: bytes) -> int:
    """Computes CRC-8 with polynomial 0x07 (ATM/ITU), matching firmware."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def encode_unsigned_vb(val: int) -> bytes:
    """Encodes a 32-bit unsigned integer using Betaflight unsigned variable-byte encoding."""
    val &= 0xFFFFFFFF
    out = bytearray()
    while val > 0x7F:
        out.append((val & 0x7F) | 0x80)
        val >>= 7
    out.append(val & 0x7F)
    return bytes(out)


def encode_signed_vb(val: int) -> bytes:
    """Encodes a 32-bit signed integer using ZigZag + Betaflight variable-byte encoding."""
    val = int(val)
    if val < -0x80000000:
        val = -0x80000000
    elif val > 0x7FFFFFFF:
        val = 0x7FFFFFFF
    zigzag = ((val << 1) ^ (val >> 31)) & 0xFFFFFFFF
    return encode_unsigned_vb(zigzag)


def decode_unsigned_vb(stream: bytes, idx: int) -> tuple[int, int]:
    """Decodes a 32-bit unsigned integer from Betaflight variable-byte encoding."""
    val = 0
    shift = 0
    while idx < len(stream):
        b = stream[idx]
        idx += 1
        val |= (b & 0x7F) << shift
        if (b & 0x80) == 0:
            break
        shift += 7
    return val & 0xFFFFFFFF, idx


def decode_signed_vb(stream: bytes, idx: int) -> tuple[int, int]:
    """Decodes a 32-bit signed integer from ZigZag + Betaflight variable-byte encoding."""
    u_val, idx = decode_unsigned_vb(stream, idx)
    val = (u_val >> 1) ^ (-(u_val & 1))
    return val, idx


def decode_frame(data: bytes, strict_crc: bool = True):
    """Decodes a single 32-byte WingFC BlackboxFrame (v1 or v2)."""
    if len(data) != 32:
        return None
    ver = data[1]
    if ver not in (1, 2):
        return None

    if strict_crc:
        if data[0] != MAGIC_BYTE:
            return None
        if crc8_itu(data[:31]) != data[31]:
            return None
    else:
        if data[0] not in (0xBB, 0xB3):
            return None

    status_flags = data[2] & 0x03
    seq_lo = data[3]
    timestamp_ms = struct.unpack("<I", data[4:8])[0] & 0x000FFFFF
    gx, gy, gz = struct.unpack("<hhh", data[8:14])
    ax, ay, az = struct.unpack("<hhh", data[14:20])

    ax_ms2 = ax / 100.0
    ay_ms2 = ay / 100.0
    az_ms2 = az / 100.0

    rad_to_deg = 180.0 / math.pi
    if ver == 2:
        # Version 2: bytes 20..25 are [pid_pitch: 3x i8, pid_roll: 3x i8] scaled by 500
        pitch_p_raw, pitch_i_raw, pitch_d_raw = struct.unpack("<bbb", data[20:23])
        roll_p_raw, roll_i_raw, roll_d_raw = struct.unpack("<bbb", data[23:26])
        roll_deg = round(max(-180.0, min(180.0, math.atan2(ay_ms2, az_ms2) * rad_to_deg)), 2)
        pitch_deg = round(max(-90.0, min(90.0, math.atan2(-ax_ms2, math.hypot(ay_ms2, az_ms2)) * rad_to_deg)), 2)
        yaw_deg = 0.0
        pitch_p = round(pitch_p_raw / 500.0, 4)
        pitch_i = round(pitch_i_raw / 500.0, 4)
        pitch_d = round(pitch_d_raw / 500.0, 4)
        roll_p = round(roll_p_raw / 500.0, 4)
        roll_i = round(roll_i_raw / 500.0, 4)
        roll_d = round(roll_d_raw / 500.0, 4)
    else:
        # Version 1 legacy: bytes 20..25 are [roll_mrad, pitch_mrad, yaw_mrad: 3x i16]
        roll_mrad, pitch_mrad, yaw_mrad = struct.unpack("<hhh", data[20:26])
        roll_deg = round(max(-180.0, min(180.0, (roll_mrad / 1000.0) * rad_to_deg)), 2)
        pitch_deg = round(max(-90.0, min(90.0, (pitch_mrad / 1000.0) * rad_to_deg)), 2)
        yaw_deg = round(max(-180.0, min(180.0, (yaw_mrad / 1000.0) * rad_to_deg)), 2)
        pitch_p = pitch_i = pitch_d = 0.0
        roll_p = roll_i = roll_d = 0.0

    # Unpack actuators (3x 10-bit in 32-bit uint)
    act_packed = struct.unpack("<I", data[26:30])[0]
    servo1 = max(1000, min(2000, (act_packed & 0x3FF) + 1000))
    servo2 = max(1000, min(2000, ((act_packed >> 10) & 0x3FF) + 1000))
    throttle = max(1000, min(2000, ((act_packed >> 20) & 0x3FF) + 1000))

    return {
        "time_ms": timestamp_ms,
        "seq": seq_lo,
        "status": status_flags,
        "gx_rads": round(gx / 1000.0, 4),
        "gy_rads": round(gy / 1000.0, 4),
        "gz_rads": round(gz / 1000.0, 4),
        "ax_ms2": round(ax_ms2, 3),
        "ay_ms2": round(ay_ms2, 3),
        "az_ms2": round(az_ms2, 3),
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg,
        "roll_p": roll_p,
        "roll_i": roll_i,
        "roll_d": roll_d,
        "pitch_p": pitch_p,
        "pitch_i": pitch_i,
        "pitch_d": pitch_d,
        "servo1_us": servo1,
        "servo2_us": servo2,
        "throttle_us": throttle,
    }


def decode_betaflight_bbl(raw: bytes) -> list[dict]:
    """Decodes a Betaflight Blackbox v2 (.bbl) binary stream back into WingFC frame dictionaries."""
    marker = b"H debug_mode:0\n"
    idx_start = raw.find(marker)
    if idx_start == -1:
        return []
    pos = idx_start + len(marker)
    num_fields = len(BETAFLIGHT_BBL_FIELDS)
    deg_to_rad = math.pi / 180.0
    rad_to_deg = 180.0 / math.pi

    rows = []
    while pos < len(raw):
        tag = raw[pos : pos + 1]
        pos += 1
        if tag == b"I":
            row = []
            for f_idx in range(num_fields):
                i_encoding = BETAFLIGHT_BBL_FIELDS[f_idx][3]
                if i_encoding == 1:
                    val, pos = decode_unsigned_vb(raw, pos)
                else:
                    val, pos = decode_signed_vb(raw, pos)
                row.append(val)
            rows.append(row)
        elif tag == b"P":
            if not rows:
                break
            row = list(rows[-1])
            row[0] += 1  # PREDICTOR_INC on loopIteration
            for f_idx in range(1, num_fields):
                delta, pos = decode_signed_vb(raw, pos)
                row[f_idx] += delta
            rows.append(row)
        elif tag == b"E":
            break
        else:
            break

    frames = []
    for r in rows:
        idx = r[0]
        time_ms = r[1] // 1000
        roll_p = round(r[2] / 1000.0, 4)
        pitch_p = round(r[3] / 1000.0, 4)
        roll_i = round(r[5] / 1000.0, 4)
        pitch_i = round(r[6] / 1000.0, 4)
        roll_d = round(r[8] / 1000.0, 4)
        pitch_d = round(r[9] / 1000.0, 4)
        gx_rads = round(r[14] * deg_to_rad, 4)
        gy_rads = round(r[15] * deg_to_rad, 4)
        gz_rads = round(r[16] * deg_to_rad, 4)
        ax_ms2 = round((r[17] / 1000.0) * 9.80665, 3)
        ay_ms2 = round((r[18] / 1000.0) * 9.80665, 3)
        az_ms2 = round((r[19] / 1000.0) * 9.80665, 3)
        thr = r[20]
        s1 = r[22]
        s2 = r[23]
        roll_deg = round(r[24] / 10.0, 2)
        pitch_deg = round(r[25] / 10.0, 2)
        yaw_deg = round(r[26] / 10.0, 2)
        if roll_deg == 0.0 and pitch_deg == 0.0 and (ax_ms2 != 0.0 or ay_ms2 != 0.0 or az_ms2 != 0.0):
            roll_deg = round(max(-180.0, min(180.0, math.atan2(ay_ms2, az_ms2) * rad_to_deg)), 2)
            pitch_deg = round(max(-90.0, min(90.0, math.atan2(-ax_ms2, math.hypot(ay_ms2, az_ms2)) * rad_to_deg)), 2)
        status = r[27]

        frames.append({
            "time_ms": time_ms,
            "seq": idx & 0xFF,
            "status": status,
            "gx_rads": gx_rads,
            "gy_rads": gy_rads,
            "gz_rads": gz_rads,
            "ax_ms2": ax_ms2,
            "ay_ms2": ay_ms2,
            "az_ms2": az_ms2,
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "yaw_deg": yaw_deg,
            "roll_p": roll_p,
            "roll_i": roll_i,
            "roll_d": roll_d,
            "pitch_p": pitch_p,
            "pitch_i": pitch_i,
            "pitch_d": pitch_d,
            "servo1_us": s1,
            "servo2_us": s2,
            "throttle_us": thr,
        })
    return frames


def extract_frames_from_raw(raw: bytes) -> list[dict]:
    """Extracts decoded frames from raw 32-byte MCU flash blocks (or a Betaflight .bbl stream)."""
    if raw.startswith(b"H Product:Blackbox"):
        return decode_betaflight_bbl(raw)

    total_chunks = len(raw) // FRAME_SIZE
    strict_frames = []
    relaxed_frames = []

    for i in range(total_chunks):
        chunk = raw[i * FRAME_SIZE : (i + 1) * FRAME_SIZE]
        if chunk == b"\xFF" * FRAME_SIZE:
            continue
        f_strict = decode_frame(chunk, strict_crc=True)
        if f_strict:
            strict_frames.append(f_strict)
        f_relaxed = decode_frame(chunk, strict_crc=False)
        if f_relaxed:
            relaxed_frames.append(f_relaxed)

    if len(strict_frames) >= max(1, len(relaxed_frames) // 2):
        return strict_frames
    return relaxed_frames


def encode_betaflight_bbl(frames: list[dict]) -> bytes:
    """Encodes decoded WingFC frames into a Betaflight Blackbox v2 (.bbl) binary stream."""
    names = ",".join(f[0] for f in BETAFLIGHT_BBL_FIELDS)
    signed = ",".join(str(f[1]) for f in BETAFLIGHT_BBL_FIELDS)
    i_pred = ",".join(str(f[2]) for f in BETAFLIGHT_BBL_FIELDS)
    i_enc = ",".join(str(f[3]) for f in BETAFLIGHT_BBL_FIELDS)
    p_pred = ",".join(str(f[4]) for f in BETAFLIGHT_BBL_FIELDS)
    p_enc = ",".join(str(f[5]) for f in BETAFLIGHT_BBL_FIELDS)

    headers = [
        "H Product:Blackbox flight data recorder by Nicholas Sherlock",
        "H Data version:2",
        "H Firmware type:Cleanflight",
        "H Firmware revision:Betaflight 4.4.0",
        "H Firmware date:Oct 26 2026 12:00:00",
        "H Board information:WINGFC_XIAO_NRF52840",
        "H Craft name:WingFC-rs",
        "H I interval:32",
        "H P interval:1/1",
        f"H Field I name:{names}",
        f"H Field I signed:{signed}",
        f"H Field I predictor:{i_pred}",
        f"H Field I encoding:{i_enc}",
        f"H Field P predictor:{p_pred}",
        f"H Field P encoding:{p_enc}",
        "H gyro_scale:0x3f800000",
        "H acc_1G:1000",
        "H motorOutput:1000,2000",
        "H minthrottle:1000",
        "H maxthrottle:2000",
        "H looptime:20000",
        "H debug_mode:0",
    ]

    out = bytearray(("\n".join(headers) + "\n").encode("ascii"))
    rad_to_deg = 180.0 / math.pi
    prev_row = None

    for idx, f in enumerate(frames):
        time_us = idx * 20000  # 50 Hz = 20,000 us per logged frame
        gx_dps = int(round(f["gx_rads"] * rad_to_deg))
        gy_dps = int(round(f["gy_rads"] * rad_to_deg))
        gz_dps = int(round(f["gz_rads"] * rad_to_deg))

        ax_mg = int(round((f["ax_ms2"] / 9.80665) * 1000.0))
        ay_mg = int(round((f["ay_ms2"] / 9.80665) * 1000.0))
        az_mg = int(round((f["az_ms2"] / 9.80665) * 1000.0))

        s1 = int(f["servo1_us"])
        s2 = int(f["servo2_us"])
        thr = int(f["throttle_us"])

        rc0 = s1 - 1500
        rc1 = s2 - 1500
        rc2 = 0
        rc3 = thr

        # Scale PID terms to Betaflight mixer effort scale (1.0 full mix = ~1000)
        roll_p_bf = int(round(f["roll_p"] * 1000.0))
        pitch_p_bf = int(round(f["pitch_p"] * 1000.0))
        roll_i_bf = int(round(f["roll_i"] * 1000.0))
        pitch_i_bf = int(round(f["pitch_i"] * 1000.0))
        roll_d_bf = int(round(f["roll_d"] * 1000.0))
        pitch_d_bf = int(round(f["pitch_d"] * 1000.0))

        dbg0 = int(round(f["roll_deg"] * 10.0))
        dbg1 = int(round(f["pitch_deg"] * 10.0))
        dbg2 = int(round(f["yaw_deg"] * 10.0))
        dbg3 = int(f["status"])

        row = [
            idx,          # loopIteration
            time_us,      # time (us)
            roll_p_bf, pitch_p_bf, 0,  # axisP[0..2] (Roll, Pitch, Yaw)
            roll_i_bf, pitch_i_bf, 0,  # axisI[0..2] (Roll, Pitch, Yaw)
            roll_d_bf, pitch_d_bf,     # axisD[0..1] (Roll, Pitch)
            rc0, rc1, rc2, rc3,        # rcCommand[0..3]
            gx_dps, gy_dps, gz_dps,    # gyroADC[0..2] (deg/s)
            ax_mg, ay_mg, az_mg,       # accSmooth[0..2] (milli-G, 1G=1000)
            thr, thr,                  # motor[0..1]
            s1, s2,                    # servo[0..1]
            dbg0, dbg1, dbg2, dbg3,    # debug[0..3]
        ]

        if idx % 32 == 0 or prev_row is None:
            out.append(ord("I"))
            for val, field_def in zip(row, BETAFLIGHT_BBL_FIELDS):
                i_encoding = field_def[3]
                if i_encoding == 1:
                    out.extend(encode_unsigned_vb(val))
                else:
                    out.extend(encode_signed_vb(val))
        else:
            out.append(ord("P"))
            for val, pval, field_def in zip(row, prev_row, BETAFLIGHT_BBL_FIELDS):
                p_encoding = field_def[5]
                if p_encoding == 9:
                    # ENCODING_NULL (loopIteration increments automatically via PREDICTOR_INC=6)
                    continue
                delta = val - pval
                out.extend(encode_signed_vb(delta))

        prev_row = row

    # Standard Betaflight End-of-Log Event (0x45 = 'E', 0xFF = FLIGHT_LOG_EVENT_LOG_END)
    out.extend(b"E\xffEnd of log\x00")
    return bytes(out)


def find_wingfc_usb_port():
    """Returns native WingFC USB CDC-ACM port if connected, else None."""
    if not serial:
        return None
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        mfg = (p.manufacturer or "").lower()
        if p.vid == 0x0D28 and p.pid == 0x0204:
            continue
        if "cmsis-dap" in desc:
            continue
        if (p.vid == 0x1209 and p.pid == 0x0001) or "wingfc" in desc or "wingfc" in mfg:
            return p.device
    return None


def should_use_swd(args) -> bool:
    """Determines whether to communicate via XIAO DebugMate SWD (probe-rs) or USB CDC."""
    if getattr(args, "swd", False):
        return True
    if getattr(args, "port", None):
        return False
    return find_wingfc_usb_port() is None


def find_elf_path(custom_elf=None) -> str:
    """Locates the compiled wingfc-firmware ELF binary to resolve BBL_SWD_MAILBOX."""
    if custom_elf and os.path.isfile(custom_elf):
        return custom_elf

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates = [
        os.path.join(repo_root, "target", "thumbv7em-none-eabihf", "debug", "wingfc-firmware"),
        os.path.join(repo_root, "target", "thumbv7em-none-eabihf", "release", "wingfc-firmware"),
    ]
    existing = [p for p in candidates if os.path.isfile(p)]
    if not existing:
        print(
            "Error: Compiled firmware ELF not found. Build or flash the firmware first with 'cargo run'.",
            file=sys.stderr,
        )
        sys.exit(1)
    existing.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return existing[0]


def resolve_symbol_address(elf_path: str, symbol_name: str) -> int:
    """Extracts a symbol's virtual address directly from a 32-bit little-endian ELF file."""
    with open(elf_path, "rb") as f:
        data = f.read()

    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        raise RuntimeError(f"Unsupported ELF format in {elf_path}")

    e_shoff = struct.unpack_from("<I", data, 32)[0]
    e_shentsize, e_shnum, _ = struct.unpack_from("<HHH", data, 46)

    sections = []
    for i in range(e_shnum):
        off = e_shoff + i * e_shentsize
        sh_name, sh_type, sh_flags, sh_addr, sh_offset, sh_size, sh_link, sh_info, sh_addralign, sh_entsize = (
            struct.unpack_from("<IIIIIIIIII", data, off)
        )
        sections.append((sh_type, sh_offset, sh_size, sh_link, sh_entsize))

    target_bytes = symbol_name.encode("ascii")
    for sh_type, sh_offset, sh_size, sh_link, sh_entsize in sections:
        if sh_type == 2 and sh_entsize >= 16:
            strtab_off = sections[sh_link][1]
            count = sh_size // sh_entsize
            for i in range(count):
                sym_off = sh_offset + i * sh_entsize
                st_name, st_value, st_size, st_info, st_other, st_shndx = struct.unpack_from(
                    "<IIIBBH", data, sym_off
                )
                if st_name == 0:
                    continue
                end = data.find(b"\x00", strtab_off + st_name)
                name = data[strtab_off + st_name : end]
                if name == target_bytes:
                    return st_value

    raise RuntimeError(
        f"Symbol '{symbol_name}' not found in {elf_path}. Re-flash the updated firmware with 'cargo run' first."
    )


def swd_read_words(addr: int, count: int, speed: int = 400) -> list[int]:
    """Reads `count` 32-bit words from target memory at `addr` via probe-rs."""
    cmd = [
        "probe-rs",
        "read",
        "--chip",
        "nRF52840_xxAA",
        "--speed",
        str(speed),
        "b32",
        f"0x{addr:08X}",
        str(count),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        err = (res.stderr or res.stdout).strip()
        print(
            f"\nError communicating via XIAO DebugMate (SWD):\n{err}\n\n"
            "Tip: If 'cargo run' is still running in another terminal, press Ctrl+C there so DebugMate SWD is free.",
            file=sys.stderr,
        )
        sys.exit(1)

    words = []
    for line in res.stdout.splitlines():
        if ":" not in line:
            continue
        _, hex_part = line.split(":", 1)
        for token in hex_part.strip().split():
            words.append(int(token, 16))
    return words[:count]


def swd_write_words(addr: int, values: list[int], speed: int = 400):
    """Writes 32-bit words to target memory at `addr` via probe-rs."""
    cmd = [
        "probe-rs",
        "write",
        "--chip",
        "nRF52840_xxAA",
        "--speed",
        str(speed),
        "b32",
        f"0x{addr:08X}",
    ] + [f"0x{v:08X}" for v in values]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        err = (res.stderr or res.stdout).strip()
        print(f"\nError writing via XIAO DebugMate (SWD):\n{err}", file=sys.stderr)
        sys.exit(1)


def get_swd_mailbox_addr(args) -> int:
    elf_path = find_elf_path(getattr(args, "elf", None))
    try:
        return resolve_symbol_address(elf_path, "BBL_SWD_MAILBOX")
    except RuntimeError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


def open_serial(port: str, baud: int = 115200, timeout: float = 3.0):
    if not serial:
        print("Error: 'pyserial' is required for serial communication. Install via 'pip install pyserial'.", file=sys.stderr)
        sys.exit(1)
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser
    except Exception as e:
        print(f"Error opening serial port {port}: {e}", file=sys.stderr)
        sys.exit(1)


def print_status_summary(transport: str, jedec: str, wr_ptr: int, sessions: int, used_bytes: int, capacity: int):
    pct = (used_bytes / capacity) * 100.0 if capacity > 0 else 0.0
    rec_sec = used_bytes / (50.0 * FRAME_SIZE)
    mins = int(rec_sec // 60)
    secs = int(rec_sec % 60)

    print("\n--- WingFC QSPI Blackbox Status ---")
    print(f"Transport      : {transport}")
    print(f"Flash Hardware : Puya P25Q16H (JEDEC: 0x{jedec})")
    print(f"Write Pointer  : 0x{wr_ptr:06X}")
    print(f"Flight Sessions: {sessions}")
    print(f"Used Capacity  : {used_bytes:,} / {capacity:,} bytes ({pct:.1f}%)")
    print(f"Logged Time    : {mins}m {secs:02d}s (approx. at 50 Hz)")


CSV_FIELDNAMES = [
    "time_ms", "seq", "status",
    "gx_rads", "gy_rads", "gz_rads",
    "ax_ms2", "ay_ms2", "az_ms2",
    "roll_deg", "pitch_deg", "yaw_deg",
    "roll_p", "roll_i", "roll_d",
    "pitch_p", "pitch_i", "pitch_d",
    "servo1_us", "servo2_us", "throttle_us",
]


def resolve_output_paths(output_arg: str) -> tuple[str, str, str]:
    """Resolves base stem from user output argument into (.bbl, .csv, .bin) paths."""
    base, ext = os.path.splitext(output_arg)
    if ext.lower() in (".bbl", ".csv", ".bin", ".raw"):
        stem = base
    else:
        stem = output_arg
    return f"{stem}.bbl", f"{stem}.csv", f"{stem}.bin"


def write_csv_frames(frames: list[dict], csv_path: str):
    """Writes decoded flight frames to a CSV file."""
    with open(csv_path, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        for frame in frames:
            writer.writerow(frame)


def save_dumped_data(raw_bytes: bytes, output_path: str, raw_flag: bool = False):
    """Simultaneously converts raw QSPI flash data into Betaflight v2 (.bbl) and CSV (.csv) formats."""
    bbl_path, csv_path, bin_path = resolve_output_paths(output_path)

    if raw_flag or output_path.lower().endswith((".bin", ".raw")):
        raw_out = output_path if output_path.lower().endswith((".bin", ".raw")) else bin_path
        with open(raw_out, "wb") as f:
            f.write(raw_bytes)
        print(f"Saved raw QSPI flash bytes : {raw_out} ({len(raw_bytes):,} bytes)")

    frames = extract_frames_from_raw(raw_bytes)
    duration_s = len(frames) / 50.0

    # 1. Encode & write Betaflight Blackbox v2 (.bbl)
    bf_bytes = encode_betaflight_bbl(frames)
    with open(bbl_path, "wb") as f:
        f.write(bf_bytes)

    # 2. Format & write CSV Telemetry Log (.csv)
    write_csv_frames(frames, csv_path)
    csv_size = os.path.getsize(csv_path)

    print(f"Decoded {len(frames):,} flight frames ({duration_s:.1f}s @ 50 Hz) from raw QSPI stream:")
    print(f"  -> Betaflight Blackbox : {bbl_path} ({len(bf_bytes):,} bytes)")
    print(f"  -> CSV Telemetry Log   : {csv_path} ({csv_size:,} bytes)")


def cmd_info(args):
    if should_use_swd(args):
        mb_addr = get_swd_mailbox_addr(args)
        print(f"Connecting to WingFC via XIAO DebugMate SWD (Mailbox @ 0x{mb_addr:08X})...")
        words = swd_read_words(mb_addr, 8)
        magic, write_ptr, capacity, sessions, status, cmd, _, _ = words
        if magic != SWD_MAILBOX_MAGIC:
            print(
                f"Error: SWD Mailbox magic mismatch (got 0x{magic:08X}, expected 0x{SWD_MAILBOX_MAGIC:08X}).\n"
                "Make sure the latest firmware is flashed and running on the board.",
                file=sys.stderr,
            )
            sys.exit(1)
        used_bytes = max(0, write_ptr - LOG_START_ADDR)
        print_status_summary("XIAO DebugMate (SWD)", "856015", write_ptr, sessions, used_bytes, capacity)
        return

    port = args.port or find_wingfc_usb_port() or "/dev/ttyACM0"
    print(f"Connecting to WingFC on {port}...")
    ser = open_serial(port, args.baud)

    ser.write(b"BBL_INFO\r\n")
    line = ser.readline().decode("latin1", errors="replace").strip()
    ser.close()

    if not line.startswith("OK"):
        print(f"Unexpected response from flight controller: {line}", file=sys.stderr)
        sys.exit(1)

    kv = {}
    for part in line.split()[1:]:
        if "=" in part:
            k, v = part.split("=", 1)
            kv[k] = v

    jedec = kv.get("JEDEC", "856015")
    used_bytes = int(kv.get("USED_BYTES", 0))
    capacity = int(kv.get("CAPACITY", 2080768))
    wr_ptr = int(kv.get("WR_PTR", "0x004000"), 16)
    sessions = int(kv.get("SESSIONS", "1"))
    print_status_summary(f"USB CDC ({port})", jedec, wr_ptr, sessions, used_bytes, capacity)


def cmd_dump(args):
    if should_use_swd(args):
        mb_addr = get_swd_mailbox_addr(args)
        print(f"Initiating SWD binary dump via XIAO DebugMate (Mailbox @ 0x{mb_addr:08X})...")
        words = swd_read_words(mb_addr, 8)
        magic, write_ptr, capacity, sessions, status, _, _, _ = words
        if magic != SWD_MAILBOX_MAGIC:
            print(f"Error: SWD Mailbox magic mismatch (0x{magic:08X}). Is the target running?", file=sys.stderr)
            sys.exit(1)

        size = max(0, write_ptr - LOG_START_ADDR)
        print(f"Streaming {size:,} bytes of raw blackbox logs over SWD...")
        received = bytearray()
        start_time = time.time()

        addr = LOG_START_ADDR
        while addr < write_ptr:
            chunk_len = min(SWD_BUF_BYTES, write_ptr - addr)
            swd_write_words(mb_addr + 0x10, [SWD_STATUS_BUSY, SWD_CMD_READ_CHUNK, addr, chunk_len])

            for _ in range(50):
                st_words = swd_read_words(mb_addr + 0x10, 2)
                st_status, st_cmd = st_words[0], st_words[1]
                if st_cmd == SWD_CMD_NONE:
                    break
                time.sleep(0.01)

            if st_status == SWD_STATUS_ERR_ARMED:
                print("\nERROR: Aircraft is currently armed! Dump is locked out for flight safety.", file=sys.stderr)
                sys.exit(1)
            if st_status != SWD_STATUS_IDLE:
                print(f"\nERROR: QSPI read failed with status {st_status} at 0x{addr:06X}.", file=sys.stderr)
                sys.exit(1)

            word_count = (chunk_len + 3) // 4
            buf_words = swd_read_words(mb_addr + 0x20, word_count)
            chunk_bytes = bytearray()
            for w in buf_words:
                chunk_bytes.extend(struct.pack("<I", w))
            received.extend(chunk_bytes[:chunk_len])

            addr += chunk_len
            pct = (len(received) / size) * 100.0 if size > 0 else 100.0
            sys.stdout.write(f"\rProgress: {len(received):,}/{size:,} bytes ({pct:.1f}%)")
            sys.stdout.flush()

        print()
        calc_crc = binascii.crc32(received) & 0xFFFFFFFF
        elapsed = time.time() - start_time
        rate_kb = (len(received) / 1024.0) / elapsed if elapsed > 0 else 0
        print(f"SWD transfer complete (CRC-32: 0x{calc_crc:08X}) in {elapsed:.2f}s ({rate_kb:.1f} KB/s)")
        save_dumped_data(bytes(received), args.output, getattr(args, "raw", False))
        return

    port = args.port or find_wingfc_usb_port() or "/dev/ttyACM0"
    print(f"Initiating binary dump from {port}...")
    ser = open_serial(port, args.baud, timeout=5.0)

    ser.write(b"BBL_DUMP\r\n")

    line = ser.readline().decode("latin1", errors="replace").strip()
    if "ERR:ARMED" in line:
        print("ERROR: Aircraft is currently armed! USB dump is locked out for flight safety.", file=sys.stderr)
        ser.close()
        sys.exit(1)

    if not line.startswith("START_DUMP"):
        print(f"Error starting dump: {line}", file=sys.stderr)
        ser.close()
        sys.exit(1)

    size = 0
    for part in line.split():
        if part.startswith("SIZE="):
            size = int(part.split("=")[1])

    print(f"Streaming {size:,} bytes of raw blackbox logs...")
    received = bytearray()
    start_time = time.time()

    while len(received) < size:
        to_read = min(4096, size - len(received))
        chunk = ser.read(to_read)
        if not chunk:
            print("\nError: Timed out waiting for dump data stream.", file=sys.stderr)
            break
        received.extend(chunk)
        pct = (len(received) / size) * 100.0 if size > 0 else 100.0
        sys.stdout.write(f"\rProgress: {len(received):,}/{size:,} bytes ({pct:.1f}%)")
        sys.stdout.flush()

    print()
    footer = ser.readline().decode("latin1", errors="replace").strip()
    ser.close()

    expected_crc = None
    if "END_DUMP" in footer and "CRC32=" in footer:
        for part in footer.split():
            if part.startswith("CRC32="):
                expected_crc = int(part.split("=")[1], 16)

    calc_crc = binascii.crc32(received) & 0xFFFFFFFF
    if expected_crc is not None:
        if calc_crc == expected_crc:
            print(f"CRC-32 Check: PASS (0x{calc_crc:08X})")
        else:
            print(f"CRC-32 Check: MISMATCH (Calculated: 0x{calc_crc:08X}, Expected: 0x{expected_crc:08X})", file=sys.stderr)

    elapsed = time.time() - start_time
    rate_kb = (len(received) / 1024.0) / elapsed if elapsed > 0 else 0
    print(f"USB transfer complete in {elapsed:.2f}s ({rate_kb:.1f} KB/s)")
    save_dumped_data(bytes(received), args.output, getattr(args, "raw", False))


def cmd_convert_bbl(args):
    with open(args.input, "rb") as f:
        raw = f.read()

    save_dumped_data(raw, args.output, raw_flag=False)


def cmd_export_csv(args):
    print(f"Decoding log {args.input} to CSV {args.output}...")
    with open(args.input, "rb") as f:
        raw = f.read()

    frames = extract_frames_from_raw(raw)
    write_csv_frames(frames, args.output)
    print(f"Export complete: {len(frames):,} valid frames exported to {args.output}")


def cmd_erase(args):
    if should_use_swd(args):
        mb_addr = get_swd_mailbox_addr(args)
        if not args.yes:
            confirm = input("Are you sure you want to erase flight logs via XIAO DebugMate SWD? (y/N): ").strip().lower()
            if confirm != "y":
                print("Erase cancelled.")
                return

        words = swd_read_words(mb_addr, 8)
        if words[0] != SWD_MAILBOX_MAGIC:
            print(f"Error: SWD Mailbox magic mismatch (0x{words[0]:08X}).", file=sys.stderr)
            sys.exit(1)

        erase_cmd = SWD_CMD_ERASE_ALL if getattr(args, "full", False) else SWD_CMD_ERASE_USED
        print("Erasing blackbox flash sectors via XIAO DebugMate SWD...")
        swd_write_words(mb_addr + 0x10, [SWD_STATUS_BUSY, erase_cmd, 0, 0])

        while True:
            st_words = swd_read_words(mb_addr + 0x10, 4)
            st_status, st_cmd, done_sectors, total_sectors = st_words
            if st_status == SWD_STATUS_ERR_ARMED:
                print("\nERROR: Aircraft is armed! Flash erase blocked for flight safety.", file=sys.stderr)
                sys.exit(1)
            if total_sectors > 0:
                sys.stdout.write(f"\rErasing sectors: {done_sectors}/{total_sectors}...")
                sys.stdout.flush()
            if st_cmd == SWD_CMD_NONE and st_status == SWD_STATUS_IDLE:
                print("\nErase verified: Log sectors cleared and write pointer reset to 0x004000.")
                break
            time.sleep(0.05)
        return

    port = args.port or find_wingfc_usb_port() or "/dev/ttyACM0"
    if not args.yes:
        confirm = input(f"Are you sure you want to erase all flight logs on {port}? (y/N): ").strip().lower()
        if confirm != "y":
            print("Erase cancelled.")
            return

    print(f"Connecting to WingFC on {port}...")
    ser = open_serial(port, args.baud, timeout=10.0)

    ser.write(b"BBL_ERASE\r\n")

    while True:
        line = ser.readline().decode("latin1", errors="replace").strip()
        if not line:
            continue
        if "ERR:ARMED" in line:
            print("ERROR: Aircraft is armed! Flash erase blocked for flight safety.", file=sys.stderr)
            break
        if "ERASING" in line:
            sys.stdout.write(f"\r{line}...")
            sys.stdout.flush()
        elif "OK ERASED" in line:
            print("\nErase verified: All 508 log sectors cleared, write pointer reset.")
            break
        elif "ERR" in line:
            print(f"\nErase failed: {line}", file=sys.stderr)
            break

    ser.close()


def main():
    parser = argparse.ArgumentParser(description="WingFC Blackbox Log Offload & Analysis Tool")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # Info
    p_info = subparsers.add_parser("info", help="Inspect flash usage and session info")
    p_info.add_argument("--port", "-p", help="USB serial port (auto-detects USB or DebugMate SWD if omitted)")
    p_info.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default 115200)")
    p_info.add_argument("--swd", action="store_true", help="Force XIAO DebugMate SWD (probe-rs) transport")
    p_info.add_argument("--elf", help="Optional path to wingfc-firmware ELF for SWD symbol lookup")

    # Dump
    p_dump = subparsers.add_parser("dump", help="Download raw QSPI blackbox log and convert to .bbl and .csv simultaneously")
    p_dump.add_argument("output", nargs="?", default="flight", help="Output base name or path (default: 'flight' -> generates flight.bbl and flight.csv)")
    p_dump.add_argument("--port", "-p", help="USB serial port (auto-detects USB or DebugMate SWD if omitted)")
    p_dump.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default 115200)")
    p_dump.add_argument("--swd", action="store_true", help="Force XIAO DebugMate SWD (probe-rs) transport")
    p_dump.add_argument("--raw", action="store_true", help="Also save raw 32-byte MCU flash frames (.bin)")
    p_dump.add_argument("--elf", help="Optional path to wingfc-firmware ELF for SWD symbol lookup")

    # Convert raw dump to Betaflight .bbl + .csv
    p_conv = subparsers.add_parser("convert-bbl", help="Convert raw WingFC binary dump into Betaflight .bbl and .csv")
    p_conv.add_argument("input", help="Input raw dump path")
    p_conv.add_argument("output", help="Output base path or .bbl path")

    # Export CSV
    p_csv = subparsers.add_parser("export-csv", help="Decode raw binary or .bbl log into CSV format")
    p_csv.add_argument("input", help="Input log path (.bin or .bbl)")
    p_csv.add_argument("output", help="Output CSV file path (.csv)")

    # Erase
    p_erase = subparsers.add_parser("erase", help="Erase log storage before flight")
    p_erase.add_argument("--port", "-p", help="USB serial port (auto-detects USB or DebugMate SWD if omitted)")
    p_erase.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default 115200)")
    p_erase.add_argument("--swd", action="store_true", help="Force XIAO DebugMate SWD (probe-rs) transport")
    p_erase.add_argument("--full", action="store_true", help="Erase all 508 sectors (2 MB) instead of used sectors")
    p_erase.add_argument("--elf", help="Optional path to wingfc-firmware ELF for SWD symbol lookup")
    p_erase.add_argument("--yes", "-y", action="store_true", help="Skip confirmation prompt")

    args = parser.parse_args()

    if args.command == "info":
        cmd_info(args)
    elif args.command == "dump":
        cmd_dump(args)
    elif args.command == "convert-bbl":
        cmd_convert_bbl(args)
    elif args.command == "export-csv":
        cmd_export_csv(args)
    elif args.command == "erase":
        cmd_erase(args)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Decode BERT26 VCU binary session logs (LOGnnnn.BIN) into CSV files.

Usage:
    python decode_vcu_log.py LOG0001.BIN
    python decode_vcu_log.py --hex dump_capture.txt

The first form reads a binary file copied from the SD card. The second form
reads a text file containing a serial capture of the VCU 'd<n>' dump command
(everything between "[Log] DUMP BEGIN" and "[Log] DUMP END").

Outputs <input>_fast.csv, <input>_medium.csv and <input>_slow.csv next to the
input file. Record layouts must match the structs in VCU_BERT26.ino.
"""

import argparse
import csv
import os
import struct
import sys

MAGIC = b"VCULOG01"

HEADER_FMT = "<8sIBBBB"
FAST_FMT = "<BIHHHHHhhhhH"
MEDIUM_FMT = "<BIhhhhhhHhhBBBBHIIBB"
SLOW_FMT = "<BIHHHhhhHBB"

RECORD_FAST = 0x01
RECORD_MEDIUM = 0x02
RECORD_SLOW = 0x03

STATUS_FLAG_NAMES = [
    "braking", "hard_braking", "bspc_active", "apps_plausible",
    "rtd_latched", "drive_mode_d", "bms_fault", "imd_fault",
    "pm100_fault", "bms_timeout", "imd_can_timeout", "contactors_closed",
    "bspd_gpio_ok", "imd_gpio_ok", "bms_gpio_ok",
]

FAST_HEADER = [
    "t_ms", "raw_apps1", "raw_apps2", "raw_bse1", "raw_bse2",
    "raw_hv_current", "commanded_torque_nm", "motor_rpm",
    "bus_voltage_v", "bus_current_a",
] + STATUS_FLAG_NAMES

MEDIUM_HEADER = [
    "t_ms", "apps1_pct", "apps2_pct", "pedal_pct", "desired_torque_nm",
    "power_kw", "speed_mph", "glv_voltage_v", "inverter_temp_c",
    "motor_temp_c", "soc_pct", "battery_temp_c", "bms_flags_hex",
    "bms_fault_code", "imd_warn_alarms_hex", "pm100_post_faults_hex",
    "pm100_run_faults_hex", "fan_duty_pct", "rtd_state",
]

SLOW_HEADER = [
    "t_ms", "min_cell_mv", "max_cell_mv", "avg_cell_mv",
    "min_cell_temp_c", "max_cell_temp_c", "avg_cell_temp_c",
    "imd_riso_kohm", "imd_riso_status",
    "cell_v_valid", "cell_t_valid", "glv_valid",
]

RTD_STATE_NAMES = {0: "FAULT_BLOCKED", 1: "READY_FOR_BUTTON", 2: "READY_TO_DRIVE"}


def read_hex_dump(path):
    """Extract the binary payload from a serial capture of the dump command."""
    data = bytearray()
    in_dump = False
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if "DUMP BEGIN" in line:
                in_dump = True
                continue
            if "DUMP END" in line:
                break
            if not in_dump or not line or line.startswith("["):
                continue
            try:
                data.extend(bytes.fromhex(line))
            except ValueError:
                pass  # skip interleaved log lines
    return bytes(data)


def decode_flags(flags):
    return [int(bool(flags & (1 << bit))) for bit in range(len(STATUS_FLAG_NAMES))]


def decode(data, base):
    header_size = struct.calcsize(HEADER_FMT)
    if len(data) < header_size:
        sys.exit("File too short for header")

    magic, session, fast_size, medium_size, slow_size, _ = struct.unpack_from(
        HEADER_FMT, data, 0)
    if magic != MAGIC:
        sys.exit(f"Bad magic {magic!r}; not a VCU log file")

    sizes = {RECORD_FAST: fast_size, RECORD_MEDIUM: medium_size,
             RECORD_SLOW: slow_size}
    for rec_type, fmt in ((RECORD_FAST, FAST_FMT), (RECORD_MEDIUM, MEDIUM_FMT),
                          (RECORD_SLOW, SLOW_FMT)):
        if sizes[rec_type] != struct.calcsize(fmt):
            sys.exit(f"Record size mismatch for type {rec_type}: file says "
                     f"{sizes[rec_type]}, decoder expects {struct.calcsize(fmt)}. "
                     "Firmware and decoder are out of sync.")

    fast_rows, medium_rows, slow_rows = [], [], []
    offset = header_size
    truncated = False

    while offset < len(data):
        rec_type = data[offset]
        size = sizes.get(rec_type)
        if size is None:
            sys.exit(f"Unknown record type 0x{rec_type:02X} at offset {offset}")
        if offset + size > len(data):
            truncated = True  # power was cut mid-record; drop the tail
            break

        if rec_type == RECORD_FAST:
            (_, ms, a1, a2, b1, b2, hv, trq, rpm, busv, busi,
             flags) = struct.unpack_from(FAST_FMT, data, offset)
            fast_rows.append([ms, a1, a2, b1, b2, hv, trq / 10.0, rpm,
                              busv / 10.0, busi / 10.0] + decode_flags(flags))
        elif rec_type == RECORD_MEDIUM:
            (_, ms, ap1, ap2, ped, des, pwr, mph, glv, invt, mtrt, soc,
             batt, bmsf, code, warn, post, run, fan,
             rtd) = struct.unpack_from(MEDIUM_FMT, data, offset)
            medium_rows.append([
                ms, ap1 / 10.0, ap2 / 10.0, ped / 10.0, des / 10.0,
                pwr / 10.0, mph, glv / 100.0, invt / 10.0, mtrt / 10.0,
                soc, batt, f"0x{bmsf:02X}", code, f"0x{warn:04X}",
                f"0x{post:08X}", f"0x{run:08X}", fan,
                RTD_STATE_NAMES.get(rtd, str(rtd)),
            ])
        else:
            (_, ms, vmin, vmax, vavg, tmin, tmax, tavg, riso, rstat,
             valid) = struct.unpack_from(SLOW_FMT, data, offset)
            slow_rows.append([ms, vmin, vmax, vavg, tmin, tmax, tavg, riso,
                              rstat, valid & 1, (valid >> 1) & 1,
                              (valid >> 2) & 1])
        offset += size

    for suffix, header, rows in (("fast", FAST_HEADER, fast_rows),
                                 ("medium", MEDIUM_HEADER, medium_rows),
                                 ("slow", SLOW_HEADER, slow_rows)):
        out = f"{base}_{suffix}.csv"
        with open(out, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)
        print(f"Wrote {out} ({len(rows)} rows)")

    print(f"Session {session}" + (" (file truncated at power-off)" if truncated else ""))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="LOGnnnn.BIN file or serial dump text")
    parser.add_argument("--hex", action="store_true",
                        help="input is a text capture of the serial 'd<n>' dump")
    args = parser.parse_args()

    if args.hex:
        data = read_hex_dump(args.input)
        if not data:
            sys.exit("No hex dump payload found in input")
    else:
        with open(args.input, "rb") as f:
            data = f.read()

    base = os.path.splitext(args.input)[0]
    decode(data, base)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Convert BERT26 VCU binary log files to CSV.

The VCU writes four files per logging session:

* MEASnnnn.BIN - measured fast, medium, slow, and IMU records
* DERInnnn.BIN - derived fast and medium records
* CELLnnnn.BIN - individual cell voltage and temperature records
* INVnnnn.BIN - raw Cascadia PM100DX inverter CAN frames

Only the Python standard library is required.  Examples:

    python vcu_bin_to_csv.py bert26-logs-20260912
    python vcu_bin_to_csv.py MEAS0223.BIN DERI0223.BIN CELL0223.BIN INV0223.BIN
    python vcu_bin_to_csv.py logs --output-dir converted --force
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Dict, Iterable, List, Mapping, Optional, Sequence, TextIO, Tuple


LOG_HEADER = struct.Struct("<8sIBBBB")
CELL_HEADER = struct.Struct("<8sIHH")
INVERTER_HEADER = struct.Struct("<8sIHH")
INVERTER_RECORD = struct.Struct("<fHBB8s")
FLOAT = struct.Struct("<f")

MEASURED_MAGIC = b"VCUMEAS1"
DERIVED_MAGIC = b"VCUDERI1"
CELL_MAGIC = b"VCUCELL1"
INVERTER_MAGIC = b"VCUINV01"


INVERTER_MESSAGE_NAMES: Mapping[int, str] = {
    0x0A0: "temperatures_1",
    0x0A1: "temperatures_2",
    0x0A2: "temperatures_3",
    0x0A3: "analog_input_voltages",
    0x0A4: "digital_input_status",
    0x0A5: "motor_position_information",
    0x0A6: "current_information",
    0x0A7: "voltage_information",
    0x0A8: "flux_information",
    0x0A9: "internal_voltages",
    0x0AA: "internal_states",
    0x0AB: "fault_codes",
    0x0AC: "torque_and_timer_information",
    0x0AD: "modulation_index_and_flux_weakening",
    0x0AE: "firmware_information",
    0x0AF: "diagnostic_data",
    0x0B0: "high_speed_message",
    0x0B1: "torque_capability",
}


class ConversionError(Exception):
    """Raised when an input is not a supported, valid VCU log."""


@dataclass(frozen=True)
class RecordDefinition:
    name: str
    type_id: int
    fields: Tuple[str, ...]

    @property
    def size(self) -> int:
        return 4 * (2 + len(self.fields))


MEASURED_RECORDS: Mapping[int, RecordDefinition] = {
    1: RecordDefinition(
        "fast",
        1,
        (
            "apps1_adc_counts",
            "apps2_adc_counts",
            "bse1_adc_counts",
            "bse2_adc_counts",
            "hv_current_adc_counts",
            "motor_rpm",
            "bus_voltage_v",
            "bus_current_a",
        ),
    ),
    2: RecordDefinition(
        "medium",
        2,
        (
            "glv_voltage_v",
            "inverter_temp_c",
            "motor_temp_c",
            "soc_pct",
            "battery_temp_c",
            "bms_flags",
            "bms_fault_code",
            "imd_warn_alarms",
            "pm100_post_faults",
            "pm100_run_faults",
        ),
    ),
    3: RecordDefinition(
        "slow",
        3,
        (
            "min_cell_voltage_v",
            "max_cell_voltage_v",
            "avg_cell_voltage_v",
            "min_cell_temp_c",
            "max_cell_temp_c",
            "avg_cell_temp_c",
            "imd_riso_kohm",
            "imd_riso_status",
            "valid_flags",
        ),
    ),
    4: RecordDefinition(
        "imu",
        4,
        (
            "accel_x_g",
            "accel_y_g",
            "accel_z_g",
            "gyro_x_dps",
            "gyro_y_dps",
            "gyro_z_dps",
            "imu_temperature_c",
        ),
    ),
}

DERIVED_RECORDS: Mapping[int, RecordDefinition] = {
    1: RecordDefinition(
        "fast",
        1,
        (
            "brake_pressure_pct",
            "commanded_torque_nm",
            "regen_torque_nm",
            "speed_kmh",
            "cumulative_distance_km",
            "status_flags",
        ),
    ),
    2: RecordDefinition(
        "medium",
        2,
        (
            "apps1_pct",
            "apps2_pct",
            "pedal_pct",
            "desired_torque_nm",
            "power_kw",
            "speed_mph",
            "fan_duty_pct",
            "rtd_state",
        ),
    ),
}

CELL_RECORD = RecordDefinition("cell", 0, ("cell_index", "value"))
CELL_TYPE_NAMES = {1: ("voltage", "V"), 2: ("temperature", "degC")}


def _read_exact(file: BinaryIO, size: int, description: str) -> bytes:
    data = file.read(size)
    if len(data) != size:
        raise ConversionError(
            f"truncated {description}: expected {size} bytes, found {len(data)}"
        )
    return data


def _union_fields(records: Mapping[int, RecordDefinition]) -> List[str]:
    fields: List[str] = []
    for definition in records.values():
        for field in definition.fields:
            if field not in fields:
                fields.append(field)
    return fields


def _format_float(value: float) -> str:
    """Use enough significant digits to reproduce an IEEE-754 float32."""
    if math.isnan(value):
        return "nan"
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return format(value, ".9g")


def _format_time(value: float) -> str:
    """Remove float32 noise from timestamps originating as integer milliseconds."""
    return f"{value:.3f}".rstrip("0").rstrip(".") or "0"


def _validate_record_sizes(
    path: Path,
    declared_sizes: Sequence[int],
    records: Mapping[int, RecordDefinition],
) -> None:
    type_labels = ("fast", "medium", "slow", "imu")
    for type_id, (label, declared) in enumerate(zip(type_labels, declared_sizes), start=1):
        definition = records.get(type_id)
        expected = definition.size if definition is not None else 0
        if declared != expected:
            raise ConversionError(
                f"{path}: unsupported {label} record size {declared}; expected {expected}"
            )


def _record_type(raw_type: float, path: Path, offset: int) -> int:
    if not raw_type.is_integer():
        raise ConversionError(
            f"{path}: invalid record type {raw_type!r} at byte offset {offset}"
        )
    return int(raw_type)


def _convert_mixed_log(
    source: BinaryIO,
    destination: TextIO,
    path: Path,
    magic: bytes,
    records: Mapping[int, RecordDefinition],
) -> Tuple[int, int, Optional[str]]:
    header = _read_exact(source, LOG_HEADER.size, "log header")
    actual_magic, session, fast_size, medium_size, slow_size, imu_size = LOG_HEADER.unpack(
        header
    )
    if actual_magic != magic:
        raise ConversionError(
            f"{path}: bad magic {actual_magic!r}; expected {magic!r}"
        )

    _validate_record_sizes(
        path, (fast_size, medium_size, slow_size, imu_size), records
    )

    fieldnames = ["session", "record_type", "record_type_id", "time_seconds"]
    fieldnames.extend(_union_fields(records))
    writer = csv.DictWriter(destination, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()

    count = 0
    trailing_bytes = 0
    warning: Optional[str] = None
    while True:
        offset = source.tell()
        raw_type_bytes = source.read(FLOAT.size)
        if not raw_type_bytes:
            break
        if len(raw_type_bytes) != FLOAT.size:
            trailing_bytes = len(raw_type_bytes)
            warning = f"ignored {trailing_bytes} trailing byte(s) after the last complete record"
            break

        raw_type = FLOAT.unpack(raw_type_bytes)[0]
        type_id = _record_type(raw_type, path, offset)
        definition = records.get(type_id)
        if definition is None:
            raise ConversionError(
                f"{path}: unknown record type {type_id} at byte offset {offset}"
            )

        remainder_size = definition.size - FLOAT.size
        remainder = source.read(remainder_size)
        if len(remainder) != remainder_size:
            trailing_bytes = FLOAT.size + len(remainder)
            warning = (
                f"ignored a truncated {definition.name} record "
                f"({trailing_bytes} of {definition.size} bytes) at byte offset {offset}"
            )
            break

        values = struct.unpack(f"<{1 + len(definition.fields)}f", remainder)
        row: Dict[str, object] = {
            "session": session,
            "record_type": definition.name,
            "record_type_id": type_id,
            "time_seconds": _format_time(values[0]),
        }
        row.update(
            (field, _format_float(value))
            for field, value in zip(definition.fields, values[1:])
        )
        writer.writerow(row)
        count += 1

    return session, count, warning


def _convert_cell_log(
    source: BinaryIO, destination: TextIO, path: Path
) -> Tuple[int, int, Optional[str]]:
    header = _read_exact(source, CELL_HEADER.size, "cell log header")
    actual_magic, session, record_size, cell_count = CELL_HEADER.unpack(header)
    if actual_magic != CELL_MAGIC:
        raise ConversionError(
            f"{path}: bad magic {actual_magic!r}; expected {CELL_MAGIC!r}"
        )
    if record_size != CELL_RECORD.size:
        raise ConversionError(
            f"{path}: unsupported cell record size {record_size}; "
            f"expected {CELL_RECORD.size}"
        )

    fieldnames = [
        "session",
        "record_type",
        "record_type_id",
        "time_seconds",
        "cell_index",
        "value",
        "unit",
        "cell_count",
    ]
    writer = csv.DictWriter(destination, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()

    unpack_record = struct.Struct("<4f").unpack
    count = 0
    warning: Optional[str] = None
    while True:
        offset = source.tell()
        data = source.read(record_size)
        if not data:
            break
        if len(data) != record_size:
            warning = (
                f"ignored a truncated cell record ({len(data)} of {record_size} bytes) "
                f"at byte offset {offset}"
            )
            break

        raw_type, time_seconds, cell_index, value = unpack_record(data)
        type_id = _record_type(raw_type, path, offset)
        try:
            type_name, unit = CELL_TYPE_NAMES[type_id]
        except KeyError as error:
            raise ConversionError(
                f"{path}: unknown cell record type {type_id} at byte offset {offset}"
            ) from error

        writer.writerow(
            {
                "session": session,
                "record_type": type_name,
                "record_type_id": type_id,
                "time_seconds": _format_time(time_seconds),
                "cell_index": _format_float(cell_index),
                "value": _format_float(value),
                "unit": unit,
                "cell_count": cell_count,
            }
        )
        count += 1

    return session, count, warning


def _convert_inverter_log(
    source: BinaryIO, destination: TextIO, path: Path
) -> Tuple[int, int, Optional[str]]:
    header = _read_exact(source, INVERTER_HEADER.size, "inverter log header")
    actual_magic, session, record_size, header_reserved = INVERTER_HEADER.unpack(header)
    if actual_magic != INVERTER_MAGIC:
        raise ConversionError(
            f"{path}: bad magic {actual_magic!r}; expected {INVERTER_MAGIC!r}"
        )
    if record_size != INVERTER_RECORD.size:
        raise ConversionError(
            f"{path}: unsupported inverter record size {record_size}; "
            f"expected {INVERTER_RECORD.size}"
        )

    fieldnames = [
        "session",
        "time_seconds",
        "can_id",
        "can_id_hex",
        "message_name",
        "dlc",
        "flags",
        "header_reserved",
        "payload_hex",
    ]
    fieldnames.extend(f"data_{index}" for index in range(8))
    writer = csv.DictWriter(destination, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()

    count = 0
    warning: Optional[str] = None
    while True:
        offset = source.tell()
        data = source.read(record_size)
        if not data:
            break
        if len(data) != record_size:
            warning = (
                f"ignored a truncated inverter record ({len(data)} of {record_size} "
                f"bytes) at byte offset {offset}"
            )
            break

        time_seconds, can_id, dlc, flags, payload = INVERTER_RECORD.unpack(data)
        if dlc > len(payload):
            raise ConversionError(
                f"{path}: invalid inverter CAN DLC {dlc} at byte offset {offset}"
            )

        active_payload = payload[:dlc]
        row: Dict[str, object] = {
            "session": session,
            "time_seconds": _format_time(time_seconds),
            "can_id": can_id,
            "can_id_hex": f"0x{can_id:03X}",
            "message_name": INVERTER_MESSAGE_NAMES.get(can_id, "unknown"),
            "dlc": dlc,
            "flags": flags,
            "header_reserved": header_reserved,
            "payload_hex": active_payload.hex(" ").upper(),
        }
        row.update(
            (f"data_{index}", f"0x{byte:02X}")
            for index, byte in enumerate(active_payload)
        )
        writer.writerow(row)
        count += 1

    return session, count, warning


def convert_file(
    source_path: Path, output_path: Path, force: bool = False
) -> Tuple[int, int, Optional[str]]:
    """Convert one VCU BIN file and return (session, record_count, warning)."""
    if output_path.exists() and not force:
        raise ConversionError(f"{output_path}: output already exists (use --force to replace it)")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(output_path.name + ".tmp")

    try:
        with source_path.open("rb") as source:
            magic = _read_exact(source, 8, "magic")
            source.seek(0)
            with temporary_path.open("w", newline="", encoding="utf-8") as destination:
                if magic == MEASURED_MAGIC:
                    result = _convert_mixed_log(
                        source, destination, source_path, MEASURED_MAGIC, MEASURED_RECORDS
                    )
                elif magic == DERIVED_MAGIC:
                    result = _convert_mixed_log(
                        source, destination, source_path, DERIVED_MAGIC, DERIVED_RECORDS
                    )
                elif magic == CELL_MAGIC:
                    result = _convert_cell_log(source, destination, source_path)
                elif magic == INVERTER_MAGIC:
                    result = _convert_inverter_log(source, destination, source_path)
                else:
                    raise ConversionError(
                        f"{source_path}: unsupported VCU log magic {magic!r}"
                    )
        os.replace(temporary_path, output_path)
        return result
    except Exception:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass
        raise


def _find_inputs(paths: Iterable[Path], recursive: bool) -> List[Path]:
    inputs: List[Path] = []
    seen = set()
    for path in paths:
        if path.is_file():
            candidates = [path]
        elif path.is_dir():
            candidates = path.rglob("*") if recursive else path.iterdir()
        else:
            raise ConversionError(f"{path}: input does not exist")

        for candidate in candidates:
            if not candidate.is_file() or candidate.suffix.lower() != ".bin":
                continue
            resolved = candidate.resolve()
            if resolved not in seen:
                inputs.append(candidate)
                seen.add(resolved)

    return sorted(inputs, key=lambda item: str(item).lower())


def _output_path(source: Path, output_dir: Optional[Path]) -> Path:
    if output_dir is None:
        return source.with_suffix(".csv")
    return output_dir / source.with_suffix(".csv").name


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert BERT26 VCU MEAS, DERI, CELL, and INV binary logs to CSV.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python vcu_bin_to_csv.py bert26-logs-20260912\n"
            "  python vcu_bin_to_csv.py MEAS0223.BIN DERI0223.BIN CELL0223.BIN INV0223.BIN\n"
            "  python vcu_bin_to_csv.py logs -o converted --force"
        ),
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="BIN files or directories containing BIN files",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        help="write all CSV files to this directory (default: beside each input)",
    )
    parser.add_argument(
        "-r",
        "--recursive",
        action="store_true",
        help="search input directories recursively",
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="replace CSV files that already exist",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        inputs = _find_inputs(args.inputs, args.recursive)
    except ConversionError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    if not inputs:
        print("error: no .BIN files found", file=sys.stderr)
        return 1

    failed = 0
    outputs_seen = set()
    for source_path in inputs:
        output_path = _output_path(source_path, args.output_dir)
        resolved_output = output_path.resolve()
        if resolved_output in outputs_seen:
            print(
                f"error: multiple inputs map to the same output file: {output_path}",
                file=sys.stderr,
            )
            failed += 1
            continue
        outputs_seen.add(resolved_output)

        try:
            session, count, warning = convert_file(
                source_path, output_path, force=args.force
            )
        except (ConversionError, OSError, struct.error) as error:
            print(f"error: {error}", file=sys.stderr)
            failed += 1
            continue

        print(
            f"Converted {source_path} -> {output_path} "
            f"(session {session}, {count:,} records)"
        )
        if warning:
            print(f"warning: {source_path}: {warning}", file=sys.stderr)

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

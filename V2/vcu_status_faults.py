"""Load and visualize BERT26 VCU status and fault logs.

This module backs ``vcu_status_fault_plotter.ipynb``.  The bit names mirror the
definitions in ``VCU_BERT26_V2.ino`` so that plots show named boolean lanes
instead of raw integer words.
"""

from __future__ import annotations

import importlib.util
import math
import re
import sys
import warnings
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


STATUS_FLAGS = {
    0: "Braking",
    1: "Hard braking",
    2: "BSPC active",
    3: "APPS plausible",
    4: "Ready-to-drive latched",
    5: "Drive mode D",
    6: "BMS fault",
    7: "IMD fault",
    8: "PM100 fault",
    9: "BMS timed out",
    10: "IMD CAN timed out",
    11: "Contactors closed",
    12: "BSPD shutdown input OK",
    13: "IMD shutdown input OK",
    14: "BMS shutdown input OK",
}

BMS_FLAGS = {
    0: "BMS fault flag",
    1: "Positive contactor closed",
    2: "Charge mode",
}

VALID_FLAGS = {
    0: "Cell-voltage stats valid",
    1: "Cell-temperature stats valid",
    2: "GLV voltage valid",
}

IMD_WARN_ALARMS = {
    0: "Device error",
    1: "HV+ connection failure",
    2: "HV- connection failure",
    3: "Earth connection failure",
    4: "Isolation alarm",
    5: "Isolation warning",
    6: "Isolation value outdated",
    7: "Unbalance alarm",
    8: "Undervoltage",
    9: "Unsafe to start",
    10: "Earthlift open",
}

PM100_POST_FAULTS = {
    0: "Gate/desaturation",
    1: "Hardware overcurrent",
    2: "Accelerator shorted",
    3: "Accelerator open",
    4: "Current sensor low",
    5: "Current sensor high",
    6: "Module temperature low",
    7: "Module temperature high",
    8: "Control-board temperature low",
    9: "Control-board temperature high",
    10: "Gate-board temperature low",
    11: "Gate-board temperature high",
    12: "5 V supply low",
    13: "5 V supply high",
    14: "12 V supply low",
    15: "12 V supply high",
    16: "2.5 V supply low",
    17: "2.5 V supply high",
    18: "1.5 V supply low",
    19: "1.5 V supply high",
    20: "DC bus high",
    21: "DC bus low",
    22: "Precharge timeout",
    23: "Precharge voltage failure",
    24: "EEPROM checksum",
    25: "EEPROM data out of range",
    26: "EEPROM update required",
    27: "Hardware bus overvoltage",
    28: "Gate-driver initialization",
    29: "Reserved bit 29",
    30: "Brake input shorted",
    31: "Brake input open",
}

PM100_RUN_FAULTS = {
    0: "Motor overspeed",
    1: "Inverter overcurrent",
    2: "Inverter overvoltage",
    3: "Inverter overtemperature",
    4: "Accelerator shorted",
    5: "Accelerator open",
    6: "Direction-command fault",
    7: "Inverter-response timeout",
    8: "Gate/desaturation",
    9: "Hardware overcurrent",
    10: "Undervoltage",
    11: "CAN command lost",
    12: "Motor overtemperature",
    13: "Reserved bit 13",
    14: "Reserved bit 14",
    15: "Reserved bit 15",
    16: "Brake input shorted",
    17: "Brake input open",
    18: "Module A overtemperature",
    19: "Module B overtemperature",
    20: "Module C overtemperature",
    21: "PCB overtemperature",
    22: "Gate 1 overtemperature",
    23: "Gate 2 overtemperature",
    24: "Gate 3 overtemperature",
    25: "Current-sensor fault",
    26: "Gate overvoltage",
    27: "Hardware bus overvoltage Gen3",
    28: "Hardware bus overvoltage Gen5",
    29: "Reserved bit 29",
    30: "Resolver disconnected",
    31: "Reserved bit 31",
}

BMS_FAULT_CODES = {
    0: "No fault",
    1: "Cell undervoltage",
    2: "Cell overvoltage",
    3: "Pack undervoltage",
    4: "Pack overvoltage",
    5: "Overtemperature",
    6: "Cell spread",
    7: "Precharge timeout",
    8: "Precharge too fast",
    9: "Pack-voltage sense",
    10: "Charger communications",
    11: "BQ initialization failure",
    12: "BQ communications failure",
}

RTD_STATES = {
    0: "Fault blocked",
    1: "Ready for button",
    2: "Ready to drive",
}

IMD_RISO_STATES = {
    0: "Not received / unknown",
    0xFC: "Startup estimated",
    0xFD: "Startup first measurement",
    0xFE: "Normal operation",
    0xFF: "SNV",
}

_RUN_ID_RE = re.compile(r"(\d+)(?!.*\d)")


def normalize_run_id(run_id) -> Tuple[int, str]:
    """Return the numeric session and filename tag for a run identifier."""
    match = _RUN_ID_RE.search(str(run_id))
    if match is None:
        raise ValueError(f"RUN_ID must contain a number, got {run_id!r}")
    number = int(match.group(1))
    if number < 0:
        raise ValueError("RUN_ID cannot be negative")
    return number, f"{number:04d}"


def _resolve(path) -> Path:
    path = Path(path).expanduser()
    return path.resolve() if path.is_absolute() else (Path.cwd() / path).resolve()


def _case_insensitive_file(directory: Path, filename: str) -> Optional[Path]:
    if not directory.is_dir():
        return None
    wanted = filename.casefold()
    matches = [path for path in directory.iterdir() if path.is_file() and path.name.casefold() == wanted]
    if len(matches) > 1:
        raise ValueError(f"More than one file matches {filename} in {directory}")
    return matches[0] if matches else None


def _load_converter(converter_path: Path):
    if not converter_path.is_file():
        raise FileNotFoundError(f"Binary converter not found: {converter_path}")
    spec = importlib.util.spec_from_file_location("vcu_bin_to_csv_local", converter_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not import binary converter: {converter_path}")
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolves annotation metadata through sys.modules while the
    # converter module is executing.
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(spec.name, None)
        raise
    return module


def _ensure_csv(
    prefix: str,
    tag: str,
    raw_log_dir: Path,
    csv_log_dir: Path,
    auto_convert: bool,
    converter_path: Path,
) -> Path:
    filename = f"{prefix}{tag}.csv"
    existing = _case_insensitive_file(csv_log_dir, filename)
    if existing is not None:
        return existing
    if not auto_convert:
        raise FileNotFoundError(f"Decoded log not found: {csv_log_dir / filename}")

    binary_name = f"{prefix}{tag}.BIN"
    binary = _case_insensitive_file(raw_log_dir, binary_name)
    if binary is None:
        raise FileNotFoundError(
            f"Neither {csv_log_dir / filename} nor {raw_log_dir / binary_name} exists"
        )
    csv_log_dir.mkdir(parents=True, exist_ok=True)
    destination = csv_log_dir / filename
    converter = _load_converter(converter_path)
    converter.convert_file(binary, destination, force=False)
    print(f"Converted {binary.name} -> {destination}")
    return destination


def load_run(
    run_id,
    raw_log_dir="bert26-logs-20260912",
    csv_log_dir="converted",
    auto_convert=True,
    converter_path="vcu_bin_to_csv.py",
):
    """Load the MEAS and DERI logs for one run, converting BIN files if needed."""
    number, tag = normalize_run_id(run_id)
    raw_dir = _resolve(raw_log_dir)
    csv_dir = _resolve(csv_log_dir)
    converter = _resolve(converter_path)
    paths = {
        prefix: _ensure_csv(prefix, tag, raw_dir, csv_dir, auto_convert, converter)
        for prefix in ("MEAS", "DERI")
    }
    frames = {source: pd.read_csv(path, low_memory=False) for source, path in paths.items()}

    starts = []
    for source, frame in frames.items():
        if "time_seconds" not in frame:
            raise ValueError(f"{paths[source].name} has no time_seconds column")
        sessions = pd.to_numeric(frame.get("session"), errors="coerce").dropna().astype(int).unique()
        if len(sessions) and (len(sessions) != 1 or sessions[0] != number):
            raise ValueError(
                f"{paths[source].name} contains session(s) {sessions.tolist()}, expected {number}"
            )
        times = pd.to_numeric(frame["time_seconds"], errors="coerce").dropna()
        if not times.empty:
            starts.append(float(times.min()))

    if not starts:
        raise ValueError(f"Run {tag} has no valid timestamps")
    time_origin = min(starts)
    for frame in frames.values():
        frame["elapsed_s"] = pd.to_numeric(frame["time_seconds"], errors="coerce") - time_origin

    duration = max(
        pd.to_numeric(frame["elapsed_s"], errors="coerce").max() for frame in frames.values()
    )
    return {
        "number": number,
        "tag": tag,
        "id": f"RUN{tag}",
        "paths": paths,
        "data": frames,
        "time_origin_s": time_origin,
        "duration_s": float(duration),
    }


def run_summary(run) -> pd.DataFrame:
    rows = []
    for source, frame in run["data"].items():
        elapsed = pd.to_numeric(frame["elapsed_s"], errors="coerce").dropna()
        rows.append(
            {
                "source": source,
                "file": run["paths"][source].name,
                "rows": len(frame),
                "first_elapsed_s": elapsed.min() if len(elapsed) else math.nan,
                "last_elapsed_s": elapsed.max() if len(elapsed) else math.nan,
            }
        )
    return pd.DataFrame(rows)


def _validate_time_range(time_range_s):
    if time_range_s is None:
        return
    if len(time_range_s) != 2 or time_range_s[0] > time_range_s[1]:
        raise ValueError("TIME_RANGE_S must be None or (start_seconds, end_seconds)")


def _signal_rows(frame: pd.DataFrame, column: str, time_range_s=None) -> pd.DataFrame:
    _validate_time_range(time_range_s)
    if column not in frame:
        raise KeyError(f"Missing required column: {column}")
    selected = pd.DataFrame(
        {
            "elapsed_s": pd.to_numeric(frame["elapsed_s"], errors="coerce"),
            column: pd.to_numeric(frame[column], errors="coerce"),
        }
    ).dropna()
    selected = selected.sort_values("elapsed_s").drop_duplicates("elapsed_s", keep="last").reset_index(drop=True)
    if time_range_s is None or selected.empty:
        return selected

    # Carry the last known state to the exact plot boundaries. This keeps a
    # fault that began before a zoomed range visible from the left edge and
    # makes interval durations within the range exact.
    requested_start, requested_end = map(float, time_range_s)
    data_start = float(selected["elapsed_s"].iloc[0])
    data_end = float(selected["elapsed_s"].iloc[-1])
    start = max(requested_start, data_start)
    end = min(requested_end, data_end)
    if start > end:
        return selected.iloc[0:0].copy()

    prior = selected.loc[selected["elapsed_s"] <= start].tail(1).copy()
    prior.loc[:, "elapsed_s"] = start
    middle = selected.loc[selected["elapsed_s"].between(start, end, inclusive="right")]
    clipped = pd.concat([prior, middle], ignore_index=True)
    clipped = clipped.drop_duplicates("elapsed_s", keep="last")
    if not clipped.empty and clipped["elapsed_s"].iloc[-1] < end:
        final = clipped.tail(1).copy()
        final.loc[:, "elapsed_s"] = end
        clipped = pd.concat([clipped, final], ignore_index=True)
    return clipped.reset_index(drop=True)


def decode_bitfield(
    frame: pd.DataFrame,
    column: str,
    bit_names: Mapping[int, str],
    time_range_s=None,
) -> pd.DataFrame:
    """Decode an integer-valued CSV column into named boolean lanes."""
    selected = _signal_rows(frame, column, time_range_s)
    decoded = pd.DataFrame({"elapsed_s": selected["elapsed_s"]})
    if selected.empty:
        for name in bit_names.values():
            decoded[name] = pd.Series(dtype=bool)
        return decoded

    raw = selected[column].to_numpy(dtype=float)
    rounded = np.rint(raw)
    if not np.allclose(raw, rounded, rtol=0, atol=1e-5):
        warnings.warn(f"{column} contains non-integer values; values were rounded")
    if np.any(rounded < 0) or np.any(rounded > np.iinfo(np.uint32).max):
        raise ValueError(f"{column} contains a value outside the uint32 range")
    if column in {"pm100_post_faults", "pm100_run_faults"} and np.any(rounded > 2**24):
        warnings.warn(
            f"{column} exceeded 2**24. The firmware stores this uint32 word as float32, "
            "so combinations of high and low bits may have lost precision before logging."
        )
    words = rounded.astype(np.uint64)
    for bit, name in bit_names.items():
        decoded[name] = (words & (np.uint64(1) << np.uint64(bit))) != 0
    return decoded


def code_signal(frame, column, time_range_s=None) -> pd.DataFrame:
    selected = _signal_rows(frame, column, time_range_s)
    selected[column] = np.rint(selected[column]).astype(int)
    return selected


def _transition_preserving_sample(frame, state_columns, max_points):
    """Reduce plot size without dropping short state transitions."""
    if not max_points or len(frame) <= max_points or frame.empty:
        return frame
    states = frame.loc[:, state_columns]
    changed = states.ne(states.shift()).any(axis=1).to_numpy()
    transitions = np.flatnonzero(changed)
    required = set(transitions.tolist())
    required.update((transitions - 1).clip(min=0).tolist())
    required.update({0, len(frame) - 1})

    # Transitions are never discarded, so max_points is a soft ceiling when a
    # highly dynamic signal contains more changes than the configured limit.
    remaining = max(0, int(max_points) - len(required))
    if remaining:
        regular = np.linspace(0, len(frame) - 1, remaining, dtype=int)
        required.update(regular.tolist())
    return frame.iloc[sorted(required)]


def build_views(run, time_range_s=None) -> Dict[str, pd.DataFrame]:
    """Build every decoded status/fault view used by the notebook."""
    meas = run["data"]["MEAS"]
    deri = run["data"]["DERI"]
    status = decode_bitfield(deri, "status_flags", STATUS_FLAGS, time_range_s)

    vcu_faults = status[["elapsed_s"]].copy()
    for name in (
        "BSPC active",
        "BMS fault",
        "IMD fault",
        "PM100 fault",
        "BMS timed out",
        "IMD CAN timed out",
    ):
        vcu_faults[name] = status[name]
    vcu_faults["APPS implausible"] = ~status["APPS plausible"]

    return {
        "VCU status word": status,
        "VCU fault conditions": vcu_faults,
        "BMS status flags": decode_bitfield(meas, "bms_flags", BMS_FLAGS, time_range_s),
        "Logged-data validity": decode_bitfield(meas, "valid_flags", VALID_FLAGS, time_range_s),
        "IMD warnings and alarms": decode_bitfield(
            meas, "imd_warn_alarms", IMD_WARN_ALARMS, time_range_s
        ),
        "PM100 POST faults": decode_bitfield(
            meas, "pm100_post_faults", PM100_POST_FAULTS, time_range_s
        ),
        "PM100 run faults": decode_bitfield(
            meas, "pm100_run_faults", PM100_RUN_FAULTS, time_range_s
        ),
        "BMS fault code": code_signal(meas, "bms_fault_code", time_range_s),
        "RTD state": code_signal(deri, "rtd_state", time_range_s),
        "IMD Riso status": code_signal(meas, "imd_riso_status", time_range_s),
    }


def plot_boolean_lanes(
    decoded: pd.DataFrame,
    title: str,
    show_clear=True,
    color="#d62728",
    max_points=20_000,
):
    """Plot boolean signals as stacked digital timing lanes."""
    names = list(decoded.columns[1:])
    if not show_clear:
        names = [name for name in names if bool(decoded[name].any())]
    lane_count = max(1, len(names))
    fig, ax = plt.subplots(figsize=(15, max(2.5, 0.34 * lane_count + 1.5)), constrained_layout=True)
    ax.set_title(title)

    if decoded.empty:
        ax.text(0.5, 0.5, "No samples in the selected time range", ha="center", va="center", transform=ax.transAxes)
    elif not names:
        ax.text(0.5, 0.5, "No bits were active", ha="center", va="center", transform=ax.transAxes)
    else:
        shown = _transition_preserving_sample(decoded, names, max_points)
        x = shown["elapsed_s"].to_numpy()
        for lane, name in enumerate(names):
            active = shown[name].astype(float).to_numpy()
            baseline = float(lane)
            ax.step(x, baseline + 0.72 * active, where="post", color=color, linewidth=1.0)
            ax.fill_between(
                x,
                baseline,
                baseline + 0.72 * active,
                step="post",
                color=color,
                alpha=0.28,
            )
        ax.set_yticks(np.arange(len(names)) + 0.36, labels=names)
        ax.set_ylim(-0.12, len(names) + 0.05)
        ax.invert_yaxis()
    ax.set_xlabel("Elapsed time (s)")
    ax.grid(axis="x", alpha=0.25)
    return fig, ax


def plot_category(signal, column, names, title, color="#1f77b4", max_points=20_000):
    """Plot an integer status/code using readable categorical y-axis labels."""
    category_count = len(names)
    fig, ax = plt.subplots(
        figsize=(15, max(3.3, 0.3 * category_count + 1.4)), constrained_layout=True
    )
    ax.set_title(title)
    if signal.empty:
        ax.text(0.5, 0.5, "No samples in the selected time range", ha="center", va="center", transform=ax.transAxes)
    else:
        shown = _transition_preserving_sample(signal, [column], max_points)
        observed = sorted(shown[column].astype(int).unique())
        unknown = [value for value in observed if value not in names]
        ordered = list(names) + unknown
        positions = {value: index for index, value in enumerate(ordered)}
        y = shown[column].astype(int).map(positions)
        ax.step(shown["elapsed_s"], y, where="post", color=color, linewidth=1.25)
        ax.set_yticks(
            range(len(ordered)),
            labels=[f"{value}: {names.get(value, 'Unknown')}" for value in ordered],
        )
        ax.set_ylim(-0.5, len(ordered) - 0.5)
        ax.invert_yaxis()
    ax.set_xlabel("Elapsed time (s)")
    ax.grid(alpha=0.25)
    return fig, ax


def plot_all_status_faults(
    run,
    time_range_s=None,
    show_clear_fault_bits=True,
    max_points=20_000,
):
    """Create all status/fault figures and return the decoded views."""
    views = build_views(run, time_range_s)
    plot_boolean_lanes(
        views["VCU status word"],
        f"{run['id']} — VCU status word",
        show_clear=True,
        color="#1f77b4",
        max_points=max_points,
    )
    plot_boolean_lanes(
        views["VCU fault conditions"],
        f"{run['id']} — VCU fault conditions",
        show_clear=show_clear_fault_bits,
        max_points=max_points,
    )
    plot_boolean_lanes(
        views["BMS status flags"],
        f"{run['id']} — BMS status flags",
        show_clear=True,
        color="#2ca02c",
        max_points=max_points,
    )
    plot_category(
        views["RTD state"], "rtd_state", RTD_STATES, f"{run['id']} — RTD state", max_points=max_points
    )
    plot_category(
        views["IMD Riso status"],
        "imd_riso_status",
        IMD_RISO_STATES,
        f"{run['id']} — IMD isolation-measurement status",
        max_points=max_points,
    )
    plot_boolean_lanes(
        views["Logged-data validity"],
        f"{run['id']} — logged-data validity",
        show_clear=True,
        color="#2ca02c",
        max_points=max_points,
    )
    plot_category(
        views["BMS fault code"],
        "bms_fault_code",
        BMS_FAULT_CODES,
        f"{run['id']} — BMS fault code",
        max_points=max_points,
    )
    plot_boolean_lanes(
        views["IMD warnings and alarms"],
        f"{run['id']} — IMD warning/alarm bits",
        show_clear=show_clear_fault_bits,
        max_points=max_points,
    )
    plot_boolean_lanes(
        views["PM100 POST faults"],
        f"{run['id']} — PM100 POST fault bits",
        show_clear=show_clear_fault_bits,
        max_points=max_points,
    )
    plot_boolean_lanes(
        views["PM100 run faults"],
        f"{run['id']} — PM100 run fault bits",
        show_clear=show_clear_fault_bits,
        max_points=max_points,
    )
    plt.show()
    return views


def _active_intervals(decoded, category, end_time_s) -> Iterable[dict]:
    if decoded.empty:
        return []
    rows = []
    times = decoded["elapsed_s"].to_numpy(dtype=float)
    for name in decoded.columns[1:]:
        active = decoded[name].to_numpy(dtype=bool)
        start = None
        episode = 0
        for index, state in enumerate(active):
            if state and start is None:
                start = float(times[index])
                episode += 1
            if not state and start is not None:
                end = float(times[index])
                rows.append(
                    {
                        "category": category,
                        "fault_or_status": name,
                        "episode": episode,
                        "start_s": start,
                        "end_s": end,
                        "duration_s": max(0.0, end - start),
                    }
                )
                start = None
        if start is not None:
            end = max(start, float(end_time_s))
            rows.append(
                {
                    "category": category,
                    "fault_or_status": name,
                    "episode": episode,
                    "start_s": start,
                    "end_s": end,
                    "duration_s": max(0.0, end - start),
                }
            )
    return rows


def fault_event_table(run, views=None, time_range_s=None) -> pd.DataFrame:
    """Return one row per contiguous active fault/warning interval."""
    views = views if views is not None else build_views(run, time_range_s)
    if time_range_s is None:
        end_time = run["duration_s"]
    else:
        end_time = min(float(time_range_s[1]), run["duration_s"])

    event_groups = (
        "VCU fault conditions",
        "IMD warnings and alarms",
        "PM100 POST faults",
        "PM100 run faults",
    )
    rows = []
    for group in event_groups:
        rows.extend(_active_intervals(views[group], group, end_time))

    bms_codes = views["BMS fault code"]
    if not bms_codes.empty:
        one_hot = pd.DataFrame({"elapsed_s": bms_codes["elapsed_s"]})
        for code in sorted(code for code in bms_codes["bms_fault_code"].unique() if code != 0):
            one_hot[f"{int(code)}: {BMS_FAULT_CODES.get(int(code), 'Unknown BMS fault')}"] = (
                bms_codes["bms_fault_code"] == code
            )
        rows.extend(_active_intervals(one_hot, "BMS fault code", end_time))

    columns = ["category", "fault_or_status", "episode", "start_s", "end_s", "duration_s"]
    if not rows:
        return pd.DataFrame(columns=columns)
    return pd.DataFrame(rows, columns=columns).sort_values(
        ["start_s", "category", "fault_or_status"]
    ).reset_index(drop=True)


def active_fault_summary(events: pd.DataFrame) -> pd.DataFrame:
    """Aggregate the interval table by fault/status name."""
    columns = ["category", "fault_or_status", "episodes", "first_start_s", "last_end_s", "total_duration_s"]
    if events.empty:
        return pd.DataFrame(columns=columns)
    return (
        events.groupby(["category", "fault_or_status"], as_index=False)
        .agg(
            episodes=("episode", "count"),
            first_start_s=("start_s", "min"),
            last_end_s=("end_s", "max"),
            total_duration_s=("duration_s", "sum"),
        )
        .sort_values(["first_start_s", "category", "fault_or_status"])
        .reset_index(drop=True)
    )

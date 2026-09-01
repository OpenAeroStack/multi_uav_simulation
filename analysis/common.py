#!/usr/bin/env python3
"""Shared, schema-aware helpers for the real-versus-simulation comparison."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
from typing import Any, Iterable, Sequence

os.environ["MPLCONFIGDIR"] = "/tmp/matplotlib-fyp-comparison"
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyproj import Transformer

REPO_ROOT = Path(__file__).resolve().parents[1]
RESULTS_ROOT = REPO_ROOT / "results"
DEFAULT_REAL = RESULTS_ROOT / "real_2026-08-31_18-03-14"
DEFAULT_SIM = RESULTS_ROOT / "sim_run_2026-09-02_01-36-45"
OUTPUT_BASENAME = "comparison_real_2026-08-31_vs_sim_2026-09-02"
SUBDIRS = ("inspection", "trajectory", "altitude", "speed", "heading", "mission", "telemetry", "network", "tables", "summary")

COLUMN_MAPPING = {
    "elapsed_time": {"real": "elapsed_s", "simulation": "elapsed_time_s", "status": "COMPARABLE_AFTER_CONVERSION"},
    "arrival_timestamp": {"real": "log_timestamp_unix_s", "simulation": "wall_time_unix_s", "status": "DIRECTLY_COMPARABLE"},
    "latitude": {"real": "gps.csv:lat_deg", "simulation": "gps.csv:latitude_deg", "status": "DIRECTLY_COMPARABLE"},
    "longitude": {"real": "gps.csv:lon_deg", "simulation": "gps.csv:longitude_deg", "status": "DIRECTLY_COMPARABLE"},
    "relative_altitude": {"real": "gps.csv:relative_alt_m", "simulation": "gps.csv:relative_altitude_m", "status": "DIRECTLY_COMPARABLE"},
    "ground_speed": {"real": "gps.csv:ground_speed_mps", "simulation": "gps.csv:ground_speed_mps", "status": "DIRECTLY_COMPARABLE"},
    "heading": {"real": "gps.csv:heading_deg", "simulation": "gps.csv:heading_deg", "status": "DIRECTLY_COMPARABLE"},
    "roll": {"real": "vehicle_state.csv ATTITUDE:roll_deg", "simulation": "vehicle_state.csv:roll_deg", "status": "COMPARABLE_AFTER_CONVERSION"},
    "pitch": {"real": "vehicle_state.csv ATTITUDE:pitch_deg", "simulation": "vehicle_state.csv:pitch_deg", "status": "COMPARABLE_AFTER_CONVERSION"},
    "yaw": {"real": "vehicle_state.csv ATTITUDE:yaw_deg (heading convention)", "simulation": "vehicle_state.csv:heading_deg", "status": "COMPARABLE_AFTER_CONVERSION"},
    "waypoint_progress": {"real": "MISSION_CURRENT/MISSION_ITEM_REACHED", "simulation": "GUIDED target/reached events", "status": "COMPARABLE_AFTER_CONVERSION"},
    "telemetry_interarrival": {"real": "GLOBAL_POSITION_INT .tlog arrivals", "simulation": "/uav1/rel_alt bridge arrivals", "status": "DIRECTLY_COMPARABLE"},
    "mavlink_sequence": {"real": "mavlink_sequence.csv", "simulation": None, "status": "REAL_ONLY"},
    "snr_db": {"real": None, "simulation": "network.csv:snr_db", "status": "SIM_ONLY"},
    "signal_dbm": {"real": None, "simulation": "network.csv:signal_dbm", "status": "SIM_ONLY"},
    "radio_status": {"real": "radio.csv (3 samples)", "simulation": None, "status": "INSUFFICIENT_DATA"},
    "packet_loss": {"real": None, "simulation": None, "status": "INSUFFICIENT_DATA"},
    "rtt": {"real": None, "simulation": None, "status": "INSUFFICIENT_DATA"},
    "throughput": {"real": None, "simulation": None, "status": "INSUFFICIENT_DATA"},
}


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--real-dir", type=Path, default=DEFAULT_REAL)
    parser.add_argument("--sim-dir", type=Path, default=DEFAULT_SIM)
    parser.add_argument("--output-dir", type=Path)


def unique_dir(base: Path) -> Path:
    candidate, index = base, 2
    while candidate.exists():
        candidate = base.with_name(f"{base.name}_{index}")
        index += 1
    return candidate


def prepare_output(path: Path | None = None) -> Path:
    output = path.resolve() if path else unique_dir(RESULTS_ROOT / OUTPUT_BASENAME)
    output.mkdir(parents=True, exist_ok=True)
    for name in SUBDIRS:
        (output / name).mkdir(exist_ok=True)
    return output


def validate_inputs(real_dir: Path, sim_dir: Path) -> tuple[Path, Path]:
    real_dir, sim_dir = real_dir.resolve(), sim_dir.resolve()
    for label, directory in (("real", real_dir), ("simulation", sim_dir)):
        if not directory.is_dir():
            raise FileNotFoundError(f"{label} dataset directory not found: {directory}")
        for required in ("gps.csv", "events.csv", "mission.csv", "telemetry_timing.csv", "metadata.json"):
            if not (directory / required).is_file():
                raise FileNotFoundError(f"{label} dataset is missing {required}: {directory}")
    return real_dir, sim_dir


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def number(value: Any) -> float:
    try:
        result = float(value)
        return result if math.isfinite(result) else math.nan
    except (TypeError, ValueError):
        return math.nan


def numeric(rows: Sequence[dict[str, str]], column: str) -> np.ndarray:
    return np.asarray([number(row.get(column)) for row in rows], dtype=float)


def paired(rows: Sequence[dict[str, str]], time_col: str, value_col: str) -> tuple[np.ndarray, np.ndarray]:
    t, v = numeric(rows, time_col), numeric(rows, value_col)
    mask = np.isfinite(t) & np.isfinite(v)
    t, v = t[mask], v[mask]
    order = np.argsort(t, kind="stable")
    return t[order], v[order]


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: clean_value(row.get(key, "")) for key in fieldnames})


def clean_value(value: Any) -> Any:
    if isinstance(value, (float, np.floating)):
        return "" if not math.isfinite(float(value)) else f"{float(value):.9g}"
    if isinstance(value, (int, np.integer)):
        return int(value)
    return value


def write_json(path: Path, data: Any) -> None:
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def first_sustained(t: np.ndarray, values: np.ndarray, threshold: float = 0.5, count: int = 2) -> tuple[float, int]:
    for index in range(0, len(values) - count + 1):
        if np.all(values[index:index + count] >= threshold):
            return float(t[index]), index
    raise ValueError(f"No {count}-sample sustained crossing above {threshold} was found")


def event_time(rows: Sequence[dict[str, str]], dataset: str, event_type: str, after: float = -math.inf) -> float | None:
    time_col = "elapsed_s" if dataset == "real" else "elapsed_time_s"
    for row in rows:
        if row.get("event_type") == event_type:
            timestamp = number(row.get(time_col))
            if timestamp >= after:
                return timestamp
    return None


def determine_alignment(real_dir: Path, sim_dir: Path) -> dict[str, Any]:
    real_gps, sim_gps = read_rows(real_dir / "gps.csv"), read_rows(sim_dir / "gps.csv")
    rt, ra = paired(real_gps, "elapsed_s", "relative_alt_m")
    st, sa = paired(sim_gps, "elapsed_time_s", "relative_altitude_m")
    real_zero, real_i = first_sustained(rt, ra)
    sim_zero, sim_i = first_sustained(st, sa)
    real_events, sim_events = read_rows(real_dir / "events.csv"), read_rows(sim_dir / "events.csv")
    real_end_abs = event_time(real_events, "real", "DISARMED", real_zero)
    sim_end_abs = event_time(sim_events, "simulation", "LANDING", sim_zero)
    if real_end_abs is None:
        real_end_abs = float(rt[-1])
        real_end_name = "last GPS sample (fallback)"
    else:
        real_end_name = "DISARMED"
    if sim_end_abs is None:
        sim_end_abs = float(st[-1])
        sim_end_name = "last GPS sample (fallback)"
    else:
        sim_end_name = "LANDING"
    comparison_end = min(real_end_abs - real_zero, sim_end_abs - sim_zero)

    def original_timestamp(rows: list[dict[str, str]], time_col: str, absolute: float, stamp_col: str) -> float | None:
        times = numeric(rows, time_col)
        valid = np.where(np.isfinite(times))[0]
        if not len(valid):
            return None
        idx = valid[int(np.argmin(np.abs(times[valid] - absolute)))]
        stamp = number(rows[idx].get(stamp_col))
        return float(stamp) if math.isfinite(stamp) else None

    return {
        "method": "first two consecutive GPS samples at or above 0.5 m relative altitude",
        "rationale": "The real event log lacks an explicit TAKEOFF event; applying the same measured-altitude rule to both datasets avoids row-index and logger-start alignment.",
        "real_alignment_event": "sustained relative_alt_m >= 0.5 m",
        "sim_alignment_event": "sustained relative_altitude_m >= 0.5 m (corroborates recorded TAKEOFF event)",
        "real_alignment_elapsed_s": real_zero,
        "sim_alignment_elapsed_s": sim_zero,
        "real_original_timestamp": original_timestamp(real_gps, "elapsed_s", real_zero, "log_timestamp_unix_s"),
        "sim_original_timestamp": original_timestamp(sim_gps, "elapsed_time_s", sim_zero, "wall_time_unix_s"),
        "real_mission_end_event": real_end_name,
        "sim_mission_end_event": sim_end_name,
        "real_mission_end_after_alignment_s": real_end_abs - real_zero,
        "sim_mission_end_after_alignment_s": sim_end_abs - sim_zero,
        "comparison_start_s": 0.0,
        "comparison_end_s": comparison_end,
        "shared_duration_s": comparison_end,
        "no_extrapolation": True,
    }


def load_or_create_alignment(real_dir: Path, sim_dir: Path, output: Path) -> dict[str, Any]:
    path = output / "summary" / "time_alignment.json"
    alignment = determine_alignment(real_dir, sim_dir)
    write_json(path, alignment)
    return alignment


def mission_time(t: np.ndarray, dataset: str, alignment: dict[str, Any]) -> np.ndarray:
    zero = alignment[f"{dataset}_alignment_elapsed_s"]
    return t - zero


def common_grid(series: Sequence[tuple[np.ndarray, np.ndarray]], end_s: float) -> np.ndarray:
    medians = []
    for t, _ in series:
        selected = np.unique(t[(t >= 0) & (t <= end_s)])
        if len(selected) > 1:
            medians.append(float(np.median(np.diff(selected))))
    step = max(medians) if medians else 1.0
    step = min(max(step, 0.1), 2.0)
    return np.arange(0.0, end_s + step * 0.25, step)


def interpolate(t: np.ndarray, values: np.ndarray, grid: np.ndarray) -> np.ndarray:
    valid = np.isfinite(t) & np.isfinite(values)
    t, values = t[valid], values[valid]
    order = np.argsort(t, kind="stable")
    t, values = t[order], values[order]
    unique_t, indices = np.unique(t, return_index=True)
    values = values[indices]
    result = np.full(grid.shape, np.nan)
    inside = (grid >= unique_t[0]) & (grid <= unique_t[-1]) if len(unique_t) else np.zeros(grid.shape, bool)
    if len(unique_t):
        result[inside] = np.interp(grid[inside], unique_t, values)
    return result


def aligned_gps(directory: Path, dataset: str, alignment: dict[str, Any]) -> dict[str, np.ndarray]:
    rows = read_rows(directory / "gps.csv")
    cols = ({
        "time": "elapsed_s", "lat": "lat_deg", "lon": "lon_deg",
        "alt": "relative_alt_m", "speed": "ground_speed_mps", "heading": "heading_deg",
    } if dataset == "real" else {
        "time": "elapsed_time_s", "lat": "latitude_deg", "lon": "longitude_deg",
        "alt": "relative_altitude_m", "speed": "ground_speed_mps", "heading": "heading_deg",
    })
    result = {name: numeric(rows, column) for name, column in cols.items()}
    result["time"] = mission_time(result["time"], dataset, alignment)
    return result


def coordinate_origin(sim_dir: Path) -> dict[str, Any]:
    meta = read_json(sim_dir / "metadata.json")
    source = meta["gps_local_origin"]
    return {
        "latitude_deg": float(source["latitude_deg"]),
        "longitude_deg": float(source["longitude_deg"]),
        "altitude_m": float(source["altitude_msl_m"]),
        "source": "simulation metadata gps_local_origin (/ap/v1/gps_global_origin/filtered)",
        "crs": "WGS84 EPSG:4979 to ECEF EPSG:4978, then common tangent ENU",
        "reason": "Both latitude/longitude tracks are transformed anew with one recorded origin; dataset-specific local coordinates are not compared.",
    }


def gps_to_enu(lat: np.ndarray, lon: np.ndarray, alt: np.ndarray, origin: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    transformer = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
    ox, oy, oz = transformer.transform(origin["longitude_deg"], origin["latitude_deg"], origin["altitude_m"])
    x, y, z = transformer.transform(lon, lat, alt)
    dx, dy, dz = x - ox, y - oy, z - oz
    phi, lam = math.radians(origin["latitude_deg"]), math.radians(origin["longitude_deg"])
    east = -math.sin(lam) * dx + math.cos(lam) * dy
    north = -math.sin(phi) * math.cos(lam) * dx - math.sin(phi) * math.sin(lam) * dy + math.cos(phi) * dz
    up = math.cos(phi) * math.cos(lam) * dx + math.cos(phi) * math.sin(lam) * dy + math.sin(phi) * dz
    return east, north, up


def metrics(errors: np.ndarray, prefix: str = "") -> dict[str, float]:
    values = errors[np.isfinite(errors)]
    absolute = np.abs(values)
    return {
        f"mean_{prefix}error": float(np.mean(values)),
        f"median_{prefix}error": float(np.median(values)),
        f"mae_{prefix}": float(np.mean(absolute)),
        f"rmse_{prefix}": float(np.sqrt(np.mean(values ** 2))),
        f"p95_abs_{prefix}error": float(np.percentile(absolute, 95)),
        f"max_abs_{prefix}error": float(np.max(absolute)),
    }


def circular_difference_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return (a - b + 180.0) % 360.0 - 180.0


def path_distance(east: np.ndarray, north: np.ndarray, time: np.ndarray, end_s: float) -> float:
    mask = np.isfinite(east) & np.isfinite(north) & (time >= 0) & (time <= end_s)
    x, y = east[mask], north[mask]
    return float(np.sum(np.hypot(np.diff(x), np.diff(y)))) if len(x) > 1 else math.nan


def save_plot(path: Path) -> None:
    plt.tight_layout()
    plt.savefig(path, dpi=300, bbox_inches="tight")
    plt.close()


def load_metric_file(path: Path) -> dict[str, float | str]:
    result: dict[str, float | str] = {}
    if not path.exists():
        return result
    for row in read_rows(path):
        raw = row.get("value", "")
        parsed = number(raw)
        result[row["metric"]] = parsed if math.isfinite(parsed) else raw
    return result


def metric_rows(data: dict[str, Any], units: dict[str, str] | None = None) -> list[dict[str, Any]]:
    units = units or {}
    return [{"metric": key, "value": value, "unit": units.get(key, "")} for key, value in data.items()]

#!/usr/bin/env python3
"""Extract a Mission Planner MAVLink telemetry log into analysis-ready CSV files.

The input file is opened read-only.  MAVLink log-arrival timestamps are the common
timeline; vehicle-provided clocks are retained as separate source-clock columns.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import statistics
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

try:
    from pymavlink import mavutil
except ImportError as exc:  # pragma: no cover - environment-dependent error path
    raise SystemExit(
        "pymavlink is required. Source the ArduPilot/ROS environment that provides "
        "it, or install pymavlink for this Python interpreter."
    ) from exc

try:
    import pyproj
    from pyproj import Transformer
except ImportError as exc:  # pragma: no cover - environment-dependent error path
    raise SystemExit(
        "pyproj is required for WGS84 geodetic-to-local conversion. "
        "Install python3-pyproj (or pyproj) and retry."
    ) from exc


DEFAULT_INPUT = Path(
    "/home/randilsk/FYP/logs_using/0-30m (copy)/2026-08-31 18-03-14.tlog"
)
DEFAULT_OUTPUT_ROOT = Path("/home/randilsk/FYP/logs_using/processed_real_logs")
GAP_THRESHOLDS_MS = (750, 1000, 1500, 2000)


CSV_SCHEMAS: Dict[str, list[str]] = {
    "gps.csv": [
        "elapsed_s", "log_timestamp_unix_s", "source_time_boot_ms", "seq",
        "sysid", "compid", "lat_raw_e7", "lon_raw_e7", "alt_raw_mm",
        "relative_alt_raw_mm", "lat_deg", "lon_deg", "alt_m_msl",
        "relative_alt_m", "vx_raw_cm_s", "vy_raw_cm_s", "vz_raw_cm_s",
        "vx_north_mps", "vy_east_mps", "vz_down_mps",
        "vertical_speed_up_mps", "ground_speed_mps", "heading_raw_cdeg",
        "heading_deg", "local_east_m", "local_north_m", "local_up_m",
    ],
    "gps_quality.csv": [
        "elapsed_s", "log_timestamp_unix_s", "source_time_usec", "seq",
        "sysid", "compid", "fix_type", "satellites_visible", "lat_raw_e7",
        "lon_raw_e7", "alt_raw_mm", "lat_deg", "lon_deg", "alt_m_msl",
        "eph_raw", "epv_raw", "eph_scaled_1e_2", "epv_scaled_1e_2",
        "vel_raw_cm_s", "ground_speed_mps", "cog_raw_cdeg", "cog_deg",
        "alt_ellipsoid_raw_mm", "alt_ellipsoid_m", "h_acc_raw_mm",
        "h_acc_m", "v_acc_raw_mm", "v_acc_m", "vel_acc_raw_mm_s",
        "vel_acc_mps", "hdg_acc_raw_1e5_deg", "hdg_acc_deg",
        "yaw_raw_cdeg", "yaw_deg",
    ],
    "vehicle_state.csv": [
        "elapsed_s", "log_timestamp_unix_s", "source_message",
        "source_time_boot_ms", "seq", "sysid", "compid", "roll_rad",
        "pitch_rad", "yaw_rad", "roll_deg", "pitch_deg", "yaw_deg",
        "roll_rate_rad_s", "pitch_rate_rad_s", "yaw_rate_rad_s",
        "airspeed_mps", "groundspeed_mps", "heading_deg", "throttle_percent",
        "alt_m_msl", "climb_up_mps", "armed", "flight_mode", "base_mode",
        "custom_mode", "system_status", "vehicle_type", "autopilot_type",
    ],
    "local_position_ned.csv": [
        "elapsed_s", "log_timestamp_unix_s", "source_time_boot_ms", "seq",
        "sysid", "compid", "north_x_m", "east_y_m", "down_z_m",
        "velocity_north_mps", "velocity_east_mps", "velocity_down_mps",
    ],
    "mission.csv": [
        "elapsed_s", "log_timestamp_unix_s", "source_message",
        "source_time_boot_ms", "packet_seq", "sysid", "compid",
        "mission_item_seq", "mission_total", "mission_state", "mission_mode",
        "coordinate_frame", "type_mask", "target_lat_raw_e7",
        "target_lon_raw_e7", "target_lat_deg", "target_lon_deg", "target_alt_m",
        "target_vx_mps", "target_vy_mps", "target_vz_mps", "target_afx_mps2",
        "target_afy_mps2", "target_afz_mps2", "target_yaw_rad",
        "target_yaw_rate_rad_s",
    ],
    "telemetry_timing.csv": [
        "elapsed_s", "log_timestamp_unix_s", "source_time_boot_ms", "seq",
        "sysid", "compid", "arrival_delta_ms", "instantaneous_rate_hz",
        "gap_gt_750ms", "gap_gt_1000ms", "gap_gt_1500ms", "gap_gt_2000ms",
    ],
    "mavlink_sequence.csv": [
        "elapsed_s", "log_timestamp_unix_s", "message_type", "seq", "sysid",
        "compid", "previous_seq", "sequence_delta_mod256",
        "estimated_missing_before", "sequence_status",
    ],
    "radio.csv": [
        "elapsed_s", "log_timestamp_unix_s", "source_message", "seq", "sysid",
        "compid", "rssi_raw", "remote_rssi_raw", "noise_raw",
        "remote_noise_raw", "tx_buffer_percent", "rx_errors", "corrected_errors",
    ],
    "events.csv": [
        "elapsed_s", "log_timestamp_unix_s", "event_type", "source_message",
        "sysid", "compid", "details",
    ],
    "statustext.csv": [
        "elapsed_s", "log_timestamp_unix_s", "seq", "sysid", "compid",
        "severity", "text_id", "chunk_seq", "text",
    ],
}


def value(msg: Any, name: str, default: Any = "") -> Any:
    """Return a MAVLink field without manufacturing absent extension fields."""
    return getattr(msg, name, default)


def scaled(raw: Any, factor: float, invalid: Iterable[int] = ()) -> Any:
    if raw in (None, "") or raw in invalid:
        return ""
    return float(raw) * factor


def fmt_float(number: Any, digits: int = 9) -> Any:
    if number in (None, ""):
        return ""
    return f"{float(number):.{digits}f}"


def percentile(values: list[float], percent: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def utc_iso(timestamp: Optional[float]) -> Optional[str]:
    if timestamp is None:
        return None
    return datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class LocalFrame:
    """WGS84 geodetic coordinates converted to a tangent ENU frame."""

    def __init__(self, lat_deg: float, lon_deg: float, alt_m: float):
        self.lat_deg = lat_deg
        self.lon_deg = lon_deg
        self.alt_m = alt_m
        self._to_ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
        self._origin_ecef = self._to_ecef.transform(lon_deg, lat_deg, alt_m)
        self._lat = math.radians(lat_deg)
        self._lon = math.radians(lon_deg)

    def enu(self, lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
        x, y, z = self._to_ecef.transform(lon_deg, lat_deg, alt_m)
        dx = x - self._origin_ecef[0]
        dy = y - self._origin_ecef[1]
        dz = z - self._origin_ecef[2]
        sin_lat, cos_lat = math.sin(self._lat), math.cos(self._lat)
        sin_lon, cos_lon = math.sin(self._lon), math.cos(self._lon)
        east = -sin_lon * dx + cos_lon * dy
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        return east, north, up


def unique_output_dir(root: Path, input_path: Path) -> Path:
    stem = re.sub(r"[^A-Za-z0-9._-]+", "_", input_path.stem).strip("_")
    base = root / f"real_{stem}"
    candidate = base
    suffix = 2
    while candidate.exists():
        candidate = root / f"{base.name}_{suffix}"
        suffix += 1
    candidate.mkdir(parents=True, exist_ok=False)
    return candidate


def write_row(writer: csv.DictWriter, schema: list[str], row: Dict[str, Any]) -> None:
    writer.writerow({column: row.get(column, "") for column in schema})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a Mission Planner .tlog into organized real-flight datasets."
    )
    parser.add_argument("input", nargs="?", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--origin-lat", type=float)
    parser.add_argument("--origin-lon", type=float)
    parser.add_argument("--origin-alt", type=float)
    args = parser.parse_args()
    if (args.origin_lat is None) != (args.origin_lon is None):
        parser.error("--origin-lat and --origin-lon must be supplied together")
    if args.origin_alt is not None and args.origin_lat is None:
        parser.error("--origin-alt requires --origin-lat and --origin-lon")
    return args


def extract(args: argparse.Namespace) -> Path:
    input_path = args.input.expanduser().resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"Input .tlog does not exist: {input_path}")
    if input_path.suffix.lower() != ".tlog":
        raise ValueError(f"Expected a .tlog input, got: {input_path}")

    input_stat = input_path.stat()
    input_hash = sha256_file(input_path)
    output_dir = unique_output_dir(args.output_root.expanduser(), input_path)

    handles: Dict[str, Any] = {}
    writers: Dict[str, csv.DictWriter] = {}
    try:
        for filename, schema in CSV_SCHEMAS.items():
            handle = (output_dir / filename).open("w", newline="", encoding="utf-8")
            writer = csv.DictWriter(handle, fieldnames=schema)
            writer.writeheader()
            handles[filename] = handle
            writers[filename] = writer

        counts: Counter[str] = Counter()
        endpoint_counts: Counter[Tuple[int, int]] = Counter()
        output_counts: Counter[str] = Counter()
        warning_messages: list[str] = []
        first_timestamp: Optional[float] = None
        last_timestamp: Optional[float] = None
        previous_log_timestamp: Optional[float] = None
        nonmonotonic_log_timestamps = 0
        malformed_packets = 0

        local_frame: Optional[LocalFrame] = None
        origin_source: Optional[str] = None
        if args.origin_lat is not None and args.origin_alt is not None:
            local_frame = LocalFrame(args.origin_lat, args.origin_lon, args.origin_alt)
            origin_source = "explicit_cli"

        previous_seq: Dict[Tuple[int, int], int] = {}
        sequence_groups: Dict[Tuple[int, int], Counter[str]] = defaultdict(Counter)
        previous_gps_timestamp: Optional[float] = None
        gps_intervals_ms: list[float] = []
        gap_counts = {threshold: 0 for threshold in GAP_THRESHOLDS_MS}
        previous_armed: Dict[Tuple[int, int], bool] = {}
        previous_mode: Dict[Tuple[int, int], str] = {}
        previous_mission_item: Dict[Tuple[int, int], int] = {}
        observed_modes: set[str] = set()
        reached_waypoints: list[int] = []

        connection = mavutil.mavlink_connection(str(input_path), robust_parsing=True)

        def event(
            timestamp: float,
            event_type: str,
            source_message: str,
            sysid: Any,
            compid: Any,
            details: str,
        ) -> None:
            write_row(writers["events.csv"], CSV_SCHEMAS["events.csv"], {
                "elapsed_s": fmt_float(timestamp - first_timestamp, 6),
                "log_timestamp_unix_s": fmt_float(timestamp, 6),
                "event_type": event_type,
                "source_message": source_message,
                "sysid": sysid,
                "compid": compid,
                "details": details,
            })
            output_counts["events.csv"] += 1

        while True:
            msg = connection.recv_match(blocking=False)
            if msg is None:
                break
            message_type = msg.get_type()
            if message_type == "BAD_DATA":
                malformed_packets += 1
                continue

            timestamp_raw = getattr(msg, "_timestamp", None)
            try:
                timestamp = float(timestamp_raw)
            except (TypeError, ValueError):
                warning_messages.append(f"Skipped {message_type}: no valid log timestamp")
                continue
            if not math.isfinite(timestamp):
                warning_messages.append(f"Skipped {message_type}: non-finite log timestamp")
                continue

            sysid = int(msg.get_srcSystem())
            compid = int(msg.get_srcComponent())
            packet_seq = int(msg.get_seq())
            if first_timestamp is None:
                first_timestamp = timestamp
                event(timestamp, "LOG_START", message_type, sysid, compid, "First valid MAVLink packet")
            if previous_log_timestamp is not None and timestamp < previous_log_timestamp:
                nonmonotonic_log_timestamps += 1
            previous_log_timestamp = timestamp
            last_timestamp = timestamp
            elapsed = timestamp - first_timestamp
            common = {
                "elapsed_s": fmt_float(elapsed, 6),
                "log_timestamp_unix_s": fmt_float(timestamp, 6),
                "seq": packet_seq,
                "packet_seq": packet_seq,
                "sysid": sysid,
                "compid": compid,
            }
            counts[message_type] += 1
            endpoint_counts[(sysid, compid)] += 1

            endpoint = (sysid, compid)
            prior_seq = previous_seq.get(endpoint)
            delta: Any = ""
            missing = 0
            if prior_seq is None:
                sequence_status = "first_observed"
            else:
                delta = (packet_seq - prior_seq) % 256
                if delta == 1:
                    sequence_status = "continuous"
                elif delta == 0:
                    sequence_status = "duplicate_or_same_sequence"
                elif 2 <= delta <= 127:
                    sequence_status = "forward_gap"
                    missing = delta - 1
                else:
                    sequence_status = "out_of_order_or_stream_reset"
                sequence_groups[endpoint][sequence_status] += 1
                sequence_groups[endpoint]["estimated_missing"] += missing
            previous_seq[endpoint] = packet_seq
            write_row(writers["mavlink_sequence.csv"], CSV_SCHEMAS["mavlink_sequence.csv"], {
                **common,
                "message_type": message_type,
                "previous_seq": "" if prior_seq is None else prior_seq,
                "sequence_delta_mod256": delta,
                "estimated_missing_before": missing,
                "sequence_status": sequence_status,
            })
            output_counts["mavlink_sequence.csv"] += 1

            if message_type == "GLOBAL_POSITION_INT":
                lat_raw, lon_raw = value(msg, "lat"), value(msg, "lon")
                alt_raw = value(msg, "alt")
                lat_deg, lon_deg, alt_m = lat_raw / 1e7, lon_raw / 1e7, alt_raw / 1000.0
                if local_frame is None:
                    origin_alt = alt_m if args.origin_alt is None else args.origin_alt
                    origin_lat = lat_deg if args.origin_lat is None else args.origin_lat
                    origin_lon = lon_deg if args.origin_lon is None else args.origin_lon
                    local_frame = LocalFrame(origin_lat, origin_lon, origin_alt)
                    origin_source = (
                        "first_valid_global_position_int"
                        if args.origin_lat is None
                        else "explicit_cli_lat_lon_first_gps_alt"
                    )
                east, north, up = local_frame.enu(lat_deg, lon_deg, alt_m)
                vx_raw, vy_raw, vz_raw = value(msg, "vx"), value(msg, "vy"), value(msg, "vz")
                heading_raw = value(msg, "hdg")
                write_row(writers["gps.csv"], CSV_SCHEMAS["gps.csv"], {
                    **common,
                    "source_time_boot_ms": value(msg, "time_boot_ms"),
                    "lat_raw_e7": lat_raw, "lon_raw_e7": lon_raw,
                    "alt_raw_mm": alt_raw,
                    "relative_alt_raw_mm": value(msg, "relative_alt"),
                    "lat_deg": fmt_float(lat_deg), "lon_deg": fmt_float(lon_deg),
                    "alt_m_msl": fmt_float(alt_m, 3),
                    "relative_alt_m": scaled(value(msg, "relative_alt"), 0.001),
                    "vx_raw_cm_s": vx_raw, "vy_raw_cm_s": vy_raw, "vz_raw_cm_s": vz_raw,
                    "vx_north_mps": scaled(vx_raw, 0.01),
                    "vy_east_mps": scaled(vy_raw, 0.01),
                    "vz_down_mps": scaled(vz_raw, 0.01),
                    "vertical_speed_up_mps": scaled(vz_raw, -0.01),
                    "ground_speed_mps": math.hypot(vx_raw, vy_raw) * 0.01,
                    "heading_raw_cdeg": heading_raw,
                    "heading_deg": scaled(heading_raw, 0.01, (65535,)),
                    "local_east_m": fmt_float(east, 4),
                    "local_north_m": fmt_float(north, 4),
                    "local_up_m": fmt_float(up, 4),
                })
                output_counts["gps.csv"] += 1

                delta_ms: Any = ""
                rate_hz: Any = ""
                flags = {threshold: False for threshold in GAP_THRESHOLDS_MS}
                if previous_gps_timestamp is not None:
                    delta_ms_value = (timestamp - previous_gps_timestamp) * 1000.0
                    delta_ms = fmt_float(delta_ms_value, 3)
                    if delta_ms_value > 0:
                        gps_intervals_ms.append(delta_ms_value)
                        rate_hz = fmt_float(1000.0 / delta_ms_value, 6)
                    for threshold in GAP_THRESHOLDS_MS:
                        flags[threshold] = delta_ms_value > threshold
                        if flags[threshold]:
                            gap_counts[threshold] += 1
                    if delta_ms_value > 1000:
                        event(
                            timestamp, "TELEMETRY_GAP_GT_1S", message_type, sysid, compid,
                            f"GLOBAL_POSITION_INT arrival interval {delta_ms_value:.3f} ms",
                        )
                previous_gps_timestamp = timestamp
                write_row(writers["telemetry_timing.csv"], CSV_SCHEMAS["telemetry_timing.csv"], {
                    **common,
                    "source_time_boot_ms": value(msg, "time_boot_ms"),
                    "arrival_delta_ms": delta_ms,
                    "instantaneous_rate_hz": rate_hz,
                    **{f"gap_gt_{threshold}ms": int(flags[threshold]) for threshold in GAP_THRESHOLDS_MS},
                })
                output_counts["telemetry_timing.csv"] += 1

            elif message_type == "GPS_RAW_INT":
                lat_raw, lon_raw, alt_raw = value(msg, "lat"), value(msg, "lon"), value(msg, "alt")
                eph, epv = value(msg, "eph"), value(msg, "epv")
                cog, yaw = value(msg, "cog"), value(msg, "yaw")
                write_row(writers["gps_quality.csv"], CSV_SCHEMAS["gps_quality.csv"], {
                    **common,
                    "source_time_usec": value(msg, "time_usec"),
                    "fix_type": value(msg, "fix_type"),
                    "satellites_visible": value(msg, "satellites_visible"),
                    "lat_raw_e7": lat_raw, "lon_raw_e7": lon_raw, "alt_raw_mm": alt_raw,
                    "lat_deg": scaled(lat_raw, 1e-7), "lon_deg": scaled(lon_raw, 1e-7),
                    "alt_m_msl": scaled(alt_raw, 0.001),
                    "eph_raw": eph, "epv_raw": epv,
                    "eph_scaled_1e_2": scaled(eph, 0.01, (65535,)),
                    "epv_scaled_1e_2": scaled(epv, 0.01, (65535,)),
                    "vel_raw_cm_s": value(msg, "vel"),
                    "ground_speed_mps": scaled(value(msg, "vel"), 0.01, (65535,)),
                    "cog_raw_cdeg": cog, "cog_deg": scaled(cog, 0.01, (65535,)),
                    "alt_ellipsoid_raw_mm": value(msg, "alt_ellipsoid"),
                    "alt_ellipsoid_m": scaled(value(msg, "alt_ellipsoid"), 0.001),
                    "h_acc_raw_mm": value(msg, "h_acc"),
                    "h_acc_m": scaled(value(msg, "h_acc"), 0.001, (4294967295,)),
                    "v_acc_raw_mm": value(msg, "v_acc"),
                    "v_acc_m": scaled(value(msg, "v_acc"), 0.001, (4294967295,)),
                    "vel_acc_raw_mm_s": value(msg, "vel_acc"),
                    "vel_acc_mps": scaled(value(msg, "vel_acc"), 0.001, (4294967295,)),
                    "hdg_acc_raw_1e5_deg": value(msg, "hdg_acc"),
                    "hdg_acc_deg": scaled(value(msg, "hdg_acc"), 1e-5, (4294967295,)),
                    "yaw_raw_cdeg": yaw, "yaw_deg": scaled(yaw, 0.01, (0, 65535)),
                })
                output_counts["gps_quality.csv"] += 1

            elif message_type in ("ATTITUDE", "VFR_HUD", "HEARTBEAT"):
                # Mission Planner writes its own GCS heartbeat into a bidirectional
                # .tlog.  It remains represented in message/sequence counts, but it
                # is not a vehicle-state sample and must not create flight-mode events.
                if message_type == "HEARTBEAT" and (
                    value(msg, "type") == mavutil.mavlink.MAV_TYPE_GCS
                    or value(msg, "autopilot") == mavutil.mavlink.MAV_AUTOPILOT_INVALID
                ):
                    continue
                row: Dict[str, Any] = {**common, "source_message": message_type}
                if message_type == "ATTITUDE":
                    roll, pitch, yaw = value(msg, "roll"), value(msg, "pitch"), value(msg, "yaw")
                    row.update({
                        "source_time_boot_ms": value(msg, "time_boot_ms"),
                        "roll_rad": roll, "pitch_rad": pitch, "yaw_rad": yaw,
                        "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch),
                        "yaw_deg": math.degrees(yaw),
                        "roll_rate_rad_s": value(msg, "rollspeed"),
                        "pitch_rate_rad_s": value(msg, "pitchspeed"),
                        "yaw_rate_rad_s": value(msg, "yawspeed"),
                    })
                elif message_type == "VFR_HUD":
                    row.update({
                        "airspeed_mps": value(msg, "airspeed"),
                        "groundspeed_mps": value(msg, "groundspeed"),
                        "heading_deg": value(msg, "heading"),
                        "throttle_percent": value(msg, "throttle"),
                        "alt_m_msl": value(msg, "alt"),
                        "climb_up_mps": value(msg, "climb"),
                    })
                else:
                    armed = bool(value(msg, "base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    try:
                        mode = mavutil.mode_string_v10(msg)
                    except Exception:
                        mode = f"CUSTOM_MODE_{value(msg, 'custom_mode', '')}"
                    observed_modes.add(mode)
                    row.update({
                        "armed": int(armed), "flight_mode": mode,
                        "base_mode": value(msg, "base_mode"),
                        "custom_mode": value(msg, "custom_mode"),
                        "system_status": value(msg, "system_status"),
                        "vehicle_type": value(msg, "type"),
                        "autopilot_type": value(msg, "autopilot"),
                    })
                    if endpoint not in previous_armed:
                        event(timestamp, "ARM_STATE_OBSERVED", message_type, sysid, compid, "ARMED" if armed else "DISARMED")
                    elif previous_armed[endpoint] != armed:
                        event(timestamp, "ARMED" if armed else "DISARMED", message_type, sysid, compid, mode)
                    previous_armed[endpoint] = armed
                    if endpoint not in previous_mode:
                        event(timestamp, "MODE_OBSERVED", message_type, sysid, compid, mode)
                    elif previous_mode[endpoint] != mode:
                        event(timestamp, "MODE_CHANGED", message_type, sysid, compid, f"{previous_mode[endpoint]} -> {mode}")
                        if mode.upper() in ("RTL", "LAND"):
                            event(timestamp, f"{mode.upper()}_MODE_ENTERED", message_type, sysid, compid, mode)
                    previous_mode[endpoint] = mode
                write_row(writers["vehicle_state.csv"], CSV_SCHEMAS["vehicle_state.csv"], row)
                output_counts["vehicle_state.csv"] += 1

            elif message_type == "LOCAL_POSITION_NED":
                write_row(writers["local_position_ned.csv"], CSV_SCHEMAS["local_position_ned.csv"], {
                    **common,
                    "source_time_boot_ms": value(msg, "time_boot_ms"),
                    "north_x_m": value(msg, "x"), "east_y_m": value(msg, "y"),
                    "down_z_m": value(msg, "z"),
                    "velocity_north_mps": value(msg, "vx"),
                    "velocity_east_mps": value(msg, "vy"),
                    "velocity_down_mps": value(msg, "vz"),
                })
                output_counts["local_position_ned.csv"] += 1

            elif message_type in ("MISSION_CURRENT", "POSITION_TARGET_GLOBAL_INT", "MISSION_ITEM_REACHED"):
                row = {**common, "source_message": message_type}
                if message_type == "MISSION_CURRENT":
                    item = int(value(msg, "seq"))
                    row.update({
                        "mission_item_seq": item, "mission_total": value(msg, "total"),
                        "mission_state": value(msg, "mission_state"),
                        "mission_mode": value(msg, "mission_mode"),
                    })
                    if endpoint not in previous_mission_item:
                        event(timestamp, "WAYPOINT_CURRENT_OBSERVED", message_type, sysid, compid, f"seq={item}")
                    elif previous_mission_item[endpoint] != item:
                        event(timestamp, "WAYPOINT_CURRENT_CHANGED", message_type, sysid, compid, f"{previous_mission_item[endpoint]} -> {item}")
                    previous_mission_item[endpoint] = item
                elif message_type == "MISSION_ITEM_REACHED":
                    item = int(value(msg, "seq"))
                    row["mission_item_seq"] = item
                    reached_waypoints.append(item)
                    event(timestamp, "WAYPOINT_REACHED", message_type, sysid, compid, f"seq={item}")
                else:
                    lat_raw, lon_raw = value(msg, "lat_int"), value(msg, "lon_int")
                    row.update({
                        "source_time_boot_ms": value(msg, "time_boot_ms"),
                        "coordinate_frame": value(msg, "coordinate_frame"),
                        "type_mask": value(msg, "type_mask"),
                        "target_lat_raw_e7": lat_raw, "target_lon_raw_e7": lon_raw,
                        "target_lat_deg": scaled(lat_raw, 1e-7),
                        "target_lon_deg": scaled(lon_raw, 1e-7),
                        "target_alt_m": value(msg, "alt"),
                        "target_vx_mps": value(msg, "vx"), "target_vy_mps": value(msg, "vy"),
                        "target_vz_mps": value(msg, "vz"), "target_afx_mps2": value(msg, "afx"),
                        "target_afy_mps2": value(msg, "afy"), "target_afz_mps2": value(msg, "afz"),
                        "target_yaw_rad": value(msg, "yaw"),
                        "target_yaw_rate_rad_s": value(msg, "yaw_rate"),
                    })
                write_row(writers["mission.csv"], CSV_SCHEMAS["mission.csv"], row)
                output_counts["mission.csv"] += 1

            elif message_type in ("RADIO_STATUS", "RADIO"):
                write_row(writers["radio.csv"], CSV_SCHEMAS["radio.csv"], {
                    **common, "source_message": message_type,
                    "rssi_raw": value(msg, "rssi"), "remote_rssi_raw": value(msg, "remrssi"),
                    "noise_raw": value(msg, "noise"), "remote_noise_raw": value(msg, "remnoise"),
                    "tx_buffer_percent": value(msg, "txbuf"), "rx_errors": value(msg, "rxerrors"),
                    "corrected_errors": value(msg, "fixed"),
                })
                output_counts["radio.csv"] += 1

            elif message_type == "STATUSTEXT":
                text = value(msg, "text")
                if isinstance(text, bytes):
                    text = text.decode("utf-8", errors="replace")
                text = str(text).rstrip("\x00")
                write_row(writers["statustext.csv"], CSV_SCHEMAS["statustext.csv"], {
                    **common, "severity": value(msg, "severity"),
                    "text_id": value(msg, "id"), "chunk_seq": value(msg, "chunk_seq"),
                    "text": text,
                })
                output_counts["statustext.csv"] += 1
                event(timestamp, "STATUSTEXT", message_type, sysid, compid, f"severity={value(msg, 'severity')}: {text}")

        if first_timestamp is None or last_timestamp is None:
            raise ValueError("No valid timestamped MAVLink packets were found")
        event(last_timestamp, "LOG_END", "TLOG", "", "", "Last valid MAVLink packet")

        required_types = (
            "GLOBAL_POSITION_INT", "GPS_RAW_INT", "ATTITUDE", "VFR_HUD",
            "LOCAL_POSITION_NED", "MISSION_CURRENT", "HEARTBEAT",
        )
        for required in required_types:
            if counts[required] == 0:
                warning_messages.append(f"Optional/expected message {required} was not present")
        if counts["RADIO_STATUS"] + counts["RADIO"] == 0:
            warning_messages.append("No RADIO_STATUS or RADIO messages; radio.csv contains only its header")

        duration_s = last_timestamp - first_timestamp
        timing_summary = {
            "source_message": "GLOBAL_POSITION_INT",
            "sample_count": counts["GLOBAL_POSITION_INT"],
            "interval_count": len(gps_intervals_ms),
            "duration_first_to_last_sample_s": (
                sum(gps_intervals_ms) / 1000.0 if gps_intervals_ms else 0.0
            ),
            "effective_rate_hz": (
                (len(gps_intervals_ms) / (sum(gps_intervals_ms) / 1000.0))
                if gps_intervals_ms and sum(gps_intervals_ms) > 0 else None
            ),
            "mean_interval_ms": statistics.fmean(gps_intervals_ms) if gps_intervals_ms else None,
            "median_interval_ms": statistics.median(gps_intervals_ms) if gps_intervals_ms else None,
            "p95_interval_ms": percentile(gps_intervals_ms, 95),
            "max_interval_ms": max(gps_intervals_ms) if gps_intervals_ms else None,
            "gap_counts_strictly_greater_than_ms": {str(k): v for k, v in gap_counts.items()},
        }
        sequence_summary = []
        for endpoint, observed_count in sorted(endpoint_counts.items()):
            group = sequence_groups[endpoint]
            estimated_missing = group["estimated_missing"]
            denominator = observed_count + estimated_missing
            sequence_summary.append({
                "sysid": endpoint[0], "compid": endpoint[1],
                "observed_packet_count": observed_count,
                "continuous_transitions": group["continuous"],
                "forward_gap_transitions": group["forward_gap"],
                "duplicate_or_same_sequence_transitions": group["duplicate_or_same_sequence"],
                "out_of_order_or_stream_reset_transitions": group["out_of_order_or_stream_reset"],
                "estimated_missing_packets": estimated_missing,
                "diagnostic_estimated_missing_percent": (
                    estimated_missing * 100.0 / denominator if denominator else None
                ),
            })

        metadata = {
            "schema_version": 1,
            "dataset_kind": "real_mission_planner_tlog",
            "processing_status": "complete",
            "processed_at_utc": datetime.now(timezone.utc).isoformat(),
            "input": {
                "path": str(input_path), "size_bytes": input_stat.st_size,
                "mtime_utc": utc_iso(input_stat.st_mtime), "sha256": input_hash,
                "opened_read_only": True,
            },
            "output_directory": str(output_dir),
            "log_timeline": {
                "primary_clock": "Mission Planner .tlog packet-arrival timestamp (_timestamp)",
                "start_unix_s": first_timestamp, "start_utc": utc_iso(first_timestamp),
                "end_unix_s": last_timestamp, "end_utc": utc_iso(last_timestamp),
                "duration_s": duration_s,
                "elapsed_definition": "log_timestamp_unix_s - first valid packet timestamp",
                "nonmonotonic_arrival_timestamp_count": nonmonotonic_log_timestamps,
                "source_clocks_preserved": ["time_boot_ms", "time_usec"],
            },
            "coordinate_frame": {
                "geodetic_crs": "WGS84 three-dimensional (EPSG:4979)",
                "local_frame": "ENU tangent frame: +east, +north, +up",
                "conversion": "pyproj EPSG:4979 to ECEF EPSG:4978, then ECEF-to-ENU rotation",
                "pyproj_version": pyproj.__version__,
                "origin_source": origin_source,
                "origin_lat_deg": local_frame.lat_deg if local_frame else None,
                "origin_lon_deg": local_frame.lon_deg if local_frame else None,
                "origin_alt_m": local_frame.alt_m if local_frame else None,
            },
            "frame_and_unit_notes": {
                "gps_velocity": "GLOBAL_POSITION_INT vx=North, vy=East, vz=Down; raw cm/s and SI m/s retained",
                "local_position_ned": "Autopilot LOCAL_POSITION_NED: x=North, y=East, z=Down",
                "vertical_speed": "vz_down_mps is positive down; vertical_speed_up_mps is its negation",
                "heading": "GLOBAL_POSITION_INT hdg and GPS_RAW_INT cog are centidegrees; 65535 means unknown",
                "gps_quality": "GPS_RAW_INT eph/epv raw values and 1e-2 scaled values retained without relabeling them as metric accuracy",
                "radio": "RADIO/RADIO_STATUS values are raw device-reported counters/indicators, not assumed to be dBm or SNR",
                "vehicle_state": "Rows are asynchronous and source_message identifies ATTITUDE, VFR_HUD, or HEARTBEAT; no synthetic joins",
            },
            "message_counts": dict(sorted(counts.items())),
            "output_row_counts": dict(sorted(output_counts.items())),
            "source_endpoints": [
                {"sysid": s, "compid": c, "packet_count": n}
                for (s, c), n in sorted(endpoint_counts.items())
            ],
            "malformed_packet_count": malformed_packets,
            "telemetry_timing_summary": timing_summary,
            "mavlink_sequence_summary": sequence_summary,
            "mavlink_sequence_caveat": (
                "Sequence gaps are diagnostic estimates per (sysid, compid), not proven RF packet loss. "
                "A bidirectional .tlog, multiple MAVLink channels, retransmission, recording loss, stream "
                "resets, and out-of-order packets can affect sequence continuity. No time gap was counted as packet loss."
            ),
            "radio_record_count": counts["RADIO_STATUS"] + counts["RADIO"],
            "waypoints_reached": reached_waypoints,
            "flight_modes_observed": sorted(observed_modes),
            "warnings": warning_messages,
        }
        with (output_dir / "metadata.json").open("w", encoding="utf-8") as stream:
            json.dump(metadata, stream, indent=2, sort_keys=False)
            stream.write("\n")
    finally:
        for handle in handles.values():
            handle.close()

    print(f"Input: {input_path}")
    print(f"Output: {output_dir}")
    print(f"Log duration: {duration_s:.3f} s")
    print(f"GLOBAL_POSITION_INT samples: {counts['GLOBAL_POSITION_INT']}")
    print(f"Effective GPS arrival rate: {timing_summary['effective_rate_hz']:.3f} Hz")
    print(f"GPS arrival gaps >1 s: {gap_counts[1000]}")
    print(f"Radio records: {metadata['radio_record_count']}")
    print(f"Modes observed: {', '.join(sorted(observed_modes)) or 'none'}")
    print("Extraction complete; input .tlog was not modified.")
    return output_dir


def main() -> int:
    args = parse_args()
    try:
        extract(args)
    except (FileNotFoundError, PermissionError, ValueError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

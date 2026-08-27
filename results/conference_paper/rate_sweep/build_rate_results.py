#!/usr/bin/env python3
"""Build one publication-complete Edge/Ground rate-sweep result."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any


TRACE_FIELDS = [
    "run_id", "rng_run", "mode", "frame_rate_hz", "sequence_id",
    "publish_time", "frame_size_bytes", "detector_callback_received",
    "callback_time", "decode_success", "inference_completed",
    "inference_ms", "result_published", "pipeline_latency_ms",
]
SUMMARY_FIELDS = [
    "run_id", "rng_run", "mode", "frame_rate_hz",
    "official_publications", "detector_callbacks", "inference_completions",
    "processed_frame_ratio", "median_pipeline_latency_ms",
    "p95_pipeline_latency_ms", "latency_sample_count", "mean_inference_ms",
    "median_inference_ms", "mean_frame_size_bytes", "tap_uav1_tx_bytes",
    "tap_uav1_tx_packets", "tap_gcs_rx_bytes", "tap_gcs_rx_packets",
    "uav_tx_bitrate_mbps", "gcs_rx_bitrate_mbps",
    "mean_relay_cpu_percent", "mean_detector_cpu_percent",
    "mean_combined_cpu_percent", "callback_delivery_ratio",
    "decode_success_ratio", "mean_jpeg_size_bytes",
]
TAP_INTERFACES = ("tap-uav1", "tap-gcs")
TAP_FIELDS = (
    "rx_bytes", "rx_packets", "rx_errors", "rx_dropped",
    "tx_bytes", "tx_packets", "tx_errors", "tx_dropped",
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build CSV outputs for one Edge/Ground rate-sweep run.")
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--rng-run", required=True, type=int)
    parser.add_argument("--mode", required=True, choices=("edge", "ground"))
    parser.add_argument("--frame-rate-hz", required=True, type=int,
                        choices=(1, 2, 5))
    parser.add_argument("--official-start-time", required=True, type=float)
    parser.add_argument("--official-end-time", required=True, type=float)
    parser.add_argument("--relay-events", required=True, type=Path)
    parser.add_argument("--detector-events", required=True, type=Path)
    parser.add_argument("--pidstat", required=True, type=Path)
    parser.add_argument("--relay-pid", required=True, type=int)
    parser.add_argument("--detector-pid", required=True, type=int)
    parser.add_argument("--tap-before", required=True, type=Path)
    parser.add_argument("--tap-after", required=True, type=Path)
    parser.add_argument("--trace-output", required=True, type=Path)
    parser.add_argument("--summary-output", required=True, type=Path)
    parser.add_argument("--tap-output", required=True, type=Path)
    return parser.parse_args()


def read_events(path: Path) -> list[dict[str, Any]]:
    events = []
    with path.open(encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
            if not line.strip():
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSON in {path}:{line_number}") from exc
    return events


def parse_link_stats(text: str, interface: str) -> dict[str, int]:
    marker = f"--- {interface} ---"
    if marker not in text:
        raise ValueError(f"missing {interface} marker in TAP snapshot")
    block = text.split(marker, 1)[1].split("--- ", 1)[0]
    lines = block.strip().splitlines()
    stats: dict[str, int] = {}
    for index, line in enumerate(lines):
        stripped = line.strip()
        if not (stripped.startswith("RX:") or stripped.startswith("TX:")):
            continue
        direction = stripped[:2].lower()
        labels = stripped.split(":", 1)[1].split()
        if index + 1 >= len(lines):
            raise ValueError(f"missing {interface} {direction} values")
        values = lines[index + 1].split()
        if len(labels) != len(values):
            raise ValueError(f"label/value mismatch for {interface} {direction}")
        for label, value in zip(labels, values):
            stats[f"{direction}_{label}"] = int(value)
    return stats


def tap_deltas(before_path: Path, after_path: Path) -> list[dict[str, Any]]:
    before_text = before_path.read_text(encoding="utf-8")
    after_text = after_path.read_text(encoding="utf-8")
    rows = []
    for interface in TAP_INTERFACES:
        before = parse_link_stats(before_text, interface)
        after = parse_link_stats(after_text, interface)
        row: dict[str, Any] = {"interface": interface}
        for field in TAP_FIELDS:
            if field not in before or field not in after:
                raise ValueError(f"missing {field} for {interface}")
            row[field] = after[field] - before[field]
            if row[field] < 0:
                raise ValueError(f"negative {field} delta for {interface}")
        rows.append(row)
    return rows


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def average(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def pidstat_cpu(path: Path, relay_pid: int, detector_pid: int) -> tuple[float | None, ...]:
    """Read CPU rows; pidstat's CPU value is the third field from the right."""
    samples: dict[str, dict[int, float]] = {}
    wanted = {relay_pid, detector_pid}
    with path.open(encoding="utf-8") as source:
        for line in source:
            parts = line.split()
            if "%CPU" in parts or len(parts) < 8 or parts[0] == "Average:":
                continue
            pid_index = next(
                (i for i, token in enumerate(parts) if token.isdigit()
                 and int(token) in wanted), None)
            if pid_index is None:
                continue
            # CPU rows have seven fields after PID; memory rows emitted by
            # the same `pidstat -u -r` invocation have only six.
            if len(parts) - pid_index - 1 < 7:
                continue
            try:
                pid = int(parts[pid_index])
                cpu = float(parts[-3])
            except (ValueError, IndexError):
                continue
            sample_key = " ".join(parts[:pid_index])
            samples.setdefault(sample_key, {})[pid] = cpu
    relay = [sample[relay_pid] for sample in samples.values() if relay_pid in sample]
    detector = [sample[detector_pid] for sample in samples.values()
                if detector_pid in sample]
    combined = [sample[relay_pid] + sample[detector_pid]
                for sample in samples.values()
                if relay_pid in sample and detector_pid in sample]
    return average(relay), average(detector), average(combined)


def main() -> int:
    args = arguments()
    duration = args.official_end_time - args.official_start_time
    if not 59.0 <= duration <= 61.5:
        raise ValueError(f"official wall-clock duration is not 60 s: {duration:.6f}")
    for output in (args.trace_output, args.summary_output, args.tap_output):
        if output.exists():
            raise FileExistsError(f"refusing to overwrite {output}")

    relay_by_sequence: dict[int, dict[str, Any]] = {}
    for event in read_events(args.relay_events):
        if event.get("event") != "relay_publish":
            continue
        publish_time = float(event["relay_publish_time"])
        if args.official_start_time <= publish_time < args.official_end_time:
            sequence = int(event["sequence_id"])
            if sequence in relay_by_sequence:
                raise ValueError(f"duplicate relay sequence {sequence}")
            relay_by_sequence[sequence] = event

    downstream: dict[int, dict[str, Any]] = {}
    official_sequences = set(relay_by_sequence)
    for event in read_events(args.detector_events):
        sequence = int(event.get("sequence_id", -1))
        if sequence not in official_sequences:
            continue
        record = downstream.setdefault(sequence, {})
        event_type = event.get("event")
        if event_type in ("detector_callback", "gcs_callback"):
            record["detector_callback_received"] = bool(event.get(
                "detector_callback_received", event.get(
                    "gcs_callback_received", False)))
            record["callback_time"] = event.get(
                "callback_time", event.get("gcs_callback_time", ""))
        elif event_type == "decode":
            record["decode_success"] = bool(event["decode_success"])
        elif event_type == "inference_result":
            record["inference_completed"] = bool(event["inference_completed"])
            record["inference_ms"] = event["inference_ms"]
            record["result_published"] = bool(event["result_published"])
            record["pipeline_latency_ms"] = event["pipeline_latency_ms"]

    trace_rows = []
    for sequence, relay in sorted(relay_by_sequence.items()):
        stage = downstream.get(sequence, {})
        trace_rows.append({
            "run_id": args.run_id, "rng_run": args.rng_run,
            "mode": args.mode, "frame_rate_hz": args.frame_rate_hz,
            "sequence_id": sequence,
            "publish_time": relay["relay_publish_time"],
            "frame_size_bytes": relay["frame_size_bytes"],
            "detector_callback_received": stage.get(
                "detector_callback_received", False),
            "callback_time": stage.get("callback_time", ""),
            "decode_success": (stage.get("decode_success", False)
                               if args.mode == "ground" else ""),
            "inference_completed": stage.get("inference_completed", False),
            "inference_ms": stage.get("inference_ms", ""),
            "result_published": stage.get("result_published", False),
            "pipeline_latency_ms": stage.get("pipeline_latency_ms", ""),
        })

    publications = len(trace_rows)
    callbacks = sum(row["detector_callback_received"] for row in trace_rows)
    completions = sum(row["inference_completed"] for row in trace_rows)
    latencies = [float(row["pipeline_latency_ms"]) for row in trace_rows
                 if row["inference_completed"] and row["pipeline_latency_ms"] != ""]
    inferences = [float(row["inference_ms"]) for row in trace_rows
                  if row["inference_completed"] and row["inference_ms"] != ""]
    frame_sizes = [float(row["frame_size_bytes"]) for row in trace_rows]
    decodes = (sum(row["decode_success"] is True for row in trace_rows)
               if args.mode == "ground" else 0)
    taps = tap_deltas(args.tap_before, args.tap_after)
    tap_map = {row["interface"]: row for row in taps}
    relay_cpu, detector_cpu, combined_cpu = pidstat_cpu(
        args.pidstat, args.relay_pid, args.detector_pid)

    summary = {
        "run_id": args.run_id, "rng_run": args.rng_run, "mode": args.mode,
        "frame_rate_hz": args.frame_rate_hz,
        "official_publications": publications,
        "detector_callbacks": callbacks, "inference_completions": completions,
        "processed_frame_ratio": completions / publications if publications else 0,
        "median_pipeline_latency_ms": statistics.median(latencies) if latencies else "",
        "p95_pipeline_latency_ms": (percentile(latencies, 0.95)
                                    if latencies else ""),
        "latency_sample_count": len(latencies),
        "mean_inference_ms": average(inferences) if inferences else "",
        "median_inference_ms": statistics.median(inferences) if inferences else "",
        "mean_frame_size_bytes": average(frame_sizes) if frame_sizes else "",
        "tap_uav1_tx_bytes": tap_map["tap-uav1"]["tx_bytes"],
        "tap_uav1_tx_packets": tap_map["tap-uav1"]["tx_packets"],
        "tap_gcs_rx_bytes": tap_map["tap-gcs"]["rx_bytes"],
        "tap_gcs_rx_packets": tap_map["tap-gcs"]["rx_packets"],
        "uav_tx_bitrate_mbps": tap_map["tap-uav1"]["tx_bytes"] * 8 / 60 / 1e6,
        "gcs_rx_bitrate_mbps": tap_map["tap-gcs"]["rx_bytes"] * 8 / 60 / 1e6,
        "mean_relay_cpu_percent": relay_cpu if relay_cpu is not None else "",
        "mean_detector_cpu_percent": detector_cpu if detector_cpu is not None else "",
        "mean_combined_cpu_percent": combined_cpu if combined_cpu is not None else "",
        "callback_delivery_ratio": (callbacks / publications
                                    if args.mode == "ground" and publications else ""),
        "decode_success_ratio": (decodes / publications
                                 if args.mode == "ground" and publications else ""),
        "mean_jpeg_size_bytes": (average(frame_sizes)
                                 if args.mode == "ground" and frame_sizes else ""),
    }

    args.trace_output.parent.mkdir(parents=True, exist_ok=True)
    with args.trace_output.open("x", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=TRACE_FIELDS)
        writer.writeheader(); writer.writerows(trace_rows)
    with args.tap_output.open("x", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=["interface", *TAP_FIELDS])
        writer.writeheader(); writer.writerows(taps)
    with args.summary_output.open("x", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=SUMMARY_FIELDS)
        writer.writeheader(); writer.writerow(summary)
    print(f"PASS: {publications} official publications, {callbacks} callbacks, "
          f"{completions} inference completions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

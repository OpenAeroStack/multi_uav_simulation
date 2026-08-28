#!/usr/bin/env python3
"""Build publication-complete transport traces and summaries for one run."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path
from typing import Any


TRACE_FIELDS = [
    "run_id", "rng_run", "jpeg_quality", "sequence_id",
    "relay_publish_time", "jpeg_size_bytes", "gcs_callback_received",
    "gcs_callback_time", "decode_success", "decode_time_ms",
    "inference_completed", "inference_ms", "result_published",
    "pipeline_latency_ms",
]
TAP_INTERFACES = ("tap-uav1", "tap-gcs")
TAP_FIELDS = (
    "rx_bytes", "rx_packets", "rx_errors", "rx_dropped",
    "tx_bytes", "tx_packets", "tx_errors", "tx_dropped",
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--rng-run", required=True, type=int)
    parser.add_argument("--jpeg-quality", required=True, type=int)
    parser.add_argument("--official-start", required=True, type=int)
    parser.add_argument("--official-end", required=True, type=int)
    parser.add_argument("--relay-events", required=True, type=Path)
    parser.add_argument("--detector-events", required=True, type=Path)
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
                raise ValueError(f"invalid JSON event in {path}:{line_number}") from exc
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
        direction = "rx" if stripped.startswith("RX:") else "tx"
        labels = stripped.replace(f"{direction.upper()}:", "").split()
        if index + 1 >= len(lines):
            raise ValueError(f"missing values after {interface} {direction} header")
        values = lines[index + 1].strip().split()
        if len(labels) != len(values):
            raise ValueError(f"label/value mismatch for {interface} {direction}")
        for label, value in zip(labels, values):
            try:
                stats[f"{direction}_{label}"] = int(value)
            except ValueError as exc:
                raise ValueError(
                    f"non-integer {interface} {direction}_{label}: {value}"
                ) from exc
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
                raise ValueError(f"missing {field} counter for {interface}")
            delta = after[field] - before[field]
            if delta < 0:
                raise ValueError(f"negative {field} delta for {interface}")
            row[field] = delta
        rows.append(row)
    return rows


def mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def main() -> int:
    args = arguments()
    if args.official_end - args.official_start != 60:
        raise ValueError("official sequence interval must contain exactly 60 IDs")
    for output in (args.trace_output, args.summary_output, args.tap_output):
        if output.exists():
            raise FileExistsError(f"refusing to overwrite {output}")

    relay_by_sequence: dict[int, dict[str, Any]] = {}
    for event in read_events(args.relay_events):
        if event.get("event") != "relay_publish":
            continue
        sequence = int(event["sequence_id"])
        if args.official_start < sequence <= args.official_end:
            if sequence in relay_by_sequence:
                raise ValueError(f"duplicate relay event for sequence {sequence}")
            relay_by_sequence[sequence] = event

    expected_sequences = list(range(args.official_start + 1, args.official_end + 1))
    if sorted(relay_by_sequence) != expected_sequences:
        raise ValueError("relay events do not contain exactly the 60 official sequences")

    downstream: dict[int, dict[str, Any]] = {}
    for event in read_events(args.detector_events):
        sequence = int(event.get("sequence_id", -1))
        if sequence not in relay_by_sequence:
            continue
        record = downstream.setdefault(sequence, {})
        event_type = event.get("event")
        if event_type == "gcs_callback":
            record.update({
                "gcs_callback_received": bool(event["gcs_callback_received"]),
                "gcs_callback_time": event["gcs_callback_time"],
            })
        elif event_type == "decode":
            record.update({
                "decode_success": bool(event["decode_success"]),
                "decode_time_ms": event["decode_time_ms"],
            })
        elif event_type == "inference_result":
            record.update({
                "inference_completed": bool(event["inference_completed"]),
                "inference_ms": event["inference_ms"],
                "result_published": bool(event["result_published"]),
                "pipeline_latency_ms": event.get(
                    "relay_to_inference_completion_ms",
                    event.get("pipeline_latency_ms")),
            })

    trace_rows = []
    for sequence in expected_sequences:
        relay = relay_by_sequence[sequence]
        stage = downstream.get(sequence, {})
        trace_rows.append({
            "run_id": args.run_id,
            "rng_run": args.rng_run,
            "jpeg_quality": args.jpeg_quality,
            "sequence_id": sequence,
            "relay_publish_time": relay["relay_publish_time"],
            "jpeg_size_bytes": relay["jpeg_size_bytes"],
            "gcs_callback_received": stage.get("gcs_callback_received", False),
            "gcs_callback_time": stage.get("gcs_callback_time", ""),
            "decode_success": stage.get("decode_success", False),
            "decode_time_ms": stage.get("decode_time_ms", ""),
            "inference_completed": stage.get("inference_completed", False),
            "inference_ms": stage.get("inference_ms", ""),
            "result_published": stage.get("result_published", False),
            "pipeline_latency_ms": stage.get("pipeline_latency_ms", ""),
        })

    tap_rows = tap_deltas(args.tap_before, args.tap_after)
    callbacks = sum(row["gcs_callback_received"] for row in trace_rows)
    decodes = sum(row["decode_success"] for row in trace_rows)
    inferences = sum(row["inference_completed"] for row in trace_rows)
    latencies = [
        float(row["pipeline_latency_ms"])
        for row in trace_rows if row["inference_completed"]
    ]
    jpeg_sizes = [float(row["jpeg_size_bytes"]) for row in trace_rows]
    summary: dict[str, Any] = {
        "run_id": args.run_id,
        "rng_run": args.rng_run,
        "jpeg_quality": args.jpeg_quality,
        "relay_publications": len(trace_rows),
        "gcs_callbacks": callbacks,
        "decode_successes": decodes,
        "inference_completions": inferences,
        "callback_delivery_ratio": callbacks / len(trace_rows),
        "decode_delivery_ratio": decodes / len(trace_rows),
        "processed_frame_ratio": inferences / len(trace_rows),
        "mean_jpeg_size_bytes": mean(jpeg_sizes),
        "mean_pipeline_latency_ms": mean(latencies),
        "median_pipeline_latency_ms": median(latencies),
    }
    for tap_row in tap_rows:
        prefix = tap_row["interface"].replace("-", "_")
        for field in TAP_FIELDS:
            summary[f"{prefix}_{field}"] = tap_row[field]

    args.trace_output.parent.mkdir(parents=True, exist_ok=True)
    with args.trace_output.open("x", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=TRACE_FIELDS)
        writer.writeheader()
        writer.writerows(trace_rows)
    with args.tap_output.open("x", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=["interface", *TAP_FIELDS])
        writer.writeheader()
        writer.writerows(tap_rows)
    with args.summary_output.open("x", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=list(summary))
        writer.writeheader()
        writer.writerow(summary)

    print(
        f"PASS: {len(trace_rows)} publications, {callbacks} callbacks, "
        f"{decodes} decodes, {inferences} inferences"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

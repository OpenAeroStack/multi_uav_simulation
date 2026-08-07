#!/usr/bin/env python3
"""Build the report-ready Phase C telemetry package from stored CSV files."""

from __future__ import annotations

import csv
import shutil
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]
PHASE = ROOT / "results/phase_c_middleware"
OUT = PHASE / "report_validation_v2"
FIG = OUT / "figures"
EXPORT = ROOT / "report_export/images/chapter5/middleware"
SOURCE_SUMMARY = PHASE / "processed/c_telemetry_summary.csv"

SELECTED = [
    ("C0 mission/no vision", "C0", "012216", "c0_mission_baseline",
     "results/phase_c_middleware/raw/telemetry_c0_mission_baseline_20260805_012216.csv"),
    ("C1 static/no vision", "C1", "001716", "c1_baseline",
     "results/phase_c_middleware/raw/telemetry_c1_baseline_20260805_001716.csv"),
    ("C2 Edge", "C2", "012804", "c2_edge_load",
     "results/phase_c_middleware/raw/telemetry_c2_edge_load_20260805_012804.csv"),
    ("C3 Ground", "C3", "011400", "c3_ground_load",
     "results/phase_c_middleware/raw/telemetry_c3_ground_load_20260805_011400.csv"),
]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def matching_summary(label: str, count: int) -> dict[str, str]:
    matches = [row for row in read_csv(SOURCE_SUMMARY)
               if row["label"] == label and int(row["n_messages"]) == count]
    if len(matches) != 1:
        raise ValueError(f"Expected one summary match for {label}, n={count}; got {len(matches)}")
    return matches[0]


def build_rows() -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    selected_rows = []
    telemetry_rows = []
    for condition, code, identifier, label, relative in SELECTED:
        path = ROOT / relative
        source = read_csv(path)
        timestamps = [float(row["arrival_time_s"]) for row in source]
        if len(timestamps) < 2:
            raise ValueError(f"Selected run does not contain enough messages: {path}")
        gaps = [(current - previous) * 1000.0
                for previous, current in zip(timestamps, timestamps[1:])]
        summary = matching_summary(label, len(timestamps))
        configured_duration = float(summary["duration_s"])
        calculated_rate = len(timestamps) / configured_duration
        if round(calculated_rate, 2) != float(summary["rate_hz"]):
            raise ValueError(f"Rate mismatch for {condition}")
        selected_rows.append({
            "condition": condition,
            "condition_code": code,
            "raw_file": relative,
            "timestamp_identifier": identifier,
            "included": "true",
            "exclusion_reason": "",
            "message_count": len(timestamps),
            "duration_s": f"{configured_duration:.1f}",
            "mean_rate_hz": f"{calculated_rate:.2f}",
            "maximum_gap_ms": f"{max(gaps):.1f}",
        })
        telemetry_rows.append({
            "condition": condition,
            "messages": len(timestamps),
            "configured_duration_s": f"{configured_duration:.1f}",
            "first_timestamp_s": f"{timestamps[0]:.6f}",
            "last_timestamp_s": f"{timestamps[-1]:.6f}",
            "observed_span_s": f"{timestamps[-1] - timestamps[0]:.6f}",
            "mean_rate_hz": f"{calculated_rate:.2f}",
            "minimum_interval_ms": f"{min(gaps):.1f}",
            "mean_interval_ms": f"{sum(gaps) / len(gaps):.1f}",
            "maximum_gap_ms": f"{max(gaps):.1f}",
            "source_file": relative,
        })
    return selected_rows, telemetry_rows


def plot_summary(rows: list[dict[str, object]]) -> None:
    labels = [str(row["condition"]).replace(" mission/no vision", "\nmission/no vision")
              .replace(" static/no vision", "\nstatic/no vision")
              .replace(" Edge", "\nEdge").replace(" Ground", "\nGround") for row in rows]
    rates = [float(row["mean_rate_hz"]) for row in rows]
    gaps = [float(row["maximum_gap_ms"]) for row in rows]
    colours = ["#4E79A7", "#59A14F", "#F28E2B", "#E15759"]

    fig, (top, bottom) = plt.subplots(2, 1, figsize=(8.5, 7.2))
    bars = top.bar(labels, rates, color=colours, width=0.58)
    top.set_ylabel("Mean telemetry rate (Hz)")
    top.set_ylim(0, 2.6)
    top.grid(axis="y", alpha=0.25)
    for bar, value in zip(bars, rates):
        top.text(bar.get_x() + bar.get_width() / 2, value + 0.04, f"{value:.2f}",
                 ha="center", va="bottom", fontsize=9)

    bars = bottom.bar(labels, gaps, color=colours, width=0.58)
    bottom.set_ylabel("Maximum message gap (ms)")
    bottom.set_ylim(0, 4300)
    bottom.grid(axis="y", alpha=0.25)
    for bar, value in zip(bars, gaps):
        bottom.text(bar.get_x() + bar.get_width() / 2, value + 70, f"{value:.1f}",
                    ha="center", va="bottom", fontsize=9)
    bottom.set_xlabel("Selected operating condition")
    fig.suptitle("Telemetry Rate and Maximum Message Gap")
    fig.tight_layout()
    fig.savefig(FIG / "telemetry_rate_and_gap.png", dpi=300)
    fig.savefig(FIG / "telemetry_rate_and_gap.pdf")
    shutil.copy2(FIG / "telemetry_rate_and_gap.png", EXPORT / "telemetry_rate_and_gap.png")
    plt.close(fig)


def main() -> int:
    FIG.mkdir(parents=True, exist_ok=True)
    EXPORT.mkdir(parents=True, exist_ok=True)
    selected_rows, telemetry_rows = build_rows()
    write_csv(OUT / "selected_runs.csv", selected_rows)
    write_csv(OUT / "telemetry_summary.csv", telemetry_rows)
    plot_summary(telemetry_rows)
    print("Phase C telemetry validation v2")
    for row in telemetry_rows:
        print(f"  {row['condition']}: n={row['messages']} rate={row['mean_rate_hz']} Hz "
              f"max_gap={row['maximum_gap_ms']} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

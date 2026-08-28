#!/usr/bin/env python3
"""Validate and summarize one network-only multi-UAV scaling run."""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from pathlib import Path

from iperf_json import parse_udp_json

ROOT = Path(__file__).resolve().parent
PROCESSED = ROOT / "processed"
FINAL = ROOT / "final"
DURATION = 30.0
OFFERED_MBPS = 0.5
TAP_FIELDS = (
    "rx_bytes", "rx_packets", "rx_errors", "rx_dropped",
    "tx_bytes", "tx_packets", "tx_errors", "tx_dropped",
)
PER_UAV_FIELDS = (
    "run_id", "n_active_uav", "rng_run", "uav_id",
    "configured_offered_mbps", "actual_sender_mbps", "sender_bytes",
    "sender_datagrams", "receiver_bytes", "receiver_datagrams",
    "received_goodput_mbps", "lost_datagrams", "packet_loss_ratio",
    "jitter_ms", "tap_tx_bytes", "tap_tx_packets", "tap_rx_bytes",
    "tap_rx_packets", "tap_errors", "tap_drops", "client_start_time",
    "client_end_time",
)
RUN_FIELDS = (
    "run_id", "n_active_uav", "rng_run", "aggregate_offered_mbps",
    "aggregate_sender_mbps", "aggregate_goodput_mbps",
    "aggregate_packet_loss_ratio", "aggregate_uav_tap_tx_mbps",
    "gcs_tap_rx_mbps", "jain_fairness", "official_start_time",
    "official_end_time", "client_start_spread_ms", "minimum_client_duration_s",
    "ns3_sim_elapsed_s", "ns3_wall_elapsed_s", "ns3_realtime_ratio",
    "realtime_interpretation",
)
AGGREGATE_METRICS = (
    "aggregate_sender_mbps", "aggregate_goodput_mbps",
    "aggregate_packet_loss_ratio", "aggregate_uav_tap_tx_mbps",
    "gcs_tap_rx_mbps", "jain_fairness", "ns3_realtime_ratio",
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--n-active-uav", required=True, type=int, choices=(1, 2, 3))
    parser.add_argument("--rng-run", required=True, type=int)
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--tap-before", required=True, type=Path)
    parser.add_argument("--tap-after", required=True, type=Path)
    parser.add_argument("--ns3-log", required=True, type=Path)
    parser.add_argument("--official-start", required=True, type=float)
    parser.add_argument("--official-end", required=True, type=float)
    return parser.parse_args()


def read_taps(path: Path) -> dict[str, dict[str, int]]:
    with path.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    return {
        row["interface"]: {field: int(row[field]) for field in TAP_FIELDS}
        for row in rows
    }


def tap_deltas(before_path: Path, after_path: Path) -> dict[str, dict[str, int]]:
    before, after = read_taps(before_path), read_taps(after_path)
    expected = {"tap-gcs", "tap-uav1", "tap-uav2", "tap-uav3"}
    if set(before) != expected or set(after) != expected:
        raise ValueError("TAP snapshots do not contain exactly the four expected interfaces")
    output = {}
    for interface in sorted(expected):
        output[interface] = {}
        for field in TAP_FIELDS:
            delta = after[interface][field] - before[interface][field]
            if delta < 0:
                raise ValueError(f"negative TAP delta: {interface} {field}")
            output[interface][field] = delta
    return output


def write_csv(path: Path, fields: tuple[str, ...] | list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def ns3_progress(path: Path, wall_elapsed: float) -> tuple[float | str, float | str]:
    values = [float(value) for value in re.findall(
        r"(?:^|\s)t=([0-9]+(?:\.[0-9]+)?)s", path.read_text(errors="replace"), re.MULTILINE)]
    if len(values) < 2 or wall_elapsed <= 0:
        return "", ""
    elapsed = values[-1] - values[0]
    return elapsed, elapsed / wall_elapsed


def rebuild_final() -> None:
    summaries = []
    for path in sorted(PROCESSED.glob("run_summary_*.csv")):
        with path.open(newline="", encoding="utf-8") as source:
            rows = list(csv.DictReader(source))
        if len(rows) != 1:
            raise ValueError(f"expected one summary row in {path}")
        summaries.append(rows[0])
    if not summaries:
        return

    FINAL.mkdir(parents=True, exist_ok=True)
    all_runs = FINAL / "all_runs.csv"
    with all_runs.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=RUN_FIELDS)
        writer.writeheader(); writer.writerows(sorted(
            summaries, key=lambda row: (int(row["n_active_uav"]), int(row["rng_run"]))))

    aggregate_rows = []
    fields = ["n_active_uav", "run_count", "aggregate_offered_mbps"]
    for metric in AGGREGATE_METRICS:
        fields.extend((f"{metric}_mean", f"{metric}_sd"))
    for count in (1, 2, 3):
        group = [row for row in summaries if int(row["n_active_uav"]) == count]
        if not group:
            continue
        record = {
            "n_active_uav": count, "run_count": len(group),
            "aggregate_offered_mbps": count * OFFERED_MBPS,
        }
        for metric in AGGREGATE_METRICS:
            values = [float(row[metric]) for row in group if row[metric] != ""]
            record[f"{metric}_mean"] = statistics.fmean(values) if values else ""
            record[f"{metric}_sd"] = statistics.stdev(values) if len(values) > 1 else ""
        aggregate_rows.append(record)
    aggregate_path = FINAL / "aggregate_by_uav_count.csv"
    with aggregate_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader(); writer.writerows(aggregate_rows)

    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("WARNING: matplotlib unavailable; final CSVs generated without plots")
        return

    plt.rcParams.update({"font.size": 10, "figure.dpi": 150, "savefig.dpi": 300})
    colors = {1: "#0072B2", 2: "#D55E00", 3: "#009E73"}

    def finish(fig, axis, name: str) -> None:
        axis.grid(True, axis="y", alpha=0.25)
        fig.tight_layout()
        fig.savefig(FINAL / f"{name}.png")
        fig.savefig(FINAL / f"{name}.pdf")
        plt.close(fig)

    counts = sorted({int(row["n_active_uav"]) for row in summaries})
    fig, ax = plt.subplots(figsize=(6.2, 4.4))
    offered = [count * OFFERED_MBPS for count in counts]
    ax.plot(counts, offered, "--", color="black", marker="s", label="Offered load")
    for count in counts:
        vals = [float(row["aggregate_goodput_mbps"]) for row in summaries
                if int(row["n_active_uav"]) == count]
        ax.scatter([count] * len(vals), vals, facecolors="none", edgecolors=colors[count], zorder=3)
        mean = statistics.fmean(vals); sd = statistics.stdev(vals) if len(vals) > 1 else 0
        ax.errorbar(count, mean, yerr=sd, fmt="o", color=colors[count], capsize=4)
    ax.plot([], [], "o", color="#0072B2", label="Goodput mean ± SD")
    ax.set(xlabel="Active UAV traffic sources", ylabel="Aggregate rate (Mbit/s)", xticks=counts)
    ax.legend()
    finish(fig, ax, "aggregate_offered_and_goodput")

    for metric, ylabel, name in (
        ("aggregate_packet_loss_ratio", "Aggregate packet-loss ratio", "aggregate_packet_loss"),
        ("jain_fairness", "Jain fairness index", "jain_fairness"),
    ):
        fig, ax = plt.subplots(figsize=(6.2, 4.4))
        for count in counts:
            vals = [float(row[metric]) for row in summaries if int(row["n_active_uav"]) == count]
            ax.scatter([count] * len(vals), vals, facecolors="none", edgecolors=colors[count])
            mean = statistics.fmean(vals); sd = statistics.stdev(vals) if len(vals) > 1 else 0
            ax.errorbar(count, mean, yerr=sd, fmt="o", color=colors[count], capsize=4)
        ax.set(xlabel="Active UAV traffic sources", ylabel=ylabel, xticks=counts)
        finish(fig, ax, name)

    per_uav_rows = []
    for path in sorted(PROCESSED.glob("per_uav_*.csv")):
        with path.open(newline="", encoding="utf-8") as source:
            per_uav_rows.extend(csv.DictReader(source))
    fig, ax = plt.subplots(figsize=(6.4, 4.5))
    offsets = {1: -0.10, 2: 0.0, 3: 0.10}
    for uav in (1, 2, 3):
        xs, ys = [], []
        for row in per_uav_rows:
            if int(row["uav_id"]) == uav:
                xs.append(int(row["n_active_uav"]) + offsets[uav])
                ys.append(float(row["received_goodput_mbps"]))
        if xs:
            ax.scatter(xs, ys, facecolors="none", edgecolors=colors[uav], label=f"UAV{uav}")
        for count in counts:
            vals = [float(row["received_goodput_mbps"]) for row in per_uav_rows
                    if int(row["uav_id"]) == uav
                    and int(row["n_active_uav"]) == count]
            if vals:
                mean = statistics.fmean(vals)
                sd = statistics.stdev(vals) if len(vals) > 1 else 0
                ax.errorbar(count + offsets[uav], mean, yerr=sd, fmt="o",
                            color=colors[uav], capsize=3, zorder=4)
    ax.set(xlabel="Active UAV traffic sources", ylabel="Per-UAV received goodput (Mbit/s)", xticks=counts)
    ax.legend()
    finish(fig, ax, "per_uav_received_goodput")


def main() -> int:
    args = arguments()
    if not math.isclose(args.official_end - args.official_start, DURATION, abs_tol=0.01):
        raise ValueError("official interval is not exactly 30 seconds")
    per_path = PROCESSED / f"per_uav_{args.run_id}.csv"
    run_path = PROCESSED / f"run_summary_{args.run_id}.csv"
    tap_path = PROCESSED / f"tap_deltas_{args.run_id}.csv"
    for output in (per_path, run_path, tap_path):
        if output.exists():
            raise FileExistsError(f"refusing to overwrite {output}")

    taps = tap_deltas(args.tap_before, args.tap_after)
    rows, starts, durations = [], [], []
    for uav in range(1, args.n_active_uav + 1):
        metrics = parse_udp_json(args.raw_dir / f"iperf_uav{uav}.json")
        metrics.pop("sender_schema")
        metrics.pop("receiver_schema")
        start = float((args.raw_dir / f"client_uav{uav}_start.txt").read_text().strip())
        end = float((args.raw_dir / f"client_uav{uav}_end.txt").read_text().strip())
        starts.append(start); durations.append(end - start)
        tap = taps[f"tap-uav{uav}"]
        rows.append({
            "run_id": args.run_id, "n_active_uav": args.n_active_uav,
            "rng_run": args.rng_run, "uav_id": uav,
            "configured_offered_mbps": OFFERED_MBPS, **metrics,
            "tap_tx_bytes": tap["tx_bytes"], "tap_tx_packets": tap["tx_packets"],
            "tap_rx_bytes": tap["rx_bytes"], "tap_rx_packets": tap["rx_packets"],
            "tap_errors": tap["rx_errors"] + tap["tx_errors"],
            "tap_drops": tap["rx_dropped"] + tap["tx_dropped"],
            "client_start_time": start, "client_end_time": end,
        })

    spread = (max(starts) - min(starts)) * 1000.0
    minimum_duration = min(durations)
    if spread > 50.0:
        raise ValueError(f"client starts did not overlap adequately: spread={spread:.3f} ms")
    if minimum_duration < 29.5:
        raise ValueError(f"incomplete client duration: {minimum_duration:.3f} s")
    if min(starts) < args.official_start - 0.02 or max(starts) > args.official_start + 0.05:
        raise ValueError("clients did not start at the shared official boundary")

    sent_packets = sum(int(row["sender_datagrams"]) for row in rows)
    lost_packets = sum(int(row["lost_datagrams"]) for row in rows)
    goodputs = [float(row["received_goodput_mbps"]) for row in rows]
    fairness = sum(goodputs) ** 2 / (len(goodputs) * sum(value ** 2 for value in goodputs))
    wall_start = float(dict(line.strip().split("=", 1) for line in
        (args.raw_dir / "metadata.txt").read_text().splitlines() if "=" in line)["ns3_wall_start_epoch"])
    wall_elapsed = max(starts) + max(durations) - wall_start
    sim_elapsed, realtime_ratio = ns3_progress(args.ns3_log, wall_elapsed)
    interpretation = ("unavailable" if realtime_ratio == "" else
                      "material_realtime_lag" if float(realtime_ratio) < 0.90 else "realtime_progress_acceptable")
    gcs = taps["tap-gcs"]
    summary = {
        "run_id": args.run_id, "n_active_uav": args.n_active_uav, "rng_run": args.rng_run,
        "aggregate_offered_mbps": args.n_active_uav * OFFERED_MBPS,
        "aggregate_sender_mbps": sum(float(row["actual_sender_mbps"]) for row in rows),
        "aggregate_goodput_mbps": sum(goodputs),
        "aggregate_packet_loss_ratio": lost_packets / sent_packets,
        "aggregate_uav_tap_tx_mbps": sum(taps[f"tap-uav{uav}"]["tx_bytes"] for uav in range(1, args.n_active_uav + 1)) * 8 / DURATION / 1e6,
        "gcs_tap_rx_mbps": gcs["rx_bytes"] * 8 / DURATION / 1e6,
        "jain_fairness": fairness,
        "official_start_time": args.official_start, "official_end_time": args.official_end,
        "client_start_spread_ms": spread, "minimum_client_duration_s": minimum_duration,
        "ns3_sim_elapsed_s": sim_elapsed, "ns3_wall_elapsed_s": wall_elapsed,
        "ns3_realtime_ratio": realtime_ratio, "realtime_interpretation": interpretation,
    }
    write_csv(per_path, list(PER_UAV_FIELDS), rows)
    tap_rows = [{"interface": interface, **values} for interface, values in sorted(taps.items())]
    write_csv(tap_path, ["interface", *TAP_FIELDS], tap_rows)
    write_csv(run_path, list(RUN_FIELDS), [summary])
    rebuild_final()
    print(f"PASS: {args.run_id}: {args.n_active_uav} simultaneous flows, start spread={spread:.3f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

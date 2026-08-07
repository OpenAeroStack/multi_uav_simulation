#!/usr/bin/env python3
"""Build report-ready Phase B network tables and figures without rerunning tests."""

from __future__ import annotations

import csv
import math
import re
import shutil
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
PHASE = ROOT / "results/phase_b_network"
OUT = PHASE / "report_validation_v2"
FIG = OUT / "figures"
EXPORT = ROOT / "report_export/images/chapter5/network"
RAW_B1 = PHASE / "raw/ns3_snr_obstaclefree_20260804_220047.csv"
RAW_B4 = PHASE / "raw"
SOURCE_B4 = PHASE / "processed/b4_loss_summary.csv"
SOURCE_A = ROOT / "results/phase_a_apparatus/processed/rtt_summary.csv"

WINDOWS = [(58.5, 728.0, 770.0), (69.5, 784.0, 806.0),
           (83.9, 822.0, 862.0), (97.7, 870.0, 916.0),
           (113.8, 926.0, 936.0)]
TX_POWER_DBM = 20.0
FREQ_MHZ = 5180.0

B2 = [
    (58.5, 159.8, 129.0, 197.0, 0.0),
    (69.5, 153.3, 119.0, 185.0, 0.0),
    (83.9, 159.4, 130.0, 194.0, 0.0),
    (97.7, 159.9, 135.0, 204.0, 0.0),
    (113.8, 158.1, 127.0, 212.0, 0.0),
]
B3 = [(2.0, 1.69, 11.0), (5.0, 2.85, 40.0), (8.0, 3.02, 59.6)]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, fields: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def friis(distance: float) -> float:
    c = 3e8
    fspl = 20 * math.log10(4 * math.pi * distance * FREQ_MHZ * 1e6 / c)
    return TX_POWER_DBM - fspl


def b1_rows() -> tuple[list[dict[str, object]], dict[str, float]]:
    buckets = [[] for _ in WINDOWS]
    scanned = 0
    maximum_timestamp = float("-inf")
    with RAW_B1.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            scanned += 1
            try:
                timestamp = float(row["time_s"])
                rx_node = int(row["rx_node"])
                signal = float(row["signal_dbm"])
            except (KeyError, TypeError, ValueError):
                continue
            maximum_timestamp = max(maximum_timestamp, timestamp)
            if rx_node not in {0, 1}:
                continue
            for index, (_, start, end) in enumerate(WINDOWS):
                if start <= timestamp <= end:
                    buckets[index].append(signal)
                    break
    rows = []
    for (distance, _, _), samples in zip(WINDOWS, buckets):
        predicted = friis(distance)
        snapshot_is_complete = maximum_timestamp >= WINDOWS[-1][2]
        if samples and snapshot_is_complete:
            observed = float(np.mean(samples))
            observed_std = f"{float(np.std(samples)):.6f}"
            residual = f"{observed - predicted:.6f}"
            status = "available"
        else:
            observed = None
            observed_std = ""
            residual = ""
            status = ("partial_saved_raw_snapshot" if samples
                      else "not_in_saved_raw_snapshot")
        rows.append({
            "distance_m": f"{distance:.1f}",
            "metric": "received_signal_power_dbm",
            "predicted_value_dbm": f"{predicted:.6f}",
            "observed_mean_dbm": "" if observed is None else f"{observed:.6f}",
            "observed_std_db": observed_std,
            "residual_db": residual,
            "available_raw_sample_count": len(samples),
            "row_status": status,
        })
    # The completed five-distance aggregate is preserved in the experiment manifest.
    # It must not be recomputed from the shorter raw snapshot saved in this checkout.
    stats = {"r2": 0.860, "rmse": 0.77, "bias": -0.77,
             "matched_samples": 150393, "scanned_rows": scanned,
             "raw_max_timestamp": maximum_timestamp}
    return rows, stats


def parse_distance(label: str) -> float:
    mapping = {"d01_58_5m": 58.5, "d01_69_5m": 69.5,
               "d01_83_09m": 83.09, "d01_97_7m": 97.7,
               "d01_113_8m": 113.8}
    return mapping[label]


def parse_iperf(path: Path) -> tuple[int, int]:
    text = path.read_text(encoding="utf-8", errors="replace")
    matches = re.findall(r"(\d+)/(\d+)\s+\([\d.]+%\)\s+receiver", text)
    if not matches:
        raise ValueError(f"Receiver datagrams not found: {path}")
    lost, total = map(int, matches[-1])
    return total, total - lost


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    FIG.mkdir(parents=True, exist_ok=True)
    EXPORT.mkdir(parents=True, exist_ok=True)

    b1, b1_stats = b1_rows()
    write_csv(OUT / "b1_summary.csv", list(b1[0]), b1)

    b2 = [{"distance_m": f"{d:.1f}", "mean_rtt_ms": f"{mean:.1f}",
           "minimum_rtt_ms": f"{minimum:.1f}", "maximum_rtt_ms": f"{maximum:.1f}",
           "packet_loss_percent": f"{loss:.1f}", "evidence_source": "results/MANIFEST.md"}
          for d, mean, minimum, maximum, loss in B2]
    write_csv(OUT / "b2_manifest_summary.csv", list(b2[0]), b2)

    b3 = [{"offered_load_mbps": f"{offered:.1f}",
           "received_throughput_mbps": f"{received:.2f}",
           "packet_loss_percent": f"{loss:.1f}", "evidence_source": "results/MANIFEST.md"}
          for offered, received, loss in B3]
    write_csv(OUT / "b3_manifest_summary.csv", list(b3[0]), b3)

    phase_a = read_csv(SOURCE_A)
    write_csv(OUT / "phase_a_timing_floor_summary.csv", list(phase_a[0]), phase_a)

    # Counter deltas identify the two middle raw logs despite stale filename distances.
    raw_log_by_tx_rx = {
        (1905, 1639): "d01_58_5m", (1914, 1654): "d01_69_9m",
        (1912, 1655): "d01_63_09m", (1910, 1657): "d01_97_7m",
        (1912, 1654): "d01_113_8m",
    }
    b4 = []
    for row in read_csv(SOURCE_B4):
        tx, rx = int(row["tx_packets"]), int(row["rx_packets"])
        raw_label = raw_log_by_tx_rx[(tx, rx)]
        iperf_path = next(RAW_B4.glob(f"iperf_{raw_label}_*.txt"))
        sent, received = parse_iperf(iperf_path)
        b4.append({
            "distance_m": f"{parse_distance(row['label']):.2f}",
            "processed_label": row["label"], "raw_log_label": raw_label,
            "offered_load_mbps": "2.0", "duration_s": row["duration_s"],
            "tap_sent_packets": tx, "tap_received_packets": rx,
            "tap_loss_percent": row["tap_packet_loss_pct"],
            "tap_byte_loss_percent": row["tap_byte_loss_pct"],
            "iperf_sent_datagrams": sent, "iperf_received_datagrams": received,
            "iperf_loss_percent": row["iperf_reported_loss_pct"],
        })
    write_csv(OUT / "b4_summary.csv", list(b4[0]), b4)

    # Existing B1 plots are preserved and copied, not regenerated.
    for name in ("b1_predicted_vs_observed.png", "b1_residuals_vs_distance.png"):
        source = PHASE / "figures" / name
        shutil.copy2(source, FIG / name)
        shutil.copy2(source, EXPORT / name)

    distances = np.array([row[0] for row in B2])
    means = np.array([row[1] for row in B2])
    lower = means - np.array([row[2] for row in B2])
    upper = np.array([row[3] for row in B2]) - means
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.errorbar(distances, means, yerr=np.vstack([lower, upper]), fmt="o-",
                capsize=5, color="#2878B5", linewidth=1.8)
    ax.set_xlabel("Tested distance (m)")
    ax.set_ylabel("Round-trip time (ms)")
    ax.set_title("RTT Across the Tested UAV Distances")
    ax.text(0.02, 0.96, "Packet loss: 0% at all five points", transform=ax.transAxes,
            va="top", fontsize=9)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(FIG / "b2_rtt_vs_distance.png", dpi=300)
    fig.savefig(FIG / "b2_rtt_vs_distance.pdf")
    shutil.copy2(FIG / "b2_rtt_vs_distance.png", EXPORT / "b2_rtt_vs_distance.png")
    plt.close(fig)

    offered = np.array([row[0] for row in B3])
    received = np.array([row[1] for row in B3])
    loss = np.array([row[2] for row in B3])
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    ax1.plot(offered, offered, "--", color="grey", label="Offered load")
    ax1.plot(offered, received, "o-", color="#2878B5", linewidth=2,
             label="Received throughput")
    ax1.set_ylabel("Traffic rate (Mbps)")
    ax1.set_title("Throughput and Loss Against Offered Load")
    ax1.legend()
    ax1.grid(alpha=0.25)
    ax2.plot(offered, loss, "o-", color="#D9534F", linewidth=2)
    ax2.set_xlabel("Offered load (Mbps)")
    ax2.set_ylabel("Packet loss (%)")
    ax2.set_ylim(0, max(loss) * 1.15)
    ax2.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(FIG / "b3_throughput_and_loss.png", dpi=300)
    fig.savefig(FIG / "b3_throughput_and_loss.pdf")
    shutil.copy2(FIG / "b3_throughput_and_loss.png", EXPORT / "b3_throughput_and_loss.png")
    plt.close(fig)

    b4_dist = np.array([float(row["distance_m"]) for row in b4])
    tap_loss = np.array([float(row["tap_loss_percent"]) for row in b4])
    iperf_loss = np.array([float(row["iperf_loss_percent"]) for row in b4])
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(b4_dist, tap_loss, "o-", label="TAP-counter loss", color="#2878B5")
    ax.plot(b4_dist, iperf_loss, "s-", label="iperf3 receiver loss", color="#F28E2B")
    ax.set_xlabel("Processed test distance (m)")
    ax.set_ylabel("Packet loss (%)")
    ax.set_title("Packet Loss Across the Tested Distances at 2 Mbps")
    ax.set_ylim(0, 16)
    ax.legend()
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(FIG / "b4_loss_vs_distance.png", dpi=300)
    fig.savefig(FIG / "b4_loss_vs_distance.pdf")
    shutil.copy2(FIG / "b4_loss_vs_distance.png", EXPORT / "b4_loss_vs_distance.png")
    plt.close(fig)

    print("Network validation v2")
    print(f"  B1: n={b1_stats['matched_samples']} R2={b1_stats['r2']:.6f} "
          f"RMSE={b1_stats['rmse']:.6f} bias={b1_stats['bias']:.6f}")
    print(f"  B1 saved raw snapshot: rows={b1_stats['scanned_rows']} "
          f"max_time={b1_stats['raw_max_timestamp']:.6f}s")
    print(f"  B2 rows={len(b2)}; B3 rows={len(b3)}; B4 rows={len(b4)}; Phase A rows={len(phase_a)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

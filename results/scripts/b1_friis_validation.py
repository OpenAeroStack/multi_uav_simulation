#!/usr/bin/env python3
"""
b1_friis_validation.py — Phase B1: predicted vs observed signal strength.

Reads real per-packet SNR log entries within known, confirmed distance
windows (from /tmp/ns3_single.log's live-tracked positions), compares the
observed signal_dbm against the Friis free-space path loss prediction at
each real distance, and produces:
  - a predicted vs observed plot (V1 style, with 1:1 reference line)
  - a residual plot (V2 style, observed - predicted vs distance)
  - R², RMSE, mean bias
  - a results table ready to paste into a report

The SNR log has NO distance column, so distance windows must be supplied
manually — each one is a real, confirmed (distance, t_start, t_end) triple
you got by watching /tmp/ns3_single.log live while holding position and
recording the ping test in that same window. Do not use estimated/computed
distances here — this script exists specifically to avoid that risk.

Usage: edit the WINDOWS list below with your own confirmed data, then run:
    python3 b1_friis_validation.py /tmp/ns3_snr.csv
"""
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# ── EDIT THIS with your own confirmed (distance_m, t_start_s, t_end_s) ──────
WINDOWS = [
    (58.5,  728.0,  770.0),
    (69.5,  784.0,  806.0),
    (83.9,  822.0,  862.0),
    (97.7,  870.0,  916.0),
    (113.8, 926.0,  936.0),
]

# ── Friis parameters, source-confirmed against the scenario ─────────────────
TX_POWER_DBM = 20.0     # three_uav_tapbridge_integrated.cc, confirmed default
FREQ_MHZ = 5180.0       # ns-3 3.38 default channel for WIFI_STANDARD_80211a,
                         # confirmed against installed ns-3 source's channel table
TX_GAIN_DB = 0.0        # ns-3 3.38 default, no override in scenario
RX_GAIN_DB = 0.0        # ns-3 3.38 default, no override in scenario

# Only count packets on the actual GCS<->UAV1 link (node 0 and node 1),
# so any incidental traffic involving the frozen placeholder UAV2/UAV3 nodes
# doesn't contaminate the measurement.
RELEVANT_NODES = {0, 1}


def friis_predicted_dbm(distance_m, tx_power_dbm=TX_POWER_DBM, freq_mhz=FREQ_MHZ,
                          tx_gain_db=TX_GAIN_DB, rx_gain_db=RX_GAIN_DB):
    f_hz = freq_mhz * 1e6
    c = 3e8
    fspl_db = 20 * math.log10(4 * math.pi * distance_m * f_hz / c)
    return tx_power_dbm + tx_gain_db + rx_gain_db - fspl_db


def load_windowed_signal(csv_path, windows):
    """Single streaming pass over the (potentially very large) SNR log,
    bucketing signal_dbm readings into each confirmed distance window."""
    buckets = {i: [] for i in range(len(windows))}
    n_rows_seen = 0
    n_rows_matched = 0

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            n_rows_seen += 1
            try:
                t = float(row["time_s"])
                rx = int(row["rx_node"])
                signal = float(row["signal_dbm"])
            except (ValueError, KeyError):
                continue
            if rx not in RELEVANT_NODES:
                continue
            for i, (dist, t0, t1) in enumerate(windows):
                if t0 <= t <= t1:
                    buckets[i].append(signal)
                    n_rows_matched += 1
                    break

    print(f"Scanned {n_rows_seen} rows in {csv_path}, matched {n_rows_matched} "
          f"to a confirmed distance window (nodes {RELEVANT_NODES}).")
    return buckets


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 b1_friis_validation.py /path/to/ns3_snr.csv",
              file=sys.stderr)
        sys.exit(1)
    csv_path = sys.argv[1]
    results_root = os.path.expanduser("~/FYP/multi_uav_simulation/results")

    buckets = load_windowed_signal(csv_path, WINDOWS)

    distances, observed_mean, observed_std, predicted, n_samples = [], [], [], [], []

    print(f"\n{'Distance (m)':>13} {'n':>5} {'Observed (dBm)':>18} "
          f"{'Predicted (dBm)':>18} {'Residual (dB)':>15}")
    for i, (dist, t0, t1) in enumerate(WINDOWS):
        samples = buckets[i]
        if not samples:
            print(f"{dist:>13.1f}  ** NO SAMPLES FOUND in window {t0}-{t1}s — "
                  f"check the time window or rx_node filter **")
            continue
        obs_mean = float(np.mean(samples))
        obs_std = float(np.std(samples))
        pred = friis_predicted_dbm(dist)
        residual = obs_mean - pred

        distances.append(dist)
        observed_mean.append(obs_mean)
        observed_std.append(obs_std)
        predicted.append(pred)
        n_samples.append(len(samples))

        print(f"{dist:>13.1f} {len(samples):>5d} {obs_mean:>18.2f} "
              f"{pred:>18.2f} {residual:>15.2f}")

    if len(distances) < 2:
        print("\nERROR: fewer than 2 valid windows had data — cannot fit a "
              "meaningful comparison. Check your time windows.", file=sys.stderr)
        sys.exit(1)

    distances = np.array(distances)
    observed_mean = np.array(observed_mean)
    predicted = np.array(predicted)
    residuals = observed_mean - predicted

    # R^2 of observed vs predicted (how well predicted explains observed)
    ss_res = np.sum(residuals ** 2)
    ss_tot = np.sum((observed_mean - np.mean(observed_mean)) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    rmse = math.sqrt(np.mean(residuals ** 2))
    mean_bias = float(np.mean(residuals))

    print(f"\n=== Summary ===")
    print(f"R^2:        {r2:.4f}")
    print(f"RMSE:       {rmse:.2f} dB")
    print(f"Mean bias:  {mean_bias:+.2f} dB "
          f"({'observed higher than predicted' if mean_bias > 0 else 'observed lower than predicted'})")

    # ── Figure 1: predicted vs observed with 1:1 line (V1) ─────────────────
    fig1, ax1 = plt.subplots(figsize=(7, 7))
    lims = [min(predicted.min(), observed_mean.min()) - 3,
            max(predicted.max(), observed_mean.max()) + 3]
    ax1.plot(lims, lims, "k--", alpha=0.5, label="1:1 (perfect agreement)")
    ax1.errorbar(predicted, observed_mean, yerr=observed_std, fmt="o",
                 color="tab:blue", capsize=4, markersize=8,
                 label="measured points (error bars = std within window)")
    for d, p, o in zip(distances, predicted, observed_mean):
        ax1.annotate(f"{d:.0f}m", (p, o), textcoords="offset points",
                     xytext=(6, 6), fontsize=8)
    ax1.set_xlabel("Predicted signal (Friis), dBm")
    ax1.set_ylabel("Observed signal (ns-3), dBm")
    ax1.set_title(f"Predicted vs observed signal strength\n"
                  f"R²={r2:.3f}, RMSE={rmse:.2f} dB, bias={mean_bias:+.2f} dB")
    ax1.legend()
    ax1.grid(alpha=0.3)
    ax1.set_xlim(lims); ax1.set_ylim(lims)
    ax1.set_aspect("equal")

    # ── Figure 2: residuals vs distance (V2) ────────────────────────────────
    fig2, ax2 = plt.subplots(figsize=(9, 5))
    ax2.axhline(0, color="black", linewidth=0.8)
    ax2.plot(distances, residuals, "o-", color="tab:red", markersize=8)
    for d, r in zip(distances, residuals):
        ax2.annotate(f"{r:+.1f}", (d, r), textcoords="offset points",
                     xytext=(6, 6), fontsize=8)
    ax2.set_xlabel("Distance (m)")
    ax2.set_ylabel("Residual (observed - predicted), dB")
    ax2.set_title("Residuals vs distance — checking for systematic trend")
    ax2.grid(alpha=0.3)

    out_dir = os.path.join(results_root, "phase_b_network", "figures")
    os.makedirs(out_dir, exist_ok=True)
    p1 = os.path.join(out_dir, "b1_predicted_vs_observed.png")
    p2 = os.path.join(out_dir, "b1_residuals_vs_distance.png")
    fig1.tight_layout(); fig1.savefig(p1, dpi=150)
    fig2.tight_layout(); fig2.savefig(p2, dpi=150)
    print(f"\nFigures saved:\n  {p1}\n  {p2}")


if __name__ == "__main__":
    main()
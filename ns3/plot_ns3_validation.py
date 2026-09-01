#!/usr/bin/env python3
"""
Model-validation plots for the three-UAV NS-3 propagation chain.

Reads the CSV produced by three_uav_tapbridge_obstacle_loss.cc
(run with --csvPath=/path/metrics.csv) and emits presentation-ready PNGs that
show each loss model is contributing correctly, ending in per-link SNR.

Layers:
  1. Path-loss validation   -> pathloss_rx vs distance vs theoretical FSPL line
  2. Fading validation      -> histogram of the fading term vs Nakagami-m PDF
  3. Composite over mission  -> SNR (and obstacle loss) vs time, per link

Usage:
  python3 plot_ns3_validation.py metrics.csv [--outdir ./figs]

Deps: numpy, pandas, matplotlib  (no scipy needed).
"""
import argparse
import math
import os

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── Must match the NS-3 channel config in three_uav_tapbridge_obstacle_loss.cc
TX_POWER_DBM   = 20.0
REF_LOSS_DB    = 46.67   # LogDistance ReferenceLoss  (FSPL at 1 m, ~5 GHz)
REF_DIST_M     = 1.0     # LogDistance ReferenceDistance
PATHLOSS_EXP   = 2.0     # LogDistance Exponent (n)


def link_label(a, b):
    return f"UAV{int(a)}–UAV{int(b)}"


def theoretical_pathloss_rx(d):
    """Deterministic log-distance rx power (dBm) the model should produce."""
    d = np.asarray(d, dtype=float)
    return TX_POWER_DBM - (REF_LOSS_DB + 10.0 * PATHLOSS_EXP
                           * np.log10(np.maximum(d, 1e-9) / REF_DIST_M))


def gamma_pdf(x, m):
    """PDF of the normalized fading power x = fadedW/meanW ~ Gamma(m, 1/m)."""
    x = np.asarray(x, dtype=float)
    out = np.zeros_like(x)
    pos = x > 0
    out[pos] = (m ** m) * (x[pos] ** (m - 1)) * np.exp(-m * x[pos]) / math.gamma(m)
    return out


def plot_pathloss(df, outdir):
    """Layer 1: path-loss-only rx vs distance, overlaid with the FSPL line."""
    fig, ax = plt.subplots(figsize=(8, 5))
    for (a, b), g in df.groupby(["node_a", "node_b"]):
        ax.scatter(g["distance_m"].to_numpy(), g["pathloss_rx_dbm"].to_numpy(),
                   s=10, alpha=0.5, label=link_label(a, b))
    dline = np.logspace(np.log10(max(df["distance_m"].min(), 1.0)),
                        np.log10(df["distance_m"].max()), 200)
    ax.plot(dline, theoretical_pathloss_rx(dline), "k--", lw=2,
            label=f"Theory (FSPL, n={PATHLOSS_EXP:g})")
    ax.set_xscale("log")
    ax.set_xlabel("Link distance (m)")
    ax.set_ylabel("Path-loss-only Rx power (dBm)")
    ax.set_title("Layer 1 — Log-distance path loss vs theory (−20 dB/decade)")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    _save(fig, outdir, "01_pathloss_vs_distance.png")


def plot_fading(df, outdir):
    """Layer 1: fading term distribution vs the Nakagami-m PDF, per regime."""
    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.linspace(0.01, 4.0, 400)
    for blocked, name in [(0, "LoS"), (1, "NLoS")]:
        g = df[df["blocked"] == blocked]
        if g.empty:
            continue
        # linear fading factor = fadedW / deterministicW = 10^(dB/10)
        factor = 10.0 ** (g["fading_delta_db"].to_numpy() / 10.0)
        m = float(np.median(g["fading_m"]))
        ax.hist(factor, bins=60, density=True, alpha=0.4,
                label=f"{name} samples (n={len(g)})")
        ax.plot(x, gamma_pdf(x, m), lw=2,
                label=f"{name} theory: Nakagami m={m:g}")
    ax.set_xlabel("Normalized fading power  (linear, mean = 1)")
    ax.set_ylabel("Probability density")
    ax.set_title("Layer 1 — Nakagami fading: samples vs theoretical PDF")
    ax.grid(True, alpha=0.3)
    ax.legend()
    _save(fig, outdir, "02_fading_distribution.png")


def plot_snr_time(df, outdir):
    """Layer 2/3: composite SNR over the mission, per link."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for (a, b), g in df.groupby(["node_a", "node_b"]):
        g = g.sort_values("t_sim")
        t = g["t_sim"].to_numpy()
        ax1.plot(t, g["snr_db"].to_numpy(), lw=1.3, label=link_label(a, b))
        ax2.plot(t, g["obstacle_loss_db"].to_numpy(), lw=1.3, label=link_label(a, b))
    ax1.set_ylabel("SNR (dB)")
    ax1.set_title("Layers 2–3 — Per-link SNR over the mission")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", fontsize=8)
    ax2.set_xlabel("Simulation time (s)")
    ax2.set_ylabel("Obstacle loss (dB)")
    ax2.set_title("Obstacle shadowing driving the SNR drops")
    ax2.grid(True, alpha=0.3)
    _save(fig, outdir, "03_snr_over_time.png")


def _save(fig, outdir, name):
    fig.tight_layout()
    path = os.path.join(outdir, name)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="metrics CSV from the NS-3 sim")
    ap.add_argument("--outdir", default="./figs")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    df = pd.read_csv(args.csv)
    print(f"loaded {len(df)} samples, {df['t_sim'].max():.1f} s, "
          f"links={sorted(set(zip(df.node_a, df.node_b)))}")

    plot_pathloss(df, args.outdir)
    plot_fading(df, args.outdir)
    plot_snr_time(df, args.outdir)
    print("done.")


if __name__ == "__main__":
    main()

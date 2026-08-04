#!/usr/bin/env python3
"""
acf_plot.py — Phase A4: autocorrelation function, justifying the
unit-of-analysis decision (runs, not frames, are independent samples).

Frames within a single run are typically correlated with their neighbours —
a slow frame tends to be followed by another slow frame. If that's true here,
treating N frames as N independent samples overstates statistical confidence.
This plot makes that autocorrelation visible and quantifies it, rather than
just asserting it.

Usage:
    python3 acf_plot.py run.csv --metric latency_ms --max-lag 30 --label edge_mission

Optionally trims a warm-up window first (use the cutoff from welch_warmup.py):
    python3 acf_plot.py run.csv --metric latency_ms --max-lag 30 \
        --label edge_mission --skip-first 56
"""
import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_series(path, metric, skip_first=0):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    vals = []
    for r in rows:
        v = r.get(metric, "")
        if v in ("", "-1", "-1.0"):
            continue
        vals.append(float(v))
    return np.array(vals[skip_first:])


def acf(x, max_lag):
    """Sample autocorrelation function, lags 0..max_lag."""
    x = x - np.mean(x)
    n = len(x)
    denom = np.sum(x ** 2)
    result = []
    for lag in range(max_lag + 1):
        if lag == 0:
            result.append(1.0)
        else:
            num = np.sum(x[:-lag] * x[lag:])
            result.append(num / denom if denom > 0 else 0.0)
    return np.array(result)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--metric", default="latency_ms")
    ap.add_argument("--max-lag", type=int, default=30)
    ap.add_argument("--skip-first", type=int, default=0,
                     help="warm-up frames to exclude (from welch_warmup.py)")
    ap.add_argument("--label", default="run")
    ap.add_argument("--results-root", default=os.path.expanduser(
        "~/FYP/multi_uav_simulation/results"))
    args = ap.parse_args()

    series = load_series(args.csv, args.metric, args.skip_first)
    if len(series) < args.max_lag + 5:
        print(f"ERROR: only {len(series)} samples after trimming — need at "
              f"least {args.max_lag + 5} for a lag-{args.max_lag} ACF.",
              file=sys.stderr)
        sys.exit(1)

    values = acf(series, args.max_lag)
    n = len(series)
    # Approximate 95% confidence band for "no autocorrelation" (standard
    # large-sample formula: ±1.96/sqrt(n))
    ci = 1.96 / np.sqrt(n)

    n_significant = int(np.sum(np.abs(values[1:]) > ci))
    print(f"=== ACF: {args.label} ({args.metric}, n={n} samples after trim) ===")
    print(f"95% no-correlation band: ±{ci:.3f}")
    print(f"Lags 1-{args.max_lag} exceeding the band: {n_significant}/{args.max_lag}")
    if n_significant > 0:
        first_within = None
        for lag in range(1, args.max_lag + 1):
            if abs(values[lag]) <= ci:
                first_within = lag
                break
        print(f"First lag within the band: {first_within if first_within else 'none within tested range'}")
        print("\nInterpretation: frames are measurably autocorrelated — treating")
        print("individual frames as independent samples would overstate")
        print("statistical confidence. Runs (not frames) should be the unit")
        print("of analysis for Phase F.")
    else:
        print("\nNo significant autocorrelation detected at any tested lag.")
        print("This would be a genuinely useful finding if it holds — it would")
        print("mean frame-level independence is a defensible assumption here.")
        print("Worth double-checking with a longer run / more lags before")
        print("relying on it, since this differs from what's typically seen")
        print("in this kind of measurement.")

    fig, ax = plt.subplots(figsize=(9, 5))
    lags = np.arange(len(values))
    ax.bar(lags, values, width=0.6, color="tab:blue")
    ax.axhline(ci, color="red", linestyle="--", linewidth=1,
               label=f"95% no-correlation band (±{ci:.3f})")
    ax.axhline(-ci, color="red", linestyle="--", linewidth=1)
    ax.axhline(0, color="black", linewidth=0.8)
    ax.set_xlabel("Lag (frames)")
    ax.set_ylabel("Autocorrelation")
    ax.set_title(f"Autocorrelation function — {args.label} ({args.metric})\n"
                 f"n={n} samples" + (f", first {args.skip_first} warm-up frames excluded" if args.skip_first else ""))
    ax.legend()
    ax.grid(alpha=0.3)

    out_dir = os.path.join(args.results_root, "phase_a_apparatus", "figures")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"acf_{args.label}.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"\nFigure saved: {out_path}")


if __name__ == "__main__":
    main()
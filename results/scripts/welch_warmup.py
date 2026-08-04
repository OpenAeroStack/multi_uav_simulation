#!/usr/bin/env python3
"""
welch_warmup.py — Phase A4: warm-up window determination via Welch's method.

PROPER Welch's method needs multiple INDEPENDENT replications of the SAME
fixed condition, averaged at each time offset — this cancels sampling noise
and reveals the underlying transient, which a single run's moving average
cannot do reliably.

Usage (proper — multiple replications of the identical condition):
    python3 welch_warmup.py rep1.csv rep2.csv rep3.csv \
        --metric latency_ms --window 5 --label edge_hover

Usage (fallback — only one run available):
    python3 welch_warmup.py single_run.csv --metric latency_ms --window 5 \
        --label edge_hover
    (prints an explicit warning that this is a single-run proxy, not true
    Welch's method, and should be replaced with real replications once
    available)

Output: a plot in <results_root>/phase_a_apparatus/figures/, plus a printed
suggested cutoff. The cutoff is a SUGGESTION from a flatness heuristic —
Welch's method is normally applied by visual inspection of the plot, so look
at the figure yourself before locking in the number.
"""
import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_series(path, metric):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    vals = []
    for r in rows:
        v = r.get(metric, "")
        if v in ("", "-1", "-1.0"):
            continue
        vals.append(float(v))
    return np.array(vals)


def moving_average(x, window):
    if window <= 1:
        return x
    kernel = np.ones(window) / window
    return np.convolve(x, kernel, mode="valid")


def suggest_cutoff(smoothed, tolerance_multiplier=2.5, consecutive=10, tail_fraction=0.25):
    """Heuristic: estimate the 'settled' mean and noise level from the last
    tail_fraction of the series, then find the first point after which the
    series stays within tolerance_multiplier * settled_std of that mean for
    `consecutive` points in a row. Threshold is relative to the run's own
    noise level rather than a fixed number, since different metrics/machines
    have very different noise scales. This is a starting suggestion only —
    always confirm by eye against the plot."""
    n = len(smoothed)
    tail_start = int(n * (1 - tail_fraction))
    if tail_start >= n - 1:
        return None
    tail = smoothed[tail_start:]
    settled_mean = np.mean(tail)
    settled_std = np.std(tail)
    tolerance = max(tolerance_multiplier * settled_std, 1e-6)

    if n < consecutive + 1:
        return None
    within = np.abs(smoothed - settled_mean) < tolerance
    for i in range(n - consecutive):
        if np.all(within[i:i + consecutive]):
            return i
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+", help="one or more replication CSVs")
    ap.add_argument("--metric", default="latency_ms")
    ap.add_argument("--window", type=int, default=5, help="moving average window")
    ap.add_argument("--label", default="run", help="label for output filenames")
    ap.add_argument("--results-root", default=os.path.expanduser(
        "~/FYP/multi_uav_simulation/results"))
    args = ap.parse_args()

    series_list = [load_series(p, args.metric) for p in args.csvs]
    series_list = [s for s in series_list if len(s) > 0]
    if not series_list:
        print("ERROR: no valid data found in any input CSV.", file=sys.stderr)
        sys.exit(1)

    n_reps = len(series_list)
    if n_reps == 1:
        print("WARNING: only one replication provided.")
        print("This is a single-run proxy, NOT true Welch's method.")
        print("A single run's moving average can still suggest a warm-up")
        print("window, but it does not cancel sampling noise the way")
        print("averaging across independent replications does. Collect")
        print("2-3 more replications of this exact condition if time allows,")
        print("and rerun this script with all of them together.\n")
        ensemble = series_list[0]
    else:
        min_len = min(len(s) for s in series_list)
        if any(len(s) != min_len for s in series_list):
            print(f"NOTE: replications have different lengths; truncating "
                  f"all to the shortest ({min_len} samples) so they align "
                  f"at the same time offsets.")
        trimmed = np.array([s[:min_len] for s in series_list])
        ensemble = trimmed.mean(axis=0)
        print(f"Welch's method: {n_reps} replications, {min_len} samples each, "
              f"ensemble-averaged at each time offset.")

    smoothed = moving_average(ensemble, args.window)

    cutoff_idx = suggest_cutoff(smoothed)

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(ensemble, alpha=0.3, color="tab:blue",
            label=f"ensemble mean (n={n_reps} replication{'s' if n_reps > 1 else ''})")
    ax.plot(range(args.window - 1, args.window - 1 + len(smoothed)), smoothed,
            color="tab:blue", linewidth=2,
            label=f"moving average (window={args.window})")
    if cutoff_idx is not None:
        real_idx = cutoff_idx + args.window - 1
        ax.axvline(real_idx, color="red", linestyle="--",
                   label=f"suggested cutoff: frame {real_idx}")
        print(f"\nSuggested warm-up cutoff: frame index {real_idx}")
        print("This is a heuristic starting point — confirm visually against")
        print("the saved plot before using it as your final exclusion window.")
    else:
        print("\nNo clear flattening point found automatically.")
        print("Inspect the plot manually to choose a cutoff.")

    ax.set_xlabel("Frame index (time offset within run)")
    ax.set_ylabel(args.metric)
    ax.set_title(f"Welch's method warm-up determination — {args.label}\n"
                 f"({n_reps} replication{'s' if n_reps > 1 else ''})")
    ax.legend()
    ax.grid(alpha=0.3)

    out_dir = os.path.join(args.results_root, "phase_a_apparatus", "figures")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"welch_warmup_{args.label}.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"\nFigure saved: {out_path}")


if __name__ == "__main__":
    main()
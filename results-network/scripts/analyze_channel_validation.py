#!/usr/bin/env python3
"""Analyze focused or legacy ns-3 channel-validation CSV files."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

try:
    from scipy.stats import gamma as scipy_gamma
except ImportError:  # SciPy is optional.
    scipy_gamma = None


RESULTS_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = RESULTS_ROOT / "plots" / "channel-validation"

DEFAULT_TX_DBM = 20.0
DEFAULT_N = 2.0
DEFAULT_D0_M = 1.0
DEFAULT_REF_LOSS_DB = 46.73

ALIASES = {
    "t_sim": "timestamp_s",
    "node_a": "tx_node",
    "node_b": "rx_node",
    "pathloss_rx_dbm": "path_loss_only_rssi_dbm",
    "fading_delta_db": "fading_db",
    "fading_m": "nakagami_m",
    "known": "obstacle_report_received",
    "blocked": "blocked",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate log-distance loss and LoS/NLoS Nakagami fading.")
    parser.add_argument("csv", nargs="+", type=Path,
                        help="one or more channel-validation CSV files")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--start-time", type=float)
    parser.add_argument("--end-time", type=float)
    parser.add_argument("--tx-power-dbm", type=float,
                        help="override CSV/default transmit power")
    parser.add_argument("--path-loss-exponent", type=float,
                        help="override CSV/default configured exponent")
    parser.add_argument("--reference-distance-m", type=float,
                        help="override CSV/default reference distance")
    parser.add_argument("--reference-loss-db", type=float,
                        help="override CSV/default reference loss")
    parser.add_argument("--bin-width", type=float, default=5.0,
                        help="distance-bin width in metres (default: 5)")
    return parser.parse_args()


def inside_results_root(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    try:
        resolved.relative_to(RESULTS_ROOT)
    except ValueError:
        raise SystemExit(
            f"Output directory must be inside {RESULTS_ROOT}, got {resolved}")
    return resolved


def load_one(path: Path, args: argparse.Namespace) -> pd.DataFrame:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise SystemExit(f"CSV not found: {path}")
    frame = pd.read_csv(path).rename(columns=ALIASES)

    required = {
        "timestamp_s", "tx_node", "rx_node", "distance_m",
        "path_loss_only_rssi_dbm", "obstacle_loss_db", "faded_rssi_dbm",
        "fading_db", "snr_db", "nakagami_m", "obstacle_report_received",
    }
    missing = sorted(required - set(frame.columns))
    if missing:
        raise SystemExit(f"{path} is missing columns: {', '.join(missing)}")

    if "after_obstacle_rssi_dbm" not in frame:
        frame["after_obstacle_rssi_dbm"] = (
            frame["path_loss_only_rssi_dbm"] - frame["obstacle_loss_db"])
    if "los_state" not in frame:
        if "blocked" not in frame:
            raise SystemExit(f"{path} has neither los_state nor blocked")
        blocked = pd.to_numeric(frame["blocked"], errors="coerce")
        frame["los_state"] = np.where(blocked == 1, "NLoS", "LoS")

    defaults = {
        "tx_power_dbm": args.tx_power_dbm if args.tx_power_dbm is not None else DEFAULT_TX_DBM,
        "path_loss_exponent": (args.path_loss_exponent
                               if args.path_loss_exponent is not None else DEFAULT_N),
        "reference_distance_m": (args.reference_distance_m
                                 if args.reference_distance_m is not None else DEFAULT_D0_M),
        "reference_loss_db": (args.reference_loss_db
                              if args.reference_loss_db is not None else DEFAULT_REF_LOSS_DB),
    }
    for column, default in defaults.items():
        if column not in frame:
            frame[column] = default
    overrides = {
        "tx_power_dbm": args.tx_power_dbm,
        "path_loss_exponent": args.path_loss_exponent,
        "reference_distance_m": args.reference_distance_m,
        "reference_loss_db": args.reference_loss_db,
    }
    for column, value in overrides.items():
        if value is not None:
            frame[column] = value

    numeric = [
        "timestamp_s", "tx_node", "rx_node", "distance_m", "tx_power_dbm",
        "path_loss_exponent", "reference_distance_m", "reference_loss_db",
        "path_loss_only_rssi_dbm", "obstacle_loss_db",
        "after_obstacle_rssi_dbm", "faded_rssi_dbm", "fading_db", "snr_db",
        "nakagami_m", "obstacle_report_received",
    ]
    if "position_report_received" in frame:
        numeric.append("position_report_received")
    for column in numeric:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")

    before = len(frame)
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(subset=numeric)
    frame = frame[(frame["distance_m"] > 0)
                  & (frame["reference_distance_m"] > 0)
                  & (frame["nakagami_m"] > 0)]
    frame = frame[frame["obstacle_report_received"] == 1]
    if "position_report_received" in frame:
        frame = frame[frame["position_report_received"] == 1]
    if args.start_time is not None:
        frame = frame[frame["timestamp_s"] >= args.start_time]
    if args.end_time is not None:
        frame = frame[frame["timestamp_s"] <= args.end_time]

    frame["los_state"] = frame["los_state"].astype(str).str.strip()
    frame = frame[frame["los_state"].isin(["LoS", "NLoS"])]
    frame["source_file"] = path.name
    removed = before - len(frame)
    print(f"Loaded {len(frame):,} usable rows from {path} ({removed:,} removed)")
    return frame


def gamma_pdf(x: np.ndarray, m: float) -> np.ndarray:
    result = np.zeros_like(x, dtype=float)
    positive = x > 0
    xp = x[positive]
    result[positive] = np.exp(
        m * math.log(m) - math.lgamma(m) + (m - 1.0) * np.log(xp) - m * xp)
    return result


def normalized_power(group: pd.DataFrame) -> np.ndarray:
    # Equivalent to P_faded/P_after_obstacle.  Using fading_db avoids numerical
    # underflow while retaining the exact decomposition logged by ns-3.
    return np.power(10.0, group["fading_db"].to_numpy(dtype=float) / 10.0)


def nakagami_statistics(frame: pd.DataFrame) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for (state, configured_m), group in frame.groupby(["los_state", "nakagami_m"]):
        power = normalized_power(group)
        power = power[np.isfinite(power) & (power > 0)]
        if power.size < 2:
            continue
        mean = float(np.mean(power))
        variance = float(np.var(power, ddof=1))
        moments = mean * mean / variance if variance > 0 else math.nan
        mle_m = math.nan
        mle_scale = math.nan
        if scipy_gamma is not None:
            mle_m, _, mle_scale = scipy_gamma.fit(power, floc=0.0)
        rows.append({
            "state": state,
            "configured_m": float(configured_m),
            "samples": int(power.size),
            "mean_distance_m": float(group["distance_m"].mean()),
            "distance_std_m": float(group["distance_m"].std(ddof=1)),
            "distance_range_m": float(group["distance_m"].max()
                                      - group["distance_m"].min()),
            "mean_obstacle_loss_db": float(group["obstacle_loss_db"].mean()),
            "obstacle_std_db": float(group["obstacle_loss_db"].std(ddof=1)),
            "obstacle_range_db": float(group["obstacle_loss_db"].max()
                                     - group["obstacle_loss_db"].min()),
            "mean_rssi_dbm": float(group["faded_rssi_dbm"].mean()),
            "rssi_std_db": float(group["faded_rssi_dbm"].std(ddof=1)),
            "normalized_power_mean": mean,
            "normalized_power_variance": variance,
            "moments_m": moments,
            "mle_m": float(mle_m),
            "mle_scale": float(mle_scale),
        })
    return rows


def theoretical_rssi(frame: pd.DataFrame) -> np.ndarray:
    return (frame["tx_power_dbm"].to_numpy()
            - frame["reference_loss_db"].to_numpy()
            - 10.0 * frame["path_loss_exponent"].to_numpy()
            * np.log10(frame["distance_m"].to_numpy()
                       / frame["reference_distance_m"].to_numpy()))


def log_distance_statistics(frame: pd.DataFrame) -> dict[str, float]:
    observed = frame["path_loss_only_rssi_dbm"].to_numpy(dtype=float)
    predicted = theoretical_rssi(frame)
    residual = observed - predicted
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    mae = float(np.mean(np.abs(residual)))
    bias = float(np.mean(residual))
    denom = float(np.sum((observed - np.mean(observed)) ** 2))
    r_squared = 1.0 - float(np.sum(residual ** 2)) / denom if denom > 1e-15 else math.nan

    log_distance = np.log10(frame["distance_m"].to_numpy(dtype=float))
    if len(frame) >= 2 and float(np.ptp(log_distance)) > 1e-9:
        slope, intercept = np.polyfit(log_distance, observed, 1)
        estimated_n = float(-slope / 10.0)
    else:
        estimated_n = math.nan
        intercept = math.nan
    return {
        "samples": float(len(frame)),
        "configured_n": float(frame["path_loss_exponent"].median()),
        "estimated_n": estimated_n,
        "regression_intercept": float(intercept),
        "rmse_db": rmse,
        "mae_db": mae,
        "bias_db": bias,
        "r_squared": r_squared,
    }


def plot_nakagami(frame: pd.DataFrame, stats: list[dict[str, object]], output: Path) -> None:
    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    colours = {"LoS": "tab:blue", "NLoS": "tab:orange"}
    for row in stats:
        state = str(row["state"])
        m = float(row["configured_m"])
        group = frame[(frame["los_state"] == state)
                      & np.isclose(frame["nakagami_m"], m)]
        raw = normalized_power(group)
        values = raw / np.mean(raw)
        upper = max(4.0, float(np.quantile(values, 0.995)) * 1.1)
        x = np.linspace(0.001, upper, 600)
        colour = colours.get(state)
        ax.hist(values, bins=60, range=(0, upper), density=True, alpha=0.28,
                color=colour, label=f"{state} samples (n={len(values)})")
        ax.plot(x, gamma_pdf(x, m), color=colour, lw=2,
                label=f"{state} theory m={m:g}")
        mle = float(row["mle_m"])
        if math.isfinite(mle):
            ax.plot(x, gamma_pdf(x, mle), color=colour, lw=1.4, ls=":",
                    label=f"{state} fitted m={mle:.2f}")
    ax.set_xlabel("Normalized received power P / E[P]")
    ax.set_ylabel("Probability density")
    ax.set_title("Nakagami normalized-power distribution")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output / "nakagami_power_distribution.png", dpi=160)
    plt.close(fig)


def plot_rssi(frame: pd.DataFrame, output: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for source, group in frame.groupby("source_file"):
        group = group.sort_values("timestamp_s")
        suffix = f" [{source}]" if frame["source_file"].nunique() > 1 else ""
        time = group["timestamp_s"].to_numpy(dtype=float)
        ax.plot(time, group["faded_rssi_dbm"].to_numpy(dtype=float), lw=0.8,
                alpha=0.7, label=f"Faded RSSI{suffix}")
        ax.plot(time, group["after_obstacle_rssi_dbm"].to_numpy(dtype=float), lw=1.6,
                label=f"After-obstacle baseline{suffix}")
        ax.plot(time, group["path_loss_only_rssi_dbm"].to_numpy(dtype=float), lw=1.2,
                ls="--", label=f"Path-loss-only baseline{suffix}")
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("Received power (dBm)")
    ax.set_title("RSSI and deterministic baselines")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output / "rssi_timeseries.png", dpi=160)
    plt.close(fig)


def plot_log_distance(frame: pd.DataFrame, output: Path, bin_width: float) -> None:
    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    distance = frame["distance_m"].to_numpy(dtype=float)
    measured = frame["path_loss_only_rssi_dbm"].to_numpy(dtype=float)
    ax.scatter(distance, measured, s=9, alpha=0.22,
               label="Path-loss-only samples")

    dline = np.linspace(max(float(np.min(distance)), 0.01),
                        float(np.max(distance)), 500)
    tx = float(frame["tx_power_dbm"].median())
    n = float(frame["path_loss_exponent"].median())
    d0 = float(frame["reference_distance_m"].median())
    ref = float(frame["reference_loss_db"].median())
    curve = tx - ref - 10.0 * n * np.log10(dline / d0)
    ax.plot(dline, curve, "k--", lw=2, label=f"Theory, n={n:g}")

    if bin_width > 0 and float(np.ptp(distance)) >= bin_width:
        bins = np.floor(distance / bin_width) * bin_width
        binned = pd.DataFrame({"bin": bins, "distance": distance,
                               "rssi": measured}).groupby("bin").mean()
        ax.plot(binned["distance"].to_numpy(dtype=float),
                binned["rssi"].to_numpy(dtype=float), "o-", color="tab:red",
                lw=1.5, label=f"{bin_width:g} m binned mean")
    ax.set_xlabel("Distance (m)")
    ax.set_ylabel("Path-loss-only RSSI (dBm)")
    ax.set_title("Log-distance model validation")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "log_distance_validation.png", dpi=160)
    plt.close(fig)


def plot_los_vs_nlos(frame: pd.DataFrame, output: Path) -> bool:
    states = set(frame["los_state"])
    if not {"LoS", "NLoS"}.issubset(states):
        return False
    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    for state, colour in [("LoS", "tab:blue"), ("NLoS", "tab:orange")]:
        group = frame[frame["los_state"] == state]
        values = normalized_power(group)
        values = values / np.mean(values)
        m = float(group["nakagami_m"].median())
        ax.hist(values, bins=60, range=(0, 5), density=True, alpha=0.35,
                color=colour, label=f"{state} samples, configured m={m:g}")
    ax.set_xlabel("Normalized received power P / E[P]")
    ax.set_ylabel("Probability density")
    ax.set_title("LoS versus NLoS fading")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "los_vs_nlos_fading.png", dpi=160)
    plt.close(fig)
    return True


def fmt(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}" if math.isfinite(value) else "unavailable"


def build_summary(frame: pd.DataFrame, nakagami: list[dict[str, object]],
                  log_distance: dict[str, float]) -> str:
    lines = ["## CHANNEL VALIDATION SUMMARY", ""]
    source_types = (sorted(frame["sample_source"].dropna().unique())
                    if "sample_source" in frame else ["legacy periodic model evaluation"])
    lines.append(f"Sample source:             {', '.join(map(str, source_types))}")
    for row in nakagami:
        lines += [
            "",
            f"State:                     {row['state']}",
            f"Samples:                   {row['samples']}",
            f"Mean distance:             {fmt(float(row['mean_distance_m']), 3)} m",
            f"Distance std:              {fmt(float(row['distance_std_m']), 3)} m",
            f"Configured Nakagami m:     {fmt(float(row['configured_m']), 2)}",
            f"Estimated m (moments):     {fmt(float(row['moments_m']), 3)}",
            f"Estimated m (MLE):         {fmt(float(row['mle_m']), 3)}",
            f"Normalized power mean:     {fmt(float(row['normalized_power_mean']), 4)}",
            f"Normalized power variance: {fmt(float(row['normalized_power_variance']), 4)}",
            f"Mean faded RSSI:           {fmt(float(row['mean_rssi_dbm']), 3)} dBm",
            f"RSSI standard deviation:   {fmt(float(row['rssi_std_db']), 3)} dB",
        ]
        distance_std = float(row["distance_std_m"])
        mean_distance = float(row["mean_distance_m"])
        if distance_std > max(0.5, 0.01 * mean_distance):
            lines.append(
                f"WARNING: distance varied over a {fmt(float(row['distance_range_m']), 2)} m "
                "range; Nakagami validation is strongest during a stationary hover.")
        if (float(row["obstacle_std_db"]) > 1.0
                or float(row["obstacle_range_db"]) > 3.0):
            lines.append(
                f"WARNING: obstacle attenuation varied over a "
                f"{fmt(float(row['obstacle_range_db']), 2)} dB range.")

    lines += [
        "",
        "## LOG-DISTANCE VALIDATION",
        "",
        f"Samples:                   {int(log_distance['samples'])}",
        f"Configured n:              {fmt(log_distance['configured_n'], 2)}",
        f"Estimated n:               {fmt(log_distance['estimated_n'], 3)}",
        f"RMSE:                      {fmt(log_distance['rmse_db'], 6)} dB",
        f"MAE:                       {fmt(log_distance['mae_db'], 6)} dB",
        f"Bias:                      {fmt(log_distance['bias_db'], 6)} dB",
        f"R²:                        {fmt(log_distance['r_squared'], 6)}",
    ]
    if not math.isfinite(log_distance["estimated_n"]):
        lines.append("NOTE: estimating n requires samples at more than one distance.")
    if scipy_gamma is None:
        lines.append("NOTE: SciPy is unavailable; MLE estimates were skipped.")
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    output = inside_results_root(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    frames = [load_one(path, args) for path in args.csv]
    frame = pd.concat(frames, ignore_index=True)
    if frame.empty:
        raise SystemExit("No usable samples remain after cleaning and time filtering.")

    nakagami = nakagami_statistics(frame)
    if not nakagami:
        raise SystemExit("Not enough samples to estimate a Nakagami distribution.")
    log_distance = log_distance_statistics(frame)
    summary = build_summary(frame, nakagami, log_distance)
    print("\n" + summary)
    (output / "summary.txt").write_text(summary, encoding="utf-8")

    pd.DataFrame(nakagami).to_csv(output / "nakagami_statistics.csv", index=False)
    pd.DataFrame([log_distance]).to_csv(
        output / "log_distance_statistics.csv", index=False)
    plot_nakagami(frame, nakagami, output)
    plot_rssi(frame, output)
    plot_log_distance(frame, output, args.bin_width)
    comparison = plot_los_vs_nlos(frame, output)

    print(f"Outputs written to {output}")
    print("Plots: nakagami_power_distribution.png, rssi_timeseries.png, "
          "log_distance_validation.png"
          + (", los_vs_nlos_fading.png" if comparison else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())

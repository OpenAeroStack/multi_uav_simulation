#!/usr/bin/env python3
"""Compare real and simulated horizontal trajectories in one ENU frame."""

import argparse
import math
import numpy as np
import matplotlib.pyplot as plt

from common import (add_common_args, aligned_gps, common_grid, coordinate_origin,
                    gps_to_enu, interpolate, load_or_create_alignment, metric_rows,
                    path_distance, prepare_output, save_plot, validate_inputs,
                    write_csv, write_json)


def cumulative(time, east, north, end):
    mask = np.isfinite(time) & np.isfinite(east) & np.isfinite(north) & (time >= 0) & (time <= end)
    t, x, y = time[mask], east[mask], north[mask]
    return t, np.r_[0.0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]


def run(real_dir, sim_dir, output):
    alignment = load_or_create_alignment(real_dir, sim_dir, output)
    origin = coordinate_origin(sim_dir)
    write_json(output / "trajectory" / "coordinate_origin.json", origin)
    real, sim = aligned_gps(real_dir, "real", alignment), aligned_gps(sim_dir, "sim", alignment)
    re, rn, _ = gps_to_enu(real["lat"], real["lon"], np.zeros_like(real["lat"]), origin)
    se, sn, _ = gps_to_enu(sim["lat"], sim["lon"], np.zeros_like(sim["lat"]), origin)
    end = alignment["comparison_end_s"]
    grid = common_grid(((real["time"], re), (sim["time"], se)), end)
    rei, rni = interpolate(real["time"], re, grid), interpolate(real["time"], rn, grid)
    sei, sni = interpolate(sim["time"], se, grid), interpolate(sim["time"], sn, grid)
    valid = np.isfinite(rei + rni + sei + sni)
    error = np.hypot(rei[valid] - sei[valid], rni[valid] - sni[valid])
    real_distance = path_distance(re, rn, real["time"], end)
    sim_distance = path_distance(se, sn, sim["time"], end)
    values = {
        "real_total_distance_m": real_distance,
        "sim_total_distance_m": sim_distance,
        "distance_difference_m": sim_distance - real_distance,
        "mean_horizontal_error_m": float(np.mean(error)),
        "median_horizontal_error_m": float(np.median(error)),
        "rmse_horizontal_error_m": float(np.sqrt(np.mean(error ** 2))),
        "p95_horizontal_error_m": float(np.percentile(error, 95)),
        "max_horizontal_error_m": float(np.max(error)),
        "comparison_duration_s": end,
        "interpolation_step_s": float(np.median(np.diff(grid))) if len(grid) > 1 else math.nan,
        "comparison_samples": int(len(error)),
    }
    write_csv(output / "trajectory" / "trajectory_metrics.csv", ["metric", "value", "unit"],
              metric_rows(values, {k: ("m" if k.endswith("_m") else "s" if k.endswith("_s") else "") for k in values}))

    mask_r = (real["time"] >= 0) & (real["time"] <= end)
    mask_s = (sim["time"] >= 0) & (sim["time"] <= end)
    plt.figure(figsize=(7.2, 6.2))
    plt.plot(re[mask_r], rn[mask_r], label="Real flight", linewidth=1.7)
    plt.plot(se[mask_s], sn[mask_s], label="Simulation", linewidth=1.7)
    plt.scatter([re[mask_r][0], se[mask_s][0]], [rn[mask_r][0], sn[mask_s][0]], marker="o", s=28, label="Aligned takeoff samples")
    plt.xlabel("East (m)"); plt.ylabel("North (m)")
    plt.title("Real vs Simulated Horizontal Trajectory")
    plt.axis("equal"); plt.grid(True, alpha=.3); plt.legend()
    save_plot(output / "trajectory" / "real_vs_sim_xy.png")

    plt.figure(figsize=(8, 4.5))
    plt.plot(grid[valid], error, color="tab:red", label="Horizontal separation")
    plt.xlabel("Mission time after takeoff (s)"); plt.ylabel("Horizontal error (m)")
    plt.title("Horizontal Position Error vs Mission Time")
    plt.grid(True, alpha=.3); plt.legend()
    save_plot(output / "trajectory" / "horizontal_error_vs_time.png")

    rt, rd = cumulative(real["time"], re, rn, end); st, sd = cumulative(sim["time"], se, sn, end)
    plt.figure(figsize=(8, 4.5))
    plt.plot(rt, rd, label="Real flight"); plt.plot(st, sd, label="Simulation")
    plt.xlabel("Mission time after takeoff (s)"); plt.ylabel("Cumulative horizontal distance (m)")
    plt.title("Cumulative Distance Along Recorded Paths")
    plt.grid(True, alpha=.3); plt.legend()
    save_plot(output / "trajectory" / "cumulative_distance.png")


def main():
    parser = argparse.ArgumentParser(); add_common_args(parser); args = parser.parse_args()
    real, sim = validate_inputs(args.real_dir, args.sim_dir); output = prepare_output(args.output_dir)
    run(real, sim, output); print(output / "trajectory")


if __name__ == "__main__": main()

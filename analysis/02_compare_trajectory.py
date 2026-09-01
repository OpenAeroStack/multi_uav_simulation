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


def point_to_polyline_distances(points, polyline):
    """Return exact shortest distances/projections to consecutive segments."""
    if len(polyline) == 0:
        raise ValueError("Cannot measure distance to an empty trajectory")
    if len(polyline) == 1:
        projections = np.repeat(polyline, len(points), axis=0)
        return np.linalg.norm(points - projections, axis=1), projections, np.zeros(len(points), dtype=int)

    starts = polyline[:-1]
    vectors = polyline[1:] - starts
    lengths_squared = np.sum(vectors * vectors, axis=1)
    distances = np.empty(len(points), dtype=float)
    projections = np.empty_like(points, dtype=float)
    segment_indices = np.empty(len(points), dtype=int)
    for index, point in enumerate(points):
        offsets = point - starts
        fractions = np.zeros(len(starts), dtype=float)
        nonzero = lengths_squared > 0
        fractions[nonzero] = (
            np.sum(offsets[nonzero] * vectors[nonzero], axis=1)
            / lengths_squared[nonzero])
        fractions = np.clip(fractions, 0.0, 1.0)
        candidates = starts + fractions[:, None] * vectors
        squared = np.sum((candidates - point) ** 2, axis=1)
        nearest = int(np.argmin(squared))
        distances[index] = math.sqrt(float(squared[nearest]))
        projections[index] = candidates[nearest]
        segment_indices[index] = nearest
    return distances, projections, segment_indices


def path_progress(points):
    if not len(points):
        return np.asarray([])
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]


def directed_metrics(prefix, distances):
    return {
        f"{prefix}_mean_path_error_m": float(np.mean(distances)),
        f"{prefix}_median_path_error_m": float(np.median(distances)),
        f"{prefix}_rmse_path_error_m": float(np.sqrt(np.mean(distances ** 2))),
        f"{prefix}_p95_path_error_m": float(np.percentile(distances, 95)),
        f"{prefix}_max_path_error_m": float(np.max(distances)),
    }


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
    real_points = np.column_stack((re[mask_r], rn[mask_r]))
    sim_points = np.column_stack((se[mask_s], sn[mask_s]))
    sim_to_real, sim_projections, sim_segments = point_to_polyline_distances(
        sim_points, real_points)
    real_to_sim, real_projections, real_segments = point_to_polyline_distances(
        real_points, sim_points)
    geometric = {}
    geometric.update(directed_metrics("sim_to_real", sim_to_real))
    geometric.update(directed_metrics("real_to_sim", real_to_sim))
    geometric.update({
        "symmetric_mean_path_error_m": float(
            (np.mean(sim_to_real) + np.mean(real_to_sim)) / 2.0),
        "symmetric_rmse_path_error_m": float(np.sqrt(
            (np.mean(sim_to_real ** 2) + np.mean(real_to_sim ** 2)) / 2.0)),
        "symmetric_max_path_deviation_m": float(
            max(np.max(sim_to_real), np.max(real_to_sim))),
        "real_path_point_count": int(len(real_points)),
        "sim_path_point_count": int(len(sim_points)),
    })
    write_csv(
        output / "trajectory" / "geometric_path_metrics.csv",
        ["metric", "value", "unit"],
        metric_rows(geometric, {
            key: ("count" if key.endswith("_count") else "m")
            for key in geometric
        }))
    write_csv(
        output / "trajectory" / "sim_to_real_path_error.csv",
        ["point_index", "east_m", "north_m", "nearest_real_segment_index",
         "nearest_east_m", "nearest_north_m", "sim_to_real_path_error_m"],
        ({
            "point_index": index,
            "east_m": point[0], "north_m": point[1],
            "nearest_real_segment_index": sim_segments[index],
            "nearest_east_m": sim_projections[index, 0],
            "nearest_north_m": sim_projections[index, 1],
            "sim_to_real_path_error_m": sim_to_real[index],
        } for index, point in enumerate(sim_points)))
    write_csv(
        output / "trajectory" / "real_to_sim_path_error.csv",
        ["point_index", "east_m", "north_m", "nearest_sim_segment_index",
         "nearest_east_m", "nearest_north_m", "real_to_sim_path_error_m"],
        ({
            "point_index": index,
            "east_m": point[0], "north_m": point[1],
            "nearest_sim_segment_index": real_segments[index],
            "nearest_east_m": real_projections[index, 0],
            "nearest_north_m": real_projections[index, 1],
            "real_to_sim_path_error_m": real_to_sim[index],
        } for index, point in enumerate(real_points)))

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

    figure, axes = plt.subplots(1, 2, figsize=(12, 5.2), sharex=True, sharey=True)
    color_limit = max(float(np.percentile(sim_to_real, 95)),
                      float(np.percentile(real_to_sim, 95)), 1e-9)
    panels = (
        (axes[0], sim_points, real_points, sim_to_real,
         "Simulation Points to Real Polyline", "Simulation points"),
        (axes[1], real_points, sim_points, real_to_sim,
         "Real Points to Simulation Polyline", "Real points"),
    )
    for axis, source, reference, errors, title, label in panels:
        axis.plot(reference[:, 0], reference[:, 1], color="0.55",
                  linewidth=1.6, label="Reference polyline")
        scatter = axis.scatter(
            source[:, 0], source[:, 1], c=errors, cmap="viridis",
            vmin=0, vmax=color_limit, s=18, label=label)
        axis.set_title(title)
        axis.set_xlabel("East (m)")
        axis.grid(True, alpha=.3)
        axis.set_aspect("equal", adjustable="box")
        axis.legend()
    axes[0].set_ylabel("North (m)")
    figure.colorbar(scatter, ax=axes, label="Shortest point-to-polyline distance (m)")
    figure.suptitle("Geometry-Only Bidirectional Path Comparison")
    figure.subplots_adjust(left=.07, right=.93, bottom=.12, top=.86, wspace=.18)
    plt.savefig(output / "trajectory" / "geometric_path_comparison.png",
                dpi=300, bbox_inches="tight")
    plt.close(figure)

    plt.figure(figsize=(8, 4.5))
    plt.plot(path_progress(sim_points), sim_to_real,
             label="Simulation points → real polyline")
    plt.plot(path_progress(real_points), real_to_sim,
             label="Real points → simulation polyline")
    plt.xlabel("Cumulative distance along source trajectory (m)")
    plt.ylabel("Shortest point-to-polyline distance (m)")
    plt.title("Geometry-Only Path Error Along Each Trajectory")
    plt.grid(True, alpha=.3); plt.legend()
    save_plot(output / "trajectory" / "path_error_along_trajectory.png")


def main():
    parser = argparse.ArgumentParser(); add_common_args(parser); args = parser.parse_args()
    real, sim = validate_inputs(args.real_dir, args.sim_dir); output = prepare_output(args.output_dir)
    run(real, sim, output); print(output / "trajectory")


if __name__ == "__main__": main()

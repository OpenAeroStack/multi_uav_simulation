#!/usr/bin/env python3
"""Compare real and simulated horizontal trajectories in one ENU frame."""

import argparse
import math
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from common import (add_common_args, aligned_gps, common_grid, coordinate_origin,
                    gps_to_enu, interpolate, load_or_create_alignment, metric_rows,
                    path_distance, prepare_output, save_plot, validate_inputs,
                    write_csv, write_json)


MISSION_WAYPOINT_FILE = (
    Path(__file__).resolve().parent
    / "data"
    / "real_2026-08-31_18-03-14.waypoints"
)


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


def load_commanded_waypoints(path):
    """Read valid, unique horizontal positions from a QGC WPL 110 mission."""
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines or lines[0].strip() != "QGC WPL 110":
        raise ValueError(f"Unsupported or missing QGC WPL 110 header: {path}")

    waypoints = []
    for line in lines[1:]:
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 12:
            raise ValueError(f"Expected 12 QGC waypoint fields in: {line}")
        mission_index = int(fields[0])
        latitude = float(fields[8])
        longitude = float(fields[9])
        altitude = float(fields[10])

        # TAKEOFF has no valid horizontal coordinate in this mission.
        if latitude == 0.0 and longitude == 0.0:
            continue
        # WP6 and LAND share one horizontal coordinate; retain it only once.
        if (waypoints
                and math.isclose(latitude, waypoints[-1]["latitude"], abs_tol=1e-10)
                and math.isclose(longitude, waypoints[-1]["longitude"], abs_tol=1e-10)):
            continue
        waypoints.append({
            "mission_index": mission_index,
            "label": "Start" if mission_index == 0 else f"WP{mission_index}",
            "latitude": latitude,
            "longitude": longitude,
            "altitude_m": altitude,
        })

    if len(waypoints) < 2:
        raise ValueError(f"Commanded mission needs at least two unique GPS positions: {path}")
    waypoints[-1]["label"] = f"Final Target (WP{waypoints[-1]['mission_index']}/Land)"
    return waypoints


def distance_summary(distances):
    return {
        "mean_m": float(np.mean(distances)),
        "median_m": float(np.median(distances)),
        "rmse_m": float(np.sqrt(np.mean(distances ** 2))),
        "p95_m": float(np.percentile(distances, 95)),
        "max_m": float(np.max(distances)),
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

    commanded_waypoints = load_commanded_waypoints(MISSION_WAYPOINT_FILE)
    waypoint_lat = np.asarray([row["latitude"] for row in commanded_waypoints])
    waypoint_lon = np.asarray([row["longitude"] for row in commanded_waypoints])
    waypoint_east, waypoint_north, _ = gps_to_enu(
        waypoint_lat, waypoint_lon, np.zeros_like(waypoint_lat), origin)
    commanded_points = np.column_stack((waypoint_east, waypoint_north))
    for index, waypoint in enumerate(commanded_waypoints):
        waypoint["east_m"] = waypoint_east[index]
        waypoint["north_m"] = waypoint_north[index]
    write_csv(
        output / "trajectory" / "commanded_waypoints_enu.csv",
        ["mission_index", "label", "latitude", "longitude", "east_m",
         "north_m", "altitude_m"],
        commanded_waypoints)

    real_to_commanded, _, _ = point_to_polyline_distances(
        real_points, commanded_points)
    sim_to_commanded, _, _ = point_to_polyline_distances(
        sim_points, commanded_points)
    commanded_metrics = [
        {"trajectory": "real", **distance_summary(real_to_commanded)},
        {"trajectory": "simulation", **distance_summary(sim_to_commanded)},
    ]
    write_csv(
        output / "trajectory" / "commanded_path_error_metrics.csv",
        ["trajectory", "mean_m", "median_m", "rmse_m", "p95_m", "max_m"],
        commanded_metrics)

    figure, axis = plt.subplots(figsize=(8.5, 7.0))
    axis.plot(real_points[:, 0], real_points[:, 1], color="tab:blue",
              linewidth=2.0, label="Real UAV trajectory", zorder=2)
    axis.plot(sim_points[:, 0], sim_points[:, 1], color="tab:orange",
              linewidth=2.0, label="Simulated UAV trajectory", zorder=2)
    axis.plot(commanded_points[:, 0], commanded_points[:, 1], color="black",
              linestyle="--", linewidth=1.8, marker="D", markersize=5.5,
              label="Commanded mission path", zorder=3)
    for index, waypoint in enumerate(commanded_waypoints):
        if index in (0, len(commanded_waypoints) - 1):
            dx = 7
        else:
            dx = 7 if index % 2 == 0 else -7
        horizontal_alignment = "left" if dx > 0 else "right"
        axis.annotate(
            waypoint["label"],
            (waypoint["east_m"], waypoint["north_m"]),
            xytext=(dx, 7), textcoords="offset points",
            ha=horizontal_alignment, va="bottom", fontsize=10,
            fontweight="bold" if index in (0, len(commanded_waypoints) - 1) else "normal")
    axis.set_title("Real vs Simulated Flight Path", fontsize=16, pad=12)
    axis.set_xlabel("East–West displacement (m)", fontsize=13, labelpad=30)
    axis.set_ylabel("North–South displacement (m)", fontsize=13)
    axis.tick_params(labelsize=11)
    axis.grid(True, alpha=.25)
    axis.legend(fontsize=10, loc="center left", bbox_to_anchor=(1.03, .5))
    axis.set_aspect("equal", adjustable="box")
    all_points = np.vstack((real_points, sim_points, commanded_points))
    x_min, y_min = np.min(all_points, axis=0)
    x_max, y_max = np.max(all_points, axis=0)
    span = max(x_max - x_min, y_max - y_min)
    margin = max(0.75, span * 0.06)
    axis.set_xlim(x_min - margin, x_max + margin)
    axis.set_ylim(y_min - margin, y_max + margin)
    direction_style = {
        "transform": axis.transAxes,
        "fontsize": 9.5,
        "color": "0.38",
        "clip_on": False,
    }
    axis.text(.97, .985, "↑ North", ha="right", va="top",
              **direction_style)
    axis.text(.03, .015, "South ↓", ha="left", va="bottom",
              **direction_style)
    axis.text(0, -.035, "← West", ha="left", va="top",
              **direction_style)
    axis.text(1, -.035, "East →", ha="right", va="top",
              **direction_style)
    save_plot(
        output / "trajectory"
        / "real_vs_sim_vs_commanded_path_directional_axes.png")

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

    print("Commanded-path comparison")
    for row, label in zip(commanded_metrics, ("Real", "Simulation")):
        print(f"{label}:")
        print(f"  mean path error = {row['mean_m']:.3f} m")
        print(f"  RMSE = {row['rmse_m']:.3f} m")
        print(f"  P95 = {row['p95_m']:.3f} m")


def main():
    parser = argparse.ArgumentParser(); add_common_args(parser); args = parser.parse_args()
    real, sim = validate_inputs(args.real_dir, args.sim_dir); output = prepare_output(args.output_dir)
    run(real, sim, output); print(output / "trajectory")


if __name__ == "__main__": main()

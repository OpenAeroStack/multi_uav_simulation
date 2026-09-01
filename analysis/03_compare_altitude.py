#!/usr/bin/env python3
"""Compare relative altitude over the aligned mission interval."""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from common import (add_common_args, aligned_gps, common_grid, interpolate,
                    load_or_create_alignment, metric_rows, prepare_output,
                    save_plot, validate_inputs, write_csv)


def run(real_dir, sim_dir, output):
    a = load_or_create_alignment(real_dir, sim_dir, output); end = a["comparison_end_s"]
    r, s = aligned_gps(real_dir, "real", a), aligned_gps(sim_dir, "sim", a)
    grid = common_grid(((r["time"], r["alt"]), (s["time"], s["alt"])), end)
    ri, si = interpolate(r["time"], r["alt"], grid), interpolate(s["time"], s["alt"], grid)
    valid = np.isfinite(ri + si); error = si[valid] - ri[valid]; absolute = np.abs(error)
    rm = (r["time"] >= 0) & (r["time"] <= end) & np.isfinite(r["alt"])
    sm = (s["time"] >= 0) & (s["time"] <= end) & np.isfinite(s["alt"])
    values = {
        "real_mean_altitude_m": float(np.mean(r["alt"][rm])), "sim_mean_altitude_m": float(np.mean(s["alt"][sm])),
        "mean_altitude_error_m": float(np.mean(error)), "mae_altitude_m": float(np.mean(absolute)),
        "rmse_altitude_m": float(np.sqrt(np.mean(error ** 2))), "p95_abs_altitude_error_m": float(np.percentile(absolute, 95)),
        "max_abs_altitude_error_m": float(np.max(absolute)), "real_max_altitude_m": float(np.max(r["alt"][rm])),
        "sim_max_altitude_m": float(np.max(s["alt"][sm])), "comparison_duration_s": end,
    }
    units={k:("s" if k.endswith("_s") else "m") for k in values}
    write_csv(output/"altitude"/"altitude_metrics.csv", ["metric","value","unit"], metric_rows(values, units))
    plt.figure(figsize=(8,4.5)); plt.plot(r["time"][rm],r["alt"][rm],label="Real flight"); plt.plot(s["time"][sm],s["alt"][sm],label="Simulation")
    plt.xlabel("Mission time after takeoff (s)"); plt.ylabel("Relative altitude (m)"); plt.title("Real vs Simulated Relative Altitude"); plt.grid(True,alpha=.3); plt.legend(); save_plot(output/"altitude"/"real_vs_sim_altitude.png")
    plt.figure(figsize=(8,4.5)); plt.plot(grid[valid],error,label="Simulation − real",color="tab:red"); plt.axhline(0,color="black",lw=.8)
    plt.xlabel("Mission time after takeoff (s)"); plt.ylabel("Altitude error (m)"); plt.title("Relative Altitude Error vs Mission Time"); plt.grid(True,alpha=.3); plt.legend(); save_plot(output/"altitude"/"altitude_error_vs_time.png")


def main():
    p=argparse.ArgumentParser(); add_common_args(p); x=p.parse_args(); r,s=validate_inputs(x.real_dir,x.sim_dir); o=prepare_output(x.output_dir); run(r,s,o); print(o/"altitude")
if __name__=="__main__": main()

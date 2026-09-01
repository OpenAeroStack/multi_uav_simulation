#!/usr/bin/env python3
"""Compare measured GPS ground speed over the aligned mission interval."""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from common import (add_common_args, aligned_gps, common_grid, interpolate,
                    load_or_create_alignment, metric_rows, prepare_output,
                    save_plot, validate_inputs, write_csv)


def run(real_dir, sim_dir, output):
    a=load_or_create_alignment(real_dir,sim_dir,output); end=a["comparison_end_s"]
    r,s=aligned_gps(real_dir,"real",a),aligned_gps(sim_dir,"sim",a)
    grid=common_grid(((r["time"],r["speed"]),(s["time"],s["speed"])),end)
    ri,si=interpolate(r["time"],r["speed"],grid),interpolate(s["time"],s["speed"],grid)
    valid=np.isfinite(ri+si); error=si[valid]-ri[valid]; absolute=np.abs(error)
    rm=(r["time"]>=0)&(r["time"]<=end)&np.isfinite(r["speed"]); sm=(s["time"]>=0)&(s["time"]<=end)&np.isfinite(s["speed"])
    values={"real_mean_speed_mps":float(np.mean(r["speed"][rm])),"sim_mean_speed_mps":float(np.mean(s["speed"][sm])),
      "real_max_speed_mps":float(np.max(r["speed"][rm])),"sim_max_speed_mps":float(np.max(s["speed"][sm])),
      "mean_speed_error_mps":float(np.mean(error)),"mae_speed_mps":float(np.mean(absolute)),"rmse_speed_mps":float(np.sqrt(np.mean(error**2))),
      "p95_abs_speed_error_mps":float(np.percentile(absolute,95)),"max_abs_speed_error_mps":float(np.max(absolute)),"comparison_duration_s":end}
    units={k:("s" if k.endswith("_s") else "m/s") for k in values}
    write_csv(output/"speed"/"speed_metrics.csv",["metric","value","unit"],metric_rows(values,units))
    plt.figure(figsize=(8,4.5)); plt.plot(r["time"][rm],r["speed"][rm],label="Real flight"); plt.plot(s["time"][sm],s["speed"][sm],label="Simulation")
    plt.xlabel("Mission time after takeoff (s)"); plt.ylabel("Measured ground speed (m/s)"); plt.title("Real vs Simulated Ground Speed"); plt.grid(True,alpha=.3); plt.legend(); save_plot(output/"speed"/"real_vs_sim_groundspeed.png")
    plt.figure(figsize=(8,4.5)); plt.plot(grid[valid],error,label="Simulation − real",color="tab:red"); plt.axhline(0,color="black",lw=.8)
    plt.xlabel("Mission time after takeoff (s)"); plt.ylabel("Ground-speed error (m/s)"); plt.title("Ground-Speed Error vs Mission Time"); plt.grid(True,alpha=.3); plt.legend(); save_plot(output/"speed"/"speed_error_vs_time.png")


def main():
    p=argparse.ArgumentParser(); add_common_args(p); x=p.parse_args(); r,s=validate_inputs(x.real_dir,x.sim_dir); o=prepare_output(x.output_dir); run(r,s,o); print(o/"speed")
if __name__=="__main__": main()

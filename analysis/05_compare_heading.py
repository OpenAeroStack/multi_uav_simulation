#!/usr/bin/env python3
"""Compare GPS heading using circular-angle error."""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from common import (add_common_args, aligned_gps, circular_difference_deg,
                    common_grid, interpolate, load_or_create_alignment,
                    metric_rows, prepare_output, save_plot, validate_inputs,
                    write_csv)


def circular_interp(t, degrees, grid):
    valid=np.isfinite(t)&np.isfinite(degrees); unwrapped=np.unwrap(np.deg2rad(degrees[valid]))
    return np.rad2deg(interpolate(t[valid],unwrapped,grid))%360.0


def run(real_dir,sim_dir,output):
    a=load_or_create_alignment(real_dir,sim_dir,output); end=a["comparison_end_s"]
    r,s=aligned_gps(real_dir,"real",a),aligned_gps(sim_dir,"sim",a)
    grid=common_grid(((r["time"],r["heading"]),(s["time"],s["heading"])),end)
    ri,si=circular_interp(r["time"],r["heading"],grid),circular_interp(s["time"],s["heading"],grid)
    valid=np.isfinite(ri+si); error=circular_difference_deg(si[valid],ri[valid]); absolute=np.abs(error)
    values={"mean_abs_heading_error_deg":float(np.mean(absolute)),"median_abs_heading_error_deg":float(np.median(absolute)),
      "p95_abs_heading_error_deg":float(np.percentile(absolute,95)),"max_abs_heading_error_deg":float(np.max(absolute)),
      "signed_mean_circular_heading_error_deg":float(np.mean(error)),"comparison_duration_s":end}
    units={k:("s" if k.endswith("_s") else "deg") for k in values}
    write_csv(output/"heading"/"heading_metrics.csv",["metric","value","unit"],metric_rows(values,units))
    rm=(r["time"]>=0)&(r["time"]<=end)&np.isfinite(r["heading"]); sm=(s["time"]>=0)&(s["time"]<=end)&np.isfinite(s["heading"])
    plt.figure(figsize=(8,4.5)); plt.plot(r["time"][rm],r["heading"][rm],label="Real GPS heading"); plt.plot(s["time"][sm],s["heading"][sm],label="Simulation GPS heading")
    plt.xlabel("Mission time after takeoff (s)"); plt.ylabel("Heading clockwise from North (deg)"); plt.title("Real vs Simulated GPS Heading"); plt.ylim(0,360); plt.grid(True,alpha=.3); plt.legend(); save_plot(output/"heading"/"real_vs_sim_heading.png")
    plt.figure(figsize=(8,4.5)); plt.plot(grid[valid],error,label="Circular error: simulation − real",color="tab:red"); plt.axhline(0,color="black",lw=.8)
    plt.xlabel("Mission time after takeoff (s)"); plt.ylabel("Circular heading error (deg)"); plt.title("Circular Heading Error vs Mission Time"); plt.ylim(-180,180); plt.grid(True,alpha=.3); plt.legend(); save_plot(output/"heading"/"heading_error_vs_time.png")


def main():
    p=argparse.ArgumentParser(); add_common_args(p); x=p.parse_args(); r,s=validate_inputs(x.real_dir,x.sim_dir); o=prepare_output(x.output_dir); run(r,s,o); print(o/"heading")
if __name__=="__main__": main()

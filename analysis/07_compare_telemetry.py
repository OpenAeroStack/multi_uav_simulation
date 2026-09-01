#!/usr/bin/env python3
"""Compare application-level GLOBAL_POSITION_INT telemetry continuity."""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from common import (add_common_args, load_or_create_alignment, number, prepare_output,
                    read_rows, save_plot, validate_inputs, write_csv)


def stream(directory,dataset,alignment):
    rows=read_rows(directory/"telemetry_timing.csv"); tc="elapsed_s" if dataset=="real" else "elapsed_time_s"
    zero=alignment[f"{dataset}_alignment_elapsed_s"]; end=alignment["comparison_end_s"]
    raw=np.asarray([number(row.get(tc)) for row in rows]); mission=raw-zero
    times=np.sort(raw[np.isfinite(raw)&(mission>=0)&(mission<=end)])
    intervals=np.diff(times)*1000.0
    return times-zero,intervals


def stats(times,intervals):
    duration=float(times[-1]-times[0]) if len(times)>1 else 0.0
    return {"message_count":len(times),"duration_s":duration,"mean_rate_hz":(len(times)-1)/duration if duration else float("nan"),
      "mean_interarrival_ms":float(np.mean(intervals)),"median_interarrival_ms":float(np.median(intervals)),
      "p95_interarrival_ms":float(np.percentile(intervals,95)),"max_interarrival_ms":float(np.max(intervals)),
      "gaps_gt_750ms":int(np.sum(intervals>750)),"gaps_gt_1000ms":int(np.sum(intervals>1000)),
      "gaps_gt_1500ms":int(np.sum(intervals>1500)),"gaps_gt_2000ms":int(np.sum(intervals>2000))}


def run(real_dir,sim_dir,output):
    a=load_or_create_alignment(real_dir,sim_dir,output); rt,ri=stream(real_dir,"real",a); st,si=stream(sim_dir,"sim",a)
    rs,ss=stats(rt,ri),stats(st,si)
    rows=[{"metric":key,"real":rs[key],"simulation":ss[key],"unit":("ms" if "interarrival" in key else "Hz" if key.endswith("hz") else "s" if key.endswith("_s") else "count")} for key in rs]
    rows.append({"metric":"estimated_mavlink_sequence_loss_percent","real":"not compared","simulation":"unavailable","unit":"diagnostic only"})
    write_csv(output/"telemetry"/"telemetry_metrics.csv",["metric","real","simulation","unit"],rows)
    for label,times,intervals,name,color in (("Real flight",rt,ri,"real_interarrival.png","tab:blue"),("Simulation",st,si,"sim_interarrival.png","tab:orange")):
        plt.figure(figsize=(8,4.5)); plt.plot(times[1:],intervals,label=f"{label} interarrival",color=color,linewidth=1)
        plt.axhline(1000,color="tab:red",linestyle="--",label="1 s continuity-gap reference")
        plt.xlabel("Mission time after takeoff (s)"); plt.ylabel("Application telemetry interarrival (ms)"); plt.title(f"{label}: GLOBAL_POSITION_INT-Equivalent Arrival Timing")
        plt.grid(True,alpha=.3); plt.legend(); save_plot(output/"telemetry"/name)
    upper=max(np.percentile(ri,99.5),np.percentile(si,99.5),1000); bins=np.linspace(0,upper,60)
    plt.figure(figsize=(8,4.5)); plt.hist(ri,bins=bins,density=True,alpha=.55,label="Real flight"); plt.hist(si,bins=bins,density=True,alpha=.55,label="Simulation")
    plt.xlabel("Application telemetry interarrival (ms)"); plt.ylabel("Probability density"); plt.title("Real vs Simulated Telemetry Interarrival Distribution")
    plt.grid(True,alpha=.3); plt.legend(); save_plot(output/"telemetry"/"real_vs_sim_interarrival_distribution.png")


def main():
    p=argparse.ArgumentParser(); add_common_args(p); x=p.parse_args(); r,s=validate_inputs(x.real_dir,x.sim_dir); o=prepare_output(x.output_dir); run(r,s,o); print(o/"telemetry")
if __name__=="__main__": main()

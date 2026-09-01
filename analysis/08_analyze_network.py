#!/usr/bin/env python3
"""Characterize simulation RF data without inventing a real RF counterpart."""

import argparse
import math
import numpy as np
import matplotlib.pyplot as plt
from common import (add_common_args, load_or_create_alignment, number, prepare_output,
                    read_rows, save_plot, validate_inputs, write_csv)


def aggregate(rows,zero,end,column,bin_s=1.0):
    t=np.asarray([number(r.get("elapsed_time_s"))-zero for r in rows]); v=np.asarray([number(r.get(column)) for r in rows])
    valid=np.isfinite(t)&np.isfinite(v)&(t>=0)&(t<=end); t,v=t[valid],v[valid]
    bins=np.floor(t/bin_s).astype(int); unique=np.unique(bins)
    return np.asarray([(bins==b).nonzero()[0].size and np.mean(t[bins==b]) for b in unique]),np.asarray([np.mean(v[bins==b]) for b in unique]),v


def run(real_dir,sim_dir,output):
    a=load_or_create_alignment(real_dir,sim_dir,output); rows=read_rows(sim_dir/"network.csv"); zero=a["sim_alignment_elapsed_s"]; end=a["comparison_end_s"]
    radio_count=len(read_rows(real_dir/"radio.csv")); summaries=[]
    for column,unit,plot,title,color in (("snr_db","dB","snr_vs_time.png","Simulated SNR vs Mission Time","tab:blue"),("signal_dbm","dBm","rssi_vs_time.png","Simulated Received Signal Level vs Mission Time","tab:orange")):
        t,mean,raw=aggregate(rows,zero,end,column)
        summaries.extend([{"metric":f"{column}_sample_count","value":len(raw),"unit":"count"},{"metric":f"{column}_mean","value":float(np.mean(raw)),"unit":unit},
                          {"metric":f"{column}_minimum","value":float(np.min(raw)),"unit":unit},{"metric":f"{column}_p05","value":float(np.percentile(raw,5)),"unit":unit}])
        plt.figure(figsize=(8,4.5)); plt.plot(t,mean,label="1 s mean",color=color)
        plt.xlabel("Mission time after takeoff (s)"); plt.ylabel(f"{column.replace('_',' ').title()} ({unit})"); plt.title(title+" (All Recorded PHY Receptions)"); plt.grid(True,alpha=.3); plt.legend(); save_plot(output/"network"/plot)
    summaries.extend([
      {"metric":"real_radio_sample_count","value":radio_count,"unit":"count"},
      {"metric":"simulation_rf_scope","value":"All delivered 802.11 PHY receive rows; not isolated application telemetry.","unit":""},
      {"metric":"real_rf_comparison_status","value":"Insufficient real radio samples for RF time-series validation.","unit":""},
      {"metric":"packet_loss_status","value":"Unavailable; no defensible application/network packet-loss measurement in these datasets.","unit":""},
      {"metric":"rtt_status","value":"Unavailable","unit":""},{"metric":"throughput_status","value":"Unavailable","unit":""}])
    write_csv(output/"network"/"network_summary.csv",["metric","value","unit"],summaries)


def main():
    p=argparse.ArgumentParser(); add_common_args(p); x=p.parse_args(); r,s=validate_inputs(x.real_dir,x.sim_dir); o=prepare_output(x.output_dir); run(r,s,o); print(o/"network")
if __name__=="__main__": main()

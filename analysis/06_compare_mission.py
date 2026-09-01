#!/usr/bin/env python3
"""Compare mission timing and the common final geographic target."""

import argparse
import math
import re
import numpy as np
import matplotlib.pyplot as plt
from common import (add_common_args, aligned_gps, coordinate_origin, gps_to_enu,
                    load_or_create_alignment, metric_rows, number, prepare_output,
                    read_rows, save_plot, validate_inputs, write_csv)


def find_event(rows,dataset,event_type,details_contains=None,after=-math.inf):
    tc="elapsed_s" if dataset=="real" else "elapsed_time_s"
    for row in rows:
        t=number(row.get(tc))
        details=row.get("details",row.get("description",""))+" "+row.get("value","")
        if row.get("event_type")==event_type and t>=after and (details_contains is None or details_contains in details): return t,row
    return None,None


def closest_distance(gps,target_e,target_n,until):
    mask=(gps["time"]>=0)&(gps["time"]<=until)
    return float(np.min(np.hypot(gps["east"][mask]-target_e,gps["north"][mask]-target_n)))


def cross_track(east,north,start_e,start_n,end_e,end_n):
    dx,dy=end_e-start_e,end_n-start_n
    return np.abs(dy*(east-start_e)-dx*(north-start_n))/math.hypot(dx,dy)


def run(real_dir,sim_dir,output):
    a=load_or_create_alignment(real_dir,sim_dir,output); rz=a["real_alignment_elapsed_s"]; sz=a["sim_alignment_elapsed_s"]
    revents,sevents=read_rows(real_dir/"events.csv"),read_rows(sim_dir/"events.csv")
    real_reached_abs,_=find_event(revents,"real","WAYPOINT_REACHED","seq=6",rz)
    sim_reached_abs,_=find_event(sevents,"simulation","WAYPOINT_REACHED",after=sz)
    real_end=a["real_mission_end_after_alignment_s"]; sim_end=a["sim_mission_end_after_alignment_s"]
    if real_reached_abs is None or sim_reached_abs is None: raise ValueError("Common final-target reached events were not found")
    real_reached,sim_reached=real_reached_abs-rz,sim_reached_abs-sz
    smission=read_rows(sim_dir/"mission.csv")
    target=next((row for row in smission if row.get("target_latitude_deg") and row.get("target_longitude_deg")),None)
    if target is None: raise ValueError("Simulation target coordinates are unavailable")
    target_lat,target_lon=number(target["target_latitude_deg"]),number(target["target_longitude_deg"])
    origin=coordinate_origin(sim_dir)
    te,tn,_=gps_to_enu(np.asarray([target_lat]),np.asarray([target_lon]),np.asarray([0.0]),origin); te,tn=float(te[0]),float(tn[0])
    real,sim=aligned_gps(real_dir,"real",a),aligned_gps(sim_dir,"sim",a)
    for gps in (real,sim):
        gps["east"],gps["north"],_=gps_to_enu(gps["lat"],gps["lon"],np.zeros_like(gps["lat"]),origin)
    waypoint_rows=[{
      "waypoint_id":"common_final_target_real_seq6_sim_guided_target","target_latitude_deg":target_lat,"target_longitude_deg":target_lon,
      "real_reached_time_s":real_reached,"sim_reached_time_s":sim_reached,"arrival_time_difference_s":sim_reached-real_reached,
      "real_closest_distance_m":closest_distance(real,te,tn,real_reached),"sim_closest_distance_m":closest_distance(sim,te,tn,sim_reached),
      "real_segment_duration_s":real_reached,"sim_segment_duration_s":sim_reached,
      "comparability_note":"Real AUTO waypoint 6 and the simulation GUIDED target terminate at the same geographic endpoint; intermediate real AUTO waypoints have no simulation equivalents."}]
    write_csv(output/"mission"/"waypoint_comparison.csv",list(waypoint_rows[0]),waypoint_rows)
    duration_diff=sim_end-real_end
    mission_values={"real_mission_duration_s":real_end,"sim_mission_duration_s":sim_end,"mission_duration_difference_s":duration_diff,
      "mission_duration_difference_percent":duration_diff/real_end*100.0,"final_target_arrival_time_difference_s":sim_reached-real_reached,
      "final_target_arrival_abs_difference_s":abs(sim_reached-real_reached)}
    units={key:("%" if key.endswith("_percent") else "s") for key in mission_values}
    write_csv(output/"mission"/"mission_metrics.csv",["metric","value","unit"],metric_rows(mission_values,units))

    start_e,start_n=0.0,0.0  # ENU origin is the declared start reference.
    cross_rows=[]
    row={"segment":"declared_origin_to_common_final_target"}
    for label,gps,reached in (("real",real,real_reached),("sim",sim,sim_reached)):
        mask=(gps["time"]>=0)&(gps["time"]<=reached)
        errors=cross_track(gps["east"][mask],gps["north"][mask],start_e,start_n,te,tn)
        row[f"{label}_mean_cross_track_error_m"]=float(np.mean(errors)); row[f"{label}_max_cross_track_error_m"]=float(np.max(errors))
    cross_rows.append(row)
    write_csv(output/"mission"/"cross_track_metrics.csv",list(row),cross_rows)

    real_points=[(0,"Takeoff")]; sim_points=[(0,"Takeoff")]
    for event in revents:
        if event.get("event_type")=="WAYPOINT_REACHED":
            t=number(event["elapsed_s"])-rz
            if 0<=t<=real_end: real_points.append((t,event.get("details","Waypoint")))
    sim_points.extend([(sim_reached,"Final target reached"),(sim_end,"LANDING mode")]); real_points.append((real_end,"Disarmed"))
    plt.figure(figsize=(9,4.8))
    for y,(label,points,color) in enumerate((("Real AUTO mission",real_points,"tab:blue"),("Simulation GUIDED mission",sim_points,"tab:orange"))):
        for t,text in points:
            plt.scatter(t,y,color=color,s=32); plt.annotate(text,(t,y),xytext=(2,7),textcoords="offset points",fontsize=7,rotation=25)
    plt.yticks([0,1],["Real AUTO","Simulation GUIDED"]); plt.xlabel("Mission time after takeoff (s)"); plt.title("Mission Event Timeline (Non-equivalent Mission Structures)")
    plt.grid(True,axis="x",alpha=.3); plt.legend(["Real events","Simulation events"],loc="lower right"); save_plot(output/"mission"/"mission_timeline.png")


def main():
    p=argparse.ArgumentParser(); add_common_args(p); x=p.parse_args(); r,s=validate_inputs(x.real_dir,x.sim_dir); o=prepare_output(x.output_dir); run(r,s,o); print(o/"mission")
if __name__=="__main__": main()

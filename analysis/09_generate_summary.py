#!/usr/bin/env python3
"""Generate thesis-ready summary tables and a cautious narrative report."""

import argparse
import json
import math
from common import (COLUMN_MAPPING, add_common_args, load_metric_file,
                    load_or_create_alignment, number, prepare_output, read_json,
                    read_rows, validate_inputs, write_csv)


def show(value, digits=3):
    if isinstance(value,(int,float)) and math.isfinite(float(value)): return f"{float(value):.{digits}f}"
    return str(value) if value not in (None,"") else "N/A"


def run(real_dir,sim_dir,output):
    a=load_or_create_alignment(real_dir,sim_dir,output)
    tr=load_metric_file(output/"trajectory"/"trajectory_metrics.csv"); al=load_metric_file(output/"altitude"/"altitude_metrics.csv")
    sp=load_metric_file(output/"speed"/"speed_metrics.csv"); hd=load_metric_file(output/"heading"/"heading_metrics.csv"); mi=load_metric_file(output/"mission"/"mission_metrics.csv")
    tele={r["metric"]:r for r in read_rows(output/"telemetry"/"telemetry_metrics.csv")}
    rows=[
      {"Category":"Trajectory","Metric":"Total horizontal distance","Real":show(tr.get("real_total_distance_m")),"Simulation":show(tr.get("sim_total_distance_m")),"Difference / Comparison":show(tr.get("distance_difference_m")),"Unit":"m"},
      {"Category":"Trajectory","Metric":"Horizontal RMSE","Real":"N/A","Simulation":"N/A","Difference / Comparison":show(tr.get("rmse_horizontal_error_m")),"Unit":"m"},
      {"Category":"Trajectory","Metric":"P95 horizontal error","Real":"N/A","Simulation":"N/A","Difference / Comparison":show(tr.get("p95_horizontal_error_m")),"Unit":"m"},
      {"Category":"Altitude","Metric":"Mean relative altitude","Real":show(al.get("real_mean_altitude_m")),"Simulation":show(al.get("sim_mean_altitude_m")),"Difference / Comparison":show(al.get("mean_altitude_error_m")),"Unit":"m"},
      {"Category":"Altitude","Metric":"Altitude RMSE","Real":"N/A","Simulation":"N/A","Difference / Comparison":show(al.get("rmse_altitude_m")),"Unit":"m"},
      {"Category":"Speed","Metric":"Mean measured groundspeed","Real":show(sp.get("real_mean_speed_mps")),"Simulation":show(sp.get("sim_mean_speed_mps")),"Difference / Comparison":show(sp.get("mean_speed_error_mps")),"Unit":"m/s"},
      {"Category":"Speed","Metric":"Groundspeed RMSE","Real":"N/A","Simulation":"N/A","Difference / Comparison":show(sp.get("rmse_speed_mps")),"Unit":"m/s"},
      {"Category":"Heading","Metric":"Circular heading MAE","Real":"N/A","Simulation":"N/A","Difference / Comparison":show(hd.get("mean_abs_heading_error_deg")),"Unit":"deg"},
      {"Category":"Mission","Metric":"Mission duration","Real":show(mi.get("real_mission_duration_s")),"Simulation":show(mi.get("sim_mission_duration_s")),"Difference / Comparison":show(mi.get("mission_duration_difference_s")),"Unit":"s"},
      {"Category":"Mission","Metric":"Common final-target arrival difference","Real":"N/A","Simulation":"N/A","Difference / Comparison":show(mi.get("final_target_arrival_time_difference_s")),"Unit":"s"},
    ]
    for metric,label,unit in (("mean_rate_hz","Mean application telemetry rate","Hz"),("median_interarrival_ms","Median interarrival","ms"),
                              ("p95_interarrival_ms","P95 interarrival","ms"),("max_interarrival_ms","Maximum interarrival","ms"),("gaps_gt_1000ms","Gaps > 1 s","count")):
        row=tele.get(metric,{})
        rows.append({"Category":"Telemetry","Metric":label,"Real":show(number(row.get("real"))),"Simulation":show(number(row.get("simulation"))),"Difference / Comparison":show(number(row.get("simulation"))-number(row.get("real"))),"Unit":unit})
    fields=["Category","Metric","Real","Simulation","Difference / Comparison","Unit"]
    write_csv(output/"tables"/"real_vs_sim_summary.csv",fields,rows)
    md=["# Real-flight versus simulation summary","", "| "+" | ".join(fields)+" |","|"+"|".join(["---"]*len(fields))+"|"]
    md.extend("| "+" | ".join(str(row[f]) for f in fields)+" |" for row in rows)
    (output/"tables"/"real_vs_sim_summary.md").write_text("\n".join(md)+"\n",encoding="utf-8")

    origin=read_json(output/"trajectory"/"coordinate_origin.json")
    direct=[k for k,v in COLUMN_MAPPING.items() if v["status"]=="DIRECTLY_COMPARABLE"]
    converted=[k for k,v in COLUMN_MAPPING.items() if v["status"]=="COMPARABLE_AFTER_CONVERSION"]
    unavailable=[k for k,v in COLUMN_MAPPING.items() if v["status"] in ("SIM_ONLY","REAL_ONLY","INSUFFICIENT_DATA")]
    waypoint=read_rows(output/"mission"/"waypoint_comparison.csv")[0]
    network={r["metric"]:r["value"] for r in read_rows(output/"network"/"network_summary.csv")}
    report=f"""REAL-FLIGHT VS SIMULATION COMPARISON REPORT
===========================================

Datasets
--------
Real: {real_dir}
Simulation: {sim_dir}

Availability
------------
Directly comparable signals: {', '.join(direct)}.
Comparable after conversion/alignment: {', '.join(converted)}.
Not directly comparable or insufficient: {', '.join(unavailable)}.

Time alignment
--------------
Mission time zero uses {a['method']}. Real alignment occurred at source elapsed {a['real_alignment_elapsed_s']:.3f} s and simulation alignment at {a['sim_alignment_elapsed_s']:.3f} s. The shared comparison interval is 0 to {a['comparison_end_s']:.3f} s and no data are extrapolated.

Coordinate alignment
--------------------
Both WGS84 GPS tracks were recomputed in a common ENU frame about ({origin['latitude_deg']:.9f}, {origin['longitude_deg']:.9f}, {origin['altitude_m']:.3f} m). Dataset-specific local-coordinate columns were not compared.

Results
-------
The trajectories have a horizontal RMSE of {show(tr.get('rmse_horizontal_error_m'))} m and P95 separation of {show(tr.get('p95_horizontal_error_m'))} m over the shared interval. Recorded horizontal distances are {show(tr.get('real_total_distance_m'))} m real and {show(tr.get('sim_total_distance_m'))} m simulated.

Relative-altitude RMSE is {show(al.get('rmse_altitude_m'))} m. Measured-groundspeed RMSE is {show(sp.get('rmse_speed_mps'))} m/s. Circular GPS-heading mean absolute error is {show(hd.get('mean_abs_heading_error_deg'))} degrees.

The common final target was reached at mission time {show(number(waypoint['real_reached_time_s']))} s in the real flight and {show(number(waypoint['sim_reached_time_s']))} s in simulation. The real mission used multiple AUTO waypoints while simulation used one GUIDED target, so intermediate waypoint timing is not treated as equivalent.

Application telemetry continuity
--------------------------------
Within the shared mission interval, mean equivalent GLOBAL_POSITION_INT arrival rates are {tele['mean_rate_hz']['real']} Hz real and {tele['mean_rate_hz']['simulation']} Hz simulated. Gaps are timing/continuity observations, not packet loss. Simulation MAVLink sequence numbers were unavailable, so sequence-loss estimates are not compared.

Network limitations
-------------------
{network.get('real_rf_comparison_status')}
Simulation SNR and signal level are characterized separately. No real SNR was invented. Packet loss, RTT, and throughput are unavailable as defensible comparable measurements.

Interpretation
--------------
These results quantify agreement and disagreement for the recorded runs; they do not by themselves establish general simulation validation. Differences include vehicle behavior, sampling, control mode, and mission structure as well as model fidelity.
"""
    (output/"summary"/"comparison_report.txt").write_text(report,encoding="utf-8")


def main():
    p=argparse.ArgumentParser(); add_common_args(p); x=p.parse_args(); r,s=validate_inputs(x.real_dir,x.sim_dir); o=prepare_output(x.output_dir); run(r,s,o); print(o/"summary")
if __name__=="__main__": main()

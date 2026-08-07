# Chapter 5 Results Map

## 1. Experimental Setup and Common Evaluation Procedure

| Subsection | Authoritative sources | Main table | Main figure | Supported claim | Caveat | Writing status |
|---|---|---|---|---|---|---|
| Platform/procedure | `results/MANIFEST.md`, primary README/provenance/registry | Platform, run unit, RNG/window rules | None | Controlled platform and run-level analysis defined | Some seeds only encoded by paired RNG registry | Ready to draft |
| Timing controls | Phase-A current summary/metrics | Emulation floor and logger overhead | Welch warm-up | NS-3/TAP floor dominates management path | Exact internal cause unconfirmed | Ready to draft |

## 2. Integration Validation

| Subsection | Authoritative sources | Main table | Main figure | Supported claim | Caveat | Writing status |
|---|---|---|---|---|---|---|
| Software/network integration | manifest, clean namespace/bridge/TAP logs, Phase-F metadata | Component and namespace mapping | At most architecture/topology figure | Gazebo/ArduPilot/ROS 2 and four network namespaces supported recorded experiments | Three-feed/coordinated-mission proof is distributed rather than one dedicated result | Needs concise synthesis |
| Mobility demonstration | final NetAnim XMLs | Trace status | Optional one screenshot | Four-node mobility recorded | Malformed endpoint; no packet events | Appendix preferred |

## 3. NS-3 Network Characterisation and Validation

| Subsection | Authoritative sources | Main table | Main figure | Supported claim | Caveat | Writing status |
|---|---|---|---|---|---|---|
| Friis validation | B1 raw/script/saved figures and manifest aggregate; `report_validation_v2/` | Fit statistics | `b1_predicted_vs_observed`, `b1_residuals_vs_distance` | R² 0.860, RMSE 0.77 dB, bias -0.77 dB | One five-position session; saved raw snapshot ends before four analysis windows | Drafted |
| RTT/emulation floor | Phase-A summary; manifest B2; `report_validation_v2/` | RTT by distance/floor | `b2_rtt_vs_distance` | Reported RTT is flat and floor-dominated | B2 is a manifest-supported aggregate without separately retained run-level logs | Drafted |
| Throughput/loss | manifest B3; authoritative B4 chain; `report_validation_v2/` | offered-load and distance-loss tables | `b3_throughput_and_loss`, `b4_loss_vs_distance` | Reported ~3 Mbps soft ceiling; B4 loss flat over range | B3 is a manifest-supported aggregate without separately retained run-level logs | Drafted |

## 4. Dynamic Link and Clustering Validation

| Subsection | Authoritative sources | Main table | Main figure | Supported claim | Caveat | Writing status |
|---|---|---|---|---|---|---|
| Dynamic links | corrected trial link/position CSVs; `report_clustering_v2/` | Three-trial RSSI/SNR/loss | `link_quality_by_trial`, `representative_rssi_timeline`, `representative_topology_initial_final` | Link quality varied during motion; Trial 3 had the weakest mean RSSI/SNR | Three corrected trials; link rows are within-trial observations | Drafted |
| CH roles | `clustering_validation_v2/canonical_trial_summary.csv`, transition audit and corrected assignments; `report_clustering_v2/` | canonical primary/backup transitions | `cluster_roles_timeline` | Primary retention 100% and zero primary changes; canonical backup transitions 2/4/3 | Controlled switching untriggered; emergency path partial and unvalidated | Drafted |

## 5. Middleware and Telemetry Validation

| Subsection | Authoritative sources | Main table | Main figure | Supported claim | Caveat | Writing status |
|---|---|---|---|---|---|---|
| Selected C0–C3 | exact selected raw files, combined summary and `report_validation_v2/` | Messages/rate/max gap | Optional `telemetry_rate_and_gap` | Telemetry active; selected Ground run has the lowest rate and largest gap | One selected run per condition; descriptive, not causal | Drafted |

## 6. Application-Layer Validation: Edge and Ground Processing

| Subsection | Authoritative sources | Main table | Main figure | Supported claim | Caveat | Writing status |
|---|---|---|---|---|---|---|
| D4 and final system comparison | D4 summary/details; primary registry and processed outputs | Accuracy + completion/latency/timing/resources | D4, processed ratio, median latency, CPU | Descriptive Edge/Ground differences across three verified pairs | D4 duplicate; small n; host resources | **Already drafted; consistency check only** |

## 7. Validation Against Project Objectives

| Subsection | Authoritative sources | Main table | Main figure | Supported claim | Caveat | Writing status |
|---|---|---|---|---|---|---|
| Objective evidence matrix | all authoritative sources | objective/evidence/scope/limitation/status | None | Evidence can be mapped without overclaiming | Exact objectives/status need human confirmation | Await decision |

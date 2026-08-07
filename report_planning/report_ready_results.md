# Report-Ready Results

## 1. Common Experimental Procedure

- Platform: Lenovo ThinkStation P3, Intel i7-13700 (16 cores/24 threads), 30 GiB RAM, Ubuntu 22.04.5, Gazebo 11.10.2, ROS 2 Humble, ns-3.38, ArduPilot commit `ec09e44a09`, YOLOv8n CPU inference.
- Independent unit: a complete run/trial. Frames, packets, messages and link rows summarise their parent run.
- RNG: final application comparison uses three matched RNG sessions with one verified run per mode.
- Official windows: Phase F uses metadata start/end rows after ≥9 processed warm-up rows, 15 s settling and a five-row stability gate; no official-window latency outlier was removed.
- Summaries use mean ± sample standard deviation across three run-level values per mode.
- Timing limitation: management veth mean RTT 0.032 ms versus 152.310–160.700 ms for tested NS-3/TAP conditions.
- Logger control: manifest reports 3.7 ± 0.5 ms overhead (n=86).

Required table: platform/procedure plus timing controls. Required figure: warm-up plot; ACF is appendix-only.

## 2. Integration Validation

Evidence: environment manifest; Phase-F metadata and successful metrics; `results_02` clean namespace/bridge/TAP logs. The system records Gazebo, ArduPilot and ROS 2 operation, mission telemetry, camera-based processing, and four namespace-to-TAP paths. Three camera feeds/coordinated mission evidence should be stated only to the extent corroborated by implementation/run artifacts, not inferred from NetAnim.

Recommended table: component, namespace/interface, evidence. Figures: one architecture/topology diagram and optionally one mission/camera illustration; avoid terminal screenshots.

## 3. NS-3 Network Characterisation and Validation

| Result | Authoritative source | Main numerical values | Required table | Required figure | Caveat |
|---|---|---|---|---|---|
| Friis comparison | B1 raw CSV/script/figures | R² 0.860; RMSE 0.77 dB; bias -0.77 dB; 58.5–113.8 m | Fit statistics | Existing predicted-observed and residual | One session/five positions |
| Emulation floor | Phase-A current RTT summary | 0.032 ms management; 158.280 colocated; 160.700 operational; 152.310 `chrt` | Floor conditions | Mark floor on RTT plot | Exact computational cause unconfirmed |
| RTT vs distance | Manifest B2; `report_validation_v2/b2_manifest_summary.csv` | means 153.3–159.9 ms; 0% loss at five positions | Five distances | `b2_rtt_vs_distance` | Manifest-supported descriptive result; detailed run-level logs were not retained separately |
| Throughput vs load | Manifest B3; `report_validation_v2/b3_manifest_summary.csv` | 1.69/2.85/3.02 Mbps received at 2/5/8 Mbps offered; 11/40/59.6% loss | Offered/received/loss | `b3_throughput_and_loss` | Manifest-supported descriptive result; detailed run-level logs were not retained separately |
| Loss vs distance | B4 summary + raw logs | TAP packet loss 13.25–13.96%; iperf 11–12% | Five B4 runs | Loss-distance | 10 s runs; packet rows not repetitions |

Observed soft goodput ceiling: approximately 3 Mbps under the tested configuration.

The subsection is drafted at `report_export/sections/chapter5/network_validation.tex`. Its five portable PNGs are under `report_export/images/chapter5/network/`; the complete report-ready evidence package is `results/phase_b_network/report_validation_v2/`.

## 4. Dynamic Link and Clustering Validation

| Trial | Duration (s) | Retained assignment states | Link rows | Mean RSSI (dBm) | Mean SNR (dB) | Mean obstacle loss (dB) | Primary | Canonical primary transitions | Canonical backup transitions | Events init/steady/total |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---|
| Corrected T1 | 69.842 | 35 | 5,802 | -58.770 | 35.230 | 2.072 | UAV3 (100%) | 0 | 2 | 0/3/3 |
| T2 | 69.800 | 35 | 5,874 | -59.516 | 34.484 | 1.854 | UAV3 (100%) | 0 | 4 | 8/4/12 |
| T3 | 69.781 | 35 | 5,808 | -62.373 | 31.627 | 2.705 | UAV1 (100%) | 0 | 3 | 9/3/12 |

The report-authoritative source is `results_02/network_evaluation/clustering_validation_v2/canonical_trial_summary.csv`. Its canonical metric uses complete assignment states after a uniform 0.01 s clustering-relative initialization interval; the first retained state is the baseline. Use the new canonical role timeline and one representative link-quality figure. Controlled primary switching was implemented in the contemporaneous committed node but not triggered. The threshold failure path was only partial—not guaranteed immediate backup promotion—and neither handover nor emergency failover was experimentally validated.

The subsection is drafted at `report_export/sections/chapter5/dynamic_link_clustering_validation.tex`. The report package is `results_02/network_evaluation/report_clustering_v2/`, with portable exports for the canonical role timeline, trial-level link comparison, corrected Trial 1 GCS-to-UAV RSSI timeline, and initial/final role topology.

## 5. Middleware and Telemetry Validation

| Condition | Selected raw file timestamp | Messages | Mean rate (Hz) | Maximum gap (ms) |
|---|---|---:|---:|---:|
| C0 mission/no vision | `012216` | 274 | 2.28 | 2243.2 |
| C1 static/no vision | `001716` | 253 | 2.11 | 1994.1 |
| C2 Edge | `012804` | 247 | 2.06 | 2354.9 |
| C3 Ground | `011400` | 184 | 1.53 | 3788.4 |

Use this table first; `telemetry_rate_and_gap` is the optional figure. Exclude header-only C2 attempts, contaminated C2 `004614`, and incomplete C3 `011330`. The mapping is resolved and the subsection is drafted at `report_export/sections/chapter5/middleware_telemetry_validation.tex`; its report package is `results/phase_c_middleware/report_validation_v2/`. Retain the limitation of one selected run per condition.

## 6. Application-Layer Validation

**Already drafted; consistency check only.**

- Report-authoritative D4 v2 (60 unique frames): Edge/raw precision 0.876623, recall 0.823171, F1 0.849057, exact-count 0.566667; Ground/Q5 1.000000, 0.176829, 0.300518, 0.433333. The original 61-row result is superseded provenance; v2 removes the later duplicated `frame_0033.png` occurrence.
- Processed-frame ratio: Edge 1.0000 ± 0.0000; Ground 0.7667 ± 0.0727.
- Mean of run median latency: 182.4050 versus 507.8583 ms; mean run p95: 421.2535 versus 562.9742 ms.
- Mean inference: 54.5237 versus 51.5667 ms; Ground compression/decode 2.5076/2.1362 ms.
- Detector-plus-relay CPU: 56.2167% versus 42.4413%; RSS: 952.3228 versus 963.3388 MB.
- Four main figures: D4 accuracy, processed-frame ratio, median pipeline latency, mean CPU utilisation.

## 7. Validation Against Project Objectives

| Evidence area | Completed scope | Remaining limitation | Recommended status wording |
|---|---|---|---|
| Integrated simulation/emulation | Gazebo/ArduPilot/ROS 2 and namespace/TAP operation recorded | Exact objective wording pending | “Implemented and integration-validated in the test environment” |
| Network model | Analytical fit, floor and loss/capacity characterisation | B2/B3 are manifest-supported aggregates without separately retained run-level logs | “Validated over the tested range, subject to emulation-floor limitation” |
| Dynamic clustering | Stable primary and canonical backup reselection observed | Controlled handover untriggered; emergency failover only partially implemented and untested | “Partially validated under nominal mobility” |
| Middleware/application | Selected telemetry, transport, corrected 60-frame D4 v2 and final paired runs | Small number of system-level runs | “Experimentally evaluated under controlled conditions” |
| Resource/efficiency objective | Detector-plus-relay CPU/RSS measured | No energy/battery/physical UAV measurement | “Host resource demand measured; energy impact not established” |

Final objective numbering and completion statuses require the authoritative project-objective list.

## 8. Appendix-Only Evidence

Full warm-up/ACF plots; raw RTT/iperf/TAP and telemetry traces; D4 per-frame details; raw pidstat; complete score arrays; original dynamic T1; startup events; NetAnim XML/status; manual LOS; excluded/superseded Phase-F runs; checksums/provenance.

## 9. Evidence Requiring Human Confirmation

Manual LOS inclusion; NetAnim placement; final objective statuses; Prism Discussion/Limitations labels.

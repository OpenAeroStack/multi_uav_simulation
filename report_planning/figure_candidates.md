# Figure Candidates

| Figure candidate | Existing source | Metric/evidence shown | Main report or appendix | Ready to use? | Required correction |
|---|---|---|---|---|---|
| Warm-up | Phase-A Welch PNG | Official warm-up rationale | Main | Yes after provenance check | Confirm raw series/window |
| ACF | Phase-A ACF PNG | Serial dependence | Appendix | Yes after provenance check | Confirm input |
| Predicted/observed SNR | B1 PNG | Friis validation | Main | Yes | None identified |
| B1 residuals | B1 PNG | Error by distance | Main/appendix | Yes | None identified |
| RTT vs distance | Generate from B2 if source restored | RTT/floor | Main | No | Raw B2 absent |
| Throughput/loss vs load | Generate from B3 if source restored | soft ceiling | Main | No | Raw B3 absent |
| B4 loss vs distance | Generate from B4 summary | TAP/iperf loss | Main | No | Use run-level points |
| Telemetry | Generate from selected mapping | rate/max gap | Optional | No | Exclude invalid runs |
| D4 accuracy | `.../figures/d4_detection_accuracy_comparison.{png,pdf}` | precision/recall/F1/exact count | Main | Yes | Caption 61 rows/60 unique IDs caveat |
| Processed-frame ratio | `mode_processed_ratio_summary` or `processed_frame_ratio_by_rng` | completion | Main | Yes | State ratio meanings by mode |
| Median latency | `mode_median_latency_summary` or per-RNG version | run median latency | Main | Yes | Prefer mode summary; n=3 |
| CPU utilisation | `mean_cpu_utilization_by_rng` | detector+relay CPU | Main | Yes | Scope in caption |
| Compression/decode | existing per-RNG figure | Ground codec overhead | Appendix/table | Yes | Ground-only |
| Inference/RSS/p95/paired latency | existing primary figures | supporting comparisons | Appendix | Yes | Avoid overcrowding main text |
| Cluster role timeline | Generate from corrected roles/events | primary/backup roles | Main | No | Mark startup window |
| Dynamic link quality | Generate from corrected links | RSSI/SNR during movement | Main | No | Predefine link/aggregation |
| Initial/final topology | Generate from positions/assignments | representative topology | Main/appendix | No | Select trial transparently |
| NetAnim screenshot | final trace before malformed endpoint | mobility | Appendix | No | At most one; no terminal screenshot |

The primary comparison contains 11 distinct figures, each as 600-DPI PNG and vector PDF. Use only four in the main application subsection: D4 accuracy, processed ratio, median latency, and CPU; retain the rest as optional/appendix evidence.

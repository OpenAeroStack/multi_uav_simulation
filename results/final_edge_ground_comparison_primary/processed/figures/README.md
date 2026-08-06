# Phase F figure index

Every figure is available as a 600-DPI PNG and a vector PDF. Points in mode-level figures are complete-run values, not frame-level replicates.

| Filename stem | Metric shown | Source CSV | Suggested report section / interpretation |
| --- | --- | --- | --- |
| `processed_frame_ratio_by_rng` | Official processed-frame ratio by RNG | `run_summary.csv` | Reliability: Edge local completion versus Ground complete compressed-frame processing. |
| `median_pipeline_latency_by_rng` | Official-window median pipeline latency | `run_summary.csv` | Latency comparison |
| `p95_pipeline_latency_by_rng` | Official-window p95 pipeline latency | `run_summary.csv` | Tail-latency comparison |
| `mean_inference_time_by_rng` | Mean YOLO inference time | `run_summary.csv` | Inference performance |
| `mean_cpu_utilization_by_rng` | Mean monitored process CPU on the simulation host | `run_summary.csv` | Host resource demand |
| `mean_rss_memory_by_rng` | Mean monitored process RSS on the simulation host | `run_summary.csv` | Host resource demand |
| `ground_compression_decode_overhead_by_rng` | Ground JPEG compression and decoding overhead | `run_summary.csv` | Image-processing overhead |
| `paired_median_latency_comparison` | Within-RNG Edge-to-Ground median latency changes | `paired_comparison.csv` | Paired latency comparison |
| `mode_processed_ratio_summary` | Mode means with three run-level ratio values | `mode_summary.csv + run_summary.csv` | Mode-level reliability summary |
| `mode_median_latency_summary` | Mode means with three run-level median latencies | `mode_summary.csv + run_summary.csv` | Mode-level latency summary |
| `d4_detection_accuracy_comparison` | Precision, recall, F1, and exact-count rate for Edge/raw and Ground/JPEG q5 | `phase_d_application processed D4 summary` | Detection accuracy under compression |

# Report value sources

| reported_value | source_file | source_columns | derivation |
| --- | --- | --- | --- |
| D4 precision, recall, F1, exact-count rate, frames | results/phase_d_application/processed/d4_detection_summary_d4_raw_vs_q5_01_20260805_221850.csv | `precision`, `recall`, `f1`, `exact_count_accuracy`, `frames` | Rows `edge_raw` and `ground_q5`. |
| Phase F per-RNG completion and official counts | results/final_edge_ground_comparison_primary/processed/run_summary.csv | `processed_frame_ratio`, `official_processed_frames`, `official_sent_frames` | One row per selected run. |
| Mode completion mean and sample SD | results/final_edge_ground_comparison_primary/processed/mode_summary.csv | `mean`, `sample_standard_deviation` where `metric=processed_frame_ratio` | Three run-level values per mode. |
| Median and p95 latency mean and sample SD | results/final_edge_ground_comparison_primary/processed/mode_summary.csv | `mean`, `sample_standard_deviation` for `median_latency_ms` and `p95_latency_ms` | Three run-level statistics per mode; no frame pooling. |
| Inference, detection count, CPU, RSS, compression, decode | results/final_edge_ground_comparison_primary/processed/mode_summary.csv | `mean`, `sample_standard_deviation` for the corresponding metric rows | Three run-level values per mode. |
| Within-RNG paired differences | results/final_edge_ground_comparison_primary/processed/paired_comparison.csv | `edge_*`, `ground_*`, and `ground_minus_edge_*` columns | One paired row per RNG. |
| Edge completion advantage | results/final_edge_ground_comparison_primary/processed/mode_summary.csv | `mean` for `processed_frame_ratio` | 100 × (Edge mean − Ground mean). |
| Ground-to-Edge latency ratio and latency difference | results/final_edge_ground_comparison_primary/processed/mode_summary.csv | `mean` for `median_latency_ms` | Ground mean ÷ Edge mean; Ground mean − Edge mean. |
| Accuracy differences | results/phase_d_application/processed/d4_detection_summary_d4_raw_vs_q5_01_20260805_221850.csv | `recall`, `f1`, `exact_count_accuracy` | Edge/raw value − Ground/q5 value. |
| CPU and RSS differences | results/final_edge_ground_comparison_primary/processed/mode_summary.csv | `mean` for `cpu_mean_percent` and `rss_mean_mb` | Edge CPU − Ground CPU; Ground RSS − Edge RSS. |
| Ground compression-plus-decode overhead | results/final_edge_ground_comparison_primary/processed/mode_summary.csv | `mean` for `mean_compression_ms` and `mean_decode_ms` | Sum of Ground run-level means. |

# Report figure plan

| Figure | Filename stem | Suggested caption | Report subsection | Placement | Main message |
| --- | --- | --- | --- | --- | --- |
| 1 | processed_frame_ratio_by_rng | Processed-frame completion for Edge local processing and Ground complete compressed-frame processing across the three RNG runs. | Processed-Frame Reliability | Main report | Edge completed every locally published official frame; Ground completion varied by RNG. |
| 2 | median_pipeline_latency_by_rng | Official-window median end-to-end pipeline latency for Edge and Ground processing. | End-to-End Pipeline Latency | Main report | Ground median latency was substantially higher in every RNG pair. |
| 3 | paired_median_latency_comparison | Within-RNG change in official-window median latency from Edge to Ground. | End-to-End Pipeline Latency | Main report | All three paired sessions show a latency increase for Ground processing. |
| 4 | d4_detection_accuracy_comparison | Phase D4 detection accuracy for raw Edge images and JPEG-quality-5 Ground images over 61 manually labelled frames. | Detection Accuracy under Compression | Main report | JPEG q5 retained precision but substantially reduced recall, F1 and exact-count rate. |
| 5 | mean_cpu_utilization_by_rng | Mean monitored process CPU utilization on the simulation host for each RNG run. | CPU and Memory Utilization | Main report | Edge processing required higher mean host CPU in all three pairs. |
| A.1 | p95_pipeline_latency_by_rng | Official-window p95 pipeline latency by RNG run. | Appendix: Tail Latency | Appendix | Tail latency complements the median comparison and retains official-window extremes. |
| A.2 | mean_inference_time_by_rng | Mean YOLO inference time by processing mode and RNG run. | Appendix: Inference Timing | Appendix | Inference time was similar between modes. |
| A.3 | mean_rss_memory_by_rng | Mean monitored process RSS on the simulation host. | Appendix: Host Resources | Appendix | Memory demand was similar relative to the latency and reliability differences. |
| A.4 | ground_compression_decode_overhead_by_rng | Mean JPEG compression and decoding time for Ground runs. | Appendix: Image-Processing Overhead | Appendix | Compression and decoding required only a few milliseconds. |
| A.5 | mode_processed_ratio_summary | Mode mean processed-frame ratio with all three run-level observations. | Appendix: Run-Level Summaries | Appendix | Displays the run-level basis of the mode reliability summary. |
| A.6 | mode_median_latency_summary | Mode mean of run median latencies with all three run-level observations. | Appendix: Run-Level Summaries | Appendix | Displays the run-level basis of the mode latency summary. |

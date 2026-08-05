# Final edge-versus-ground comparison

## 1. Experimental configuration

- Fixed pose: latitude 6.079430, longitude 80.193085, relative altitude 25.0 m.
- Yaw: 102.6 degrees, externally controlled by `goto_comparison_pose.py`.
- Input rate: 1 Hz; official measurement: 60 seconds; first 9 processed rows are warm-up.
- YOLO confidence: 0.25; debug images disabled; ground JPEG quality: 5.
- Three independent RNG sessions per mode. Each complete run, not each frame, is the independent experimental unit.

## 2. Selected runs

RNG 1 used edge then ground; RNG 2 used ground then edge; RNG 3 used edge then ground. Exact IDs and artifact paths are registered in `../selected_runs.csv`.

## 3. Per-run comparison

| rng_run | mode | run_id | official_sent_frames | official_processed_frames | processed_frame_ratio | mean_pipeline_latency_ms | p95_pipeline_latency_ms | mean_inference_ms | mean_detection_count |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | edge | f_edge_01_final_v2 | 60 | 60 | 1.0000 | 938.3622 | 5213.7405 | 50.9867 | 3.0833 |
| 1 | ground | f_ground_q5_01_final_v4 | 60 | 56 | 0.9333 | 507.2352 | 557.7750 | 51.4411 | 1.0536 |
| 2 | ground | f_ground_q5_02_final_v2 | 60 | 35 | 0.5833 | 508.6889 | 550.4030 | 47.9486 | 1.3143 |
| 2 | edge | f_edge_02_final | 60 | 60 | 1.0000 | 335.5532 | 553.1265 | 51.8417 | 3.1833 |
| 3 | edge | f_edge_03_final | 60 | 60 | 1.0000 | 216.7643 | 505.0150 | 56.6383 | 3.0333 |
| 3 | ground | f_ground_q5_03_final | 60 | 42 | 0.7000 | 512.6207 | 562.8050 | 54.7571 | 0.4048 |

## 4. Mode-level summary

These values summarize the three run-level values per mode; frames were not pooled as independent repetitions.

| mode | metric | n | mean | sample_standard_deviation | median | minimum | maximum |
| --- | --- | --- | --- | --- | --- | --- | --- |
| edge | processed_frame_ratio | 3 | 1.0000 | 0.0000 | 1.0000 | 1.0000 | 1.0000 |
| edge | mean_pipeline_latency_ms | 3 | 496.8932 | 386.9093 | 335.5532 | 216.7643 | 938.3622 |
| edge | p95_pipeline_latency_ms | 3 | 2090.6273 | 2704.8023 | 553.1265 | 505.0150 | 5213.7405 |
| edge | mean_inference_ms | 3 | 53.1556 | 3.0463 | 51.8417 | 50.9867 | 56.6383 |
| edge | mean_detection_count | 3 | 3.1000 | 0.0764 | 3.0833 | 3.0333 | 3.1833 |
| ground | processed_frame_ratio | 3 | 0.7389 | 0.1782 | 0.7000 | 0.5833 | 0.9333 |
| ground | mean_pipeline_latency_ms | 3 | 509.5149 | 2.7862 | 508.6889 | 507.2352 | 512.6207 |
| ground | p95_pipeline_latency_ms | 3 | 556.9943 | 6.2377 | 557.7750 | 550.4030 | 562.8050 |
| ground | mean_inference_ms | 3 | 51.3823 | 3.4046 | 51.4411 | 47.9486 | 54.7571 |
| ground | mean_detection_count | 3 | 0.9242 | 0.4683 | 1.0536 | 0.4048 | 1.3143 |

## 5. Paired RNG comparison

Latency differences are ground minus edge. No formal significance test is reported for only three pairs.

| rng_run | edge_ratio | ground_ratio | edge_advantage_percentage_points | ground_minus_edge_mean_pipeline_latency_ms |
| --- | --- | --- | --- | --- |
| 1 | 1.0000 | 0.9333 | 6.6700 | -431.1270 |
| 2 | 1.0000 | 0.5833 | 41.6700 | 173.1357 |
| 3 | 1.0000 | 0.7000 | 30.0000 | 295.8564 |

## 6. Resource comparison

CPU and RSS are summed across the monitored detector and relay processes at each pidstat sample, then summarized over the official window.

| rng_run | mode | resource_cpu_mean_percent | resource_cpu_peak_percent | resource_rss_mean_mb | resource_rss_peak_mb |
| --- | --- | --- | --- | --- | --- |
| 1 | edge | 54.3500 | 78.0000 | 975.1066 | 1002.5742 |
| 1 | ground | 50.7580 | 85.0000 | 968.3861 | 997.3164 |
| 2 | ground | 31.4333 | 84.0000 | 971.2721 | 1005.2617 |
| 2 | edge | 55.1833 | 77.0000 | 954.4010 | 972.0000 |
| 3 | edge | 58.8000 | 83.0000 | 987.7340 | 990.7812 |
| 3 | ground | 41.4667 | 88.0000 | 965.5916 | 982.6797 |

## 7. Main findings

- Every selected edge run completed 60/60 official frames in its local edge pipeline.
- Ground completed 56/60, 35/60 and 42/60 frames across RNG 1–3, showing session-dependent loss in the complete compressed-frame ground pipeline.
- Detection count differs between modes but is only the number of YOLO outputs per processed frame. It is not precision, recall, F1, or detection accuracy; those belong to Phase D4.
- Pipeline latency includes compression for ground. Compression and decoding are reported separately. `wireless_transit_ms` is not interpreted as pure network-only delay.

## 8. Limitations

- There are only three independent repetitions per mode, so generalization is limited.
- Frame-level summaries describe individual runs and are not treated as independent replicates.
- Saved NavSat altitude is interpreted according to the run metadata; relative-altitude provenance comes from the pose controller/runner, not by relabeling an AMSL field.
- Per-run yaw telemetry and the runner's DDS-match console line were not saved in the selected artifacts. Yaw was externally controlled at 102.6 degrees; endpoint matching is indirectly evidenced by successful post-gate frame flow.
- Resource files identify both processes as `python3`; aggregation therefore reports combined monitored pipeline resources, not process-specific attribution.

## 9. Excluded runs

| run_id | reason_excluded |
| --- | --- |
| edge_final_smoke_02 | short smoke test |
| edge_qos_smoke_01 | QoS diagnostic run |
| f_edge_01 | not selected as an official run |
| f_edge_01_final | superseded retry |
| f_ground_q5_01_final | superseded retry |
| f_ground_q5_01_final_v2 | not selected as an official run |
| f_ground_q5_02_final | not selected as an official run |

Raw and excluded artifacts remain unmodified in their original directories.

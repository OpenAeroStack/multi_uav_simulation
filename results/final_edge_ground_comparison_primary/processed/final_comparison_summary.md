# Definitive Edge-versus-Ground comparison

## 1. Experimental configuration

The fixed target was latitude 6.079430, longitude 80.193085, relative altitude 25.0 m, and externally commanded yaw 102.6 degrees. Relay rate was 1 Hz, the official window was 60 seconds, YOLO confidence was 0.25, debug publication was disabled by the runner, and Ground used JPEG quality 5.

## 2. Protocol revision and stabilization

Every selected run used at least nine processed warm-up rows, 15 seconds of additional settling, and a five-row stable-delivery gate with 0.70–1.30 second intervals. Official statistics use only metadata-recorded start/end boundaries. No official-window latency outlier was removed.

## 3. Final selected runs

| rng_run | actual_mode | run_id | official_sent_frames | official_processed_frames | processed_frame_ratio | median_pipeline_latency_ms | p95_pipeline_latency_ms | mean_inference_ms | resource_cpu_mean_percent | resource_rss_mean_mb | validation_warnings |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | edge | f_edge_01_primary1 | 58 | 58 | 1.0000 | 174.9800 | 228.5820 | 55.4310 | 54.6500 | 947.0567 | none |
| 1 | ground | f_ground_q5_01_primary | 60 | 49 | 0.8167 | 502.8800 | 560.4640 | 48.4633 | 43.5500 | 961.9036 | none |
| 2 | ground | f_ground_q5_02_primary | 60 | 48 | 0.8000 | 510.8350 | 559.2585 | 48.9979 | 43.0000 | 961.8003 | mean_interarrival_outside_0.8_to_1.2s |
| 2 | edge | f_edge_02_primary | 60 | 60 | 1.0000 | 168.0000 | 551.2300 | 51.0550 | 54.8333 | 957.2693 | none |
| 3 | ground | f_edge_03_final_v2 | 60 | 41 | 0.6833 | 509.8600 | 569.2000 | 57.2390 | 40.7738 | 966.3125 | mean_interarrival_outside_0.8_to_1.2s |
| 3 | edge | f_ground_q5_03_final_v3 | 60 | 60 | 1.0000 | 204.2350 | 483.9485 | 57.0850 | 59.1667 | 952.6424 | none |

## 4. Provenance of RNG3 mode labels

RNG3 Run IDs are inverted: `f_edge_03_final_v2` is Ground and `f_ground_q5_03_final_v3` is Edge. Detector, relay, metrics, metadata, and JPEG evidence agree on those actual modes. Files were not renamed.

## 5. Per-run results

All six official slices contain exactly the metadata-recorded processed-frame count. Frames summarize their parent run; they are not treated as independent experimental repetitions.

## 6. Processed-frame reliability

| mode | metric | n | mean | sample_standard_deviation | median | minimum | maximum |
| --- | --- | --- | --- | --- | --- | --- | --- |
| edge | processed_frame_ratio | 3 | 1.0000 | 0.0000 | 1.0000 | 1.0000 | 1.0000 |
| edge | median_latency_ms | 3 | 182.4050 | 19.2248 | 174.9800 | 168.0000 | 204.2350 |
| edge | p95_latency_ms | 3 | 421.2535 | 170.2158 | 483.9485 | 228.5820 | 551.2300 |
| edge | mean_inference_ms | 3 | 54.5237 | 3.1157 | 55.4310 | 51.0550 | 57.0850 |
| edge | cpu_mean_percent | 3 | 56.2167 | 2.5564 | 54.8333 | 54.6500 | 59.1667 |
| edge | rss_mean_mb | 3 | 952.3228 | 5.1138 | 952.6424 | 947.0567 | 957.2693 |
| ground | processed_frame_ratio | 3 | 0.7667 | 0.0727 | 0.8000 | 0.6833 | 0.8167 |
| ground | median_latency_ms | 3 | 507.8583 | 4.3388 | 509.8600 | 502.8800 | 510.8350 |
| ground | p95_latency_ms | 3 | 562.9742 | 5.4253 | 560.4640 | 559.2585 | 569.2000 |
| ground | mean_inference_ms | 3 | 51.5667 | 4.9196 | 48.9979 | 48.4633 | 57.2390 |
| ground | cpu_mean_percent | 3 | 42.4413 | 1.4700 | 43.0000 | 40.7738 | 43.5500 |
| ground | rss_mean_mb | 3 | 963.3388 | 2.5758 | 961.9036 | 961.8003 | 966.3125 |

Edge ratio means local edge-pipeline completion. Ground ratio means complete compressed-frame reception and processing; it is not interchangeable with Edge wireless delivery.

## 7. Latency comparison

Median and p95 latency are reported per run and summarized across the three run-level values per mode. Official outliers remain included. `wireless_transit_ms` is not interpreted as guaranteed pure radio propagation delay.

## 8. Inference and image-processing overhead

Inference is reported for both modes. JPEG compression and decoding are Ground-specific overheads and are shown separately in `ground_overhead_by_rng`.

## 9. CPU and memory results

CPU and RSS aggregate the monitored detector and relay processes on the simulation host. They measure host process demand, not physical-UAV energy or battery consumption.

## 10. Combined D4 accuracy and Phase F system results

D4 evaluates detection accuracy against manually labelled frames. Phase F evaluates live end-to-end system behavior; its detection count is not precision or recall.

| measure | Edge/raw | Ground/q5 | phase |
| --- | --- | --- | --- |
| Precision | 0.878205 | 1.000000 | D4 accuracy |
| Recall | 0.810651 | 0.171598 | D4 accuracy |
| F1 | 0.843077 | 0.292929 | D4 accuracy |
| Exact-count accuracy | 0.557377 | 0.426230 | D4 accuracy |
| Mean processed-frame ratio | 1.0000 | 0.7667 | Phase F system |
| Run median latency values (ms) | 174.98;168.00;204.24 | 502.88;510.83;509.86 | Phase F system |
| Mean of run median latencies (ms) | 182.4050 | 507.8583 | Phase F system |


D4 uses manually labelled frames for accuracy. Phase F measures live system completion and timing. Phase F `detection_count` is not precision, recall, or F1.

## 11. Main conclusions

The tables and paired plots show the observed reliability, latency, inference, and host-resource differences for these three controlled RNG pairs. Conclusions are descriptive: no significance test was performed with only three pairs.

## 12. Limitations

There are only three independent repetitions per mode, limiting generalization. Yaw was commanded externally rather than independently measured in each saved artifact. Host CPU/RSS cannot establish UAV power demand, and the available timing decomposition cannot isolate pure radio propagation delay.

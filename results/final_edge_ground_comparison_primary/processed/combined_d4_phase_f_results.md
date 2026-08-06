# Combined Phase D4 and Phase F results

## Detection accuracy from Phase D4

D4 evaluates detection accuracy against manually labelled frames.

| mode | precision | recall | f1 | exact_count_rate | evaluation_frames | image_representation |
| --- | --- | --- | --- | --- | --- | --- |
| Edge/raw | 0.878205 | 0.810651 | 0.843077 | 0.557377 | 61 | edge_raw |
| Ground/JPEG q5 | 1.000000 | 0.171598 | 0.292929 | 0.426230 | 61 | ground_q5 |

## Live system performance from Phase F

Phase F evaluates live end-to-end behavior over three independent runs per mode. Phase F detection count is not an accuracy metric. Edge processed-frame ratio denotes local pipeline completion; Ground processed-frame ratio denotes complete compressed-frame reception and processing.

| mode | mean_processed_frame_ratio | mean_of_run_median_latency_ms | mean_p95_latency_ms | mean_inference_ms | mean_detection_count | mean_cpu_percent | mean_rss_mb | mean_compression_ms | mean_decode_ms | independent_runs |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Edge/raw | 1.0000 | 182.4050 | 421.2535 | 54.5237 | 3.0567 | 56.2167 | 952.3228 |  |  | 3 |
| Ground/JPEG q5 | 0.7667 | 507.8583 | 562.9742 | 51.5667 | 1.1078 | 42.4413 | 963.3388 | 2.5076 | 2.1362 | 3 |

## Key cross-mode comparisons

| comparison | value |
| --- | --- |
| Edge completion advantage | 23.33 percentage points |
| Ground-to-Edge mean run-median latency ratio | 2.784× |
| Ground minus Edge mean run-median latency | 325.45 ms |
| Edge minus Ground recall | 0.639053 |
| Edge minus Ground F1 | 0.550148 |
| Edge minus Ground exact-count rate | 0.131147 |
| Edge minus Ground mean CPU utilization | 13.78 percentage points |
| Ground minus Edge mean RSS | 11.02 MB |
| Ground mean compression + decode overhead | 4.64 ms |

Cross-mode values use the three run-level summaries rather than pooling individual frames. No significance test is reported for three independent runs per mode.

# Conference-paper 1 Hz / 2 Hz rate-sweep summary

This directory contains the final analysis of the completed 1 Hz and 2 Hz conference workload experiments. Only the following official runs are included:

- `rate_edge_1hz_rng1`
- `rate_edge_1hz_rng2`
- `rate_edge_1hz_rng3`
- `rate_ground_1hz_rng1`
- `rate_ground_1hz_rng2`
- `rate_ground_1hz_rng3`
- `rate_edge_2hz_rng1`
- `rate_edge_2hz_rng2`
- `rate_edge_2hz_rng3`
- `rate_ground_2hz_rng1`
- `rate_ground_2hz_rng2`
- `rate_ground_2hz_rng3`

Failed, debug, `v2`, pilot, and 5 Hz runs are excluded.

## Methodology

Each run uses a fixed 60-second official measurement window. The offered application workload is approximately 60 frames at 1 Hz and approximately 120 frames at 2 Hz. Ground mode uses independently encoded JPEG quality 5 (Q5) images.

Pipeline latency is measured from frame admission at the relay to receipt of the corresponding detection result by the GCS result callback. Latency statistics include only frames whose corresponding results were successfully delivered to that callback. Processed-frame ratio and result-delivery ratio must therefore be considered alongside latency.

In particular, the Ground 2 Hz runs have very small successful-frame sample counts: 5, 7, and 7 results, respectively, from 120 offered frames per run. Their latency, CPU, and inference-time metrics must be interpreted together with their processed-frame ratios and must not be treated as representative of the full offered workload in isolation.

TAP bitrate is computed from interface byte-counter deltas over the official window. It includes all traffic observed on the interface, not only vision payload traffic.

Edge P95 latency exhibits substantial run-to-run variation caused by long-tail result-delivery behavior. The individual RNG observations are retained in the plots so this variability is not hidden by the group means.

## Outputs

- `all_official_runs.csv`: one row per included run, preserving every field from the existing per-run summaries.
- `aggregate_by_mode_rate.csv`: run-level means and sample standard deviations (`n = 3`, `ddof = 1`) grouped by processing mode and frame rate.
- `paper_table.csv`: reduced conference-paper table containing the requested processed-frame, result-delivery, latency, and TAP-bitrate statistics.
- `processed_frame_ratio_vs_rate.pdf` and `.png`: processed-frame ratio, with individual RNG observations and mean ± sample SD.
- `median_pipeline_latency_vs_rate.pdf` and `.png`: median pipeline latency, with individual RNG observations and mean ± sample SD.
- `uav_tx_bitrate_vs_rate.pdf` and `.png`: UAV-side TAP TX bitrate, with individual RNG observations and mean ± sample SD.

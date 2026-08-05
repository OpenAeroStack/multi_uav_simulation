# Revised edge-versus-ground comparison

## Outcome

The requested six-run selection cannot currently support the intended three-by-three primary comparison. Only `f_edge_01_final_v4` has complete provenance matching its requested mode. Four run IDs contain the opposite saved mode, and `f_ground_q5_02_final_v3` has no metadata from which to recover official CSV boundaries. No run was silently relabeled and no official boundary was guessed.

## Per-run audit and diagnostics

| rng_run | expected_mode | observed_mode | run_id | official_analysis_rows | processed_frame_ratio | mean_pipeline_latency_ms | p95_pipeline_latency_ms | verification_status | validation_warnings |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | edge | edge | f_edge_01_final_v4 | 60 | 1.0000 | 311.2253 | 585.7845 | verified_with_validation_warnings | latency_exceeds_2000ms |
| 1 | ground | edge | f_ground_q5_01_final_v7 | 60 | 1.0000 | 221.2147 | 528.4860 | provenance_failure | mode_mismatch;ground_jpeg_quality_not_5 |
| 2 | ground | ground | f_ground_q5_02_final_v3 |  |  |  |  | provenance_failure | metadata_missing;official_boundaries_unavailable;resource_file_missing |
| 2 | edge | ground | f_edge_02_final_v2 | 58 | 0.9667 | 499.5141 | 546.5295 | provenance_failure | mode_mismatch;edge_jpeg_quality_not_na |
| 3 | edge | ground | f_edge_03_final_v2 | 41 | 0.6833 | 504.2520 | 569.2000 | provenance_failure | mode_mismatch;edge_jpeg_quality_not_na;mean_interarrival_outside_0.8_to_1.2s |
| 3 | ground | edge | f_ground_q5_03_final_v3 | 60 | 1.0000 | 221.9465 | 483.9485 | provenance_failure | mode_mismatch;ground_jpeg_quality_not_5 |

Statistics shown above use only `rows[official_start_csv_row:official_end_csv_row]`. Official-window outliers are retained. A provenance failure remains visible diagnostically but is excluded from mode-level and paired claims.

## Paired RNG status

| rng_run | edge_run_id | ground_run_id | pair_status |
| --- | --- | --- | --- |
| 1 | f_edge_01_final_v4 | f_ground_q5_01_final_v7 | not_comparable_provenance_failure |
| 2 | f_edge_02_final_v2 | f_ground_q5_02_final_v3 | not_comparable_provenance_failure |
| 3 | f_edge_03_final_v2 | f_ground_q5_03_final_v3 | not_comparable_provenance_failure |

There are no complete verified pairs, so no significance testing or comparative mode finding is reported. Each complete run—not each frame—is the intended independent experimental unit.

## Interpretation boundaries

- Detection count is not precision or recall; detection accuracy belongs to Phase D4.
- Edge completion is local edge-pipeline completion, not wireless image delivery.
- Ground completion is complete compressed-frame reception and processing.
- CPU and RSS measure simulation-host process demand, not physical-UAV energy.
- `wireless_transit_ms` is not treated as pure radio delay.
- Yaw 102.6 degrees was externally commanded, not measured per run in these artifacts.

The initial curated analysis is preserved in `../../final_edge_ground_comparison_initial_v1/`. Original raw artifacts remain unchanged.

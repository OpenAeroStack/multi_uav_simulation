# Data checks

## Selected Phase F runs

Actual modes were verified using the detector startup log, camera-relay startup log, metrics evidence, and the final selection manifest.

| NS-3 RNG run | Run ID | Verified actual mode | Run ID matches actual mode |
| --- | --- | --- | --- |
| 1 | `f_edge_01_primary1` | Edge | Yes |
| 1 | `f_ground_q5_01_primary` | Ground | Yes |
| 2 | `f_ground_q5_02_primary` | Ground | Yes |
| 2 | `f_edge_02_primary` | Edge | Yes |
| 3 | `f_edge_03_final_v2` | Ground | No |
| 3 | `f_ground_q5_03_final_v3` | Edge | No |

The RNG3 Run IDs are inverted relative to the verified actual modes. The detector, relay, metrics, metadata, JPEG configuration, and namespace evidence agree on the actual modes. The source files were not renamed or modified.

## D4 labelled-frame check

- Authoritative detail file: `results/phase_d_application/processed/d4_detection_details_d4_raw_vs_q5_01_20260805_221850.csv`
- Total D4 data rows: 61
- Unique frame IDs: 60
- Duplicate frame IDs: `frame_0033.png` occurs twice
- Final evaluation-frame count used by the authoritative D4 summary: 61
- Authoritative summary file: `results/phase_d_application/processed/d4_detection_summary_d4_raw_vs_q5_01_20260805_221850.csv`
- Verified Edge/raw values: precision 0.878205, recall 0.810651, F1 0.843077, exact-count rate 0.557377
- Verified Ground/JPEG-quality-5 values: precision 1.000000, recall 0.171598, F1 0.292929, exact-count rate 0.426230

The duplicate is retained in the reported evaluation count because the required values and the authoritative summary are calculated over 61 rows. No source data were changed or reprocessed.

## Analysis-unit and window checks

- Independent runs per mode: 3
- One complete run was treated as one independent repetition.
- Frames were not treated as independent experimental repetitions.
- Phase F values use the official start and end rows recorded in each run's metadata.
- Startup rows, the settling period, and shutdown or late rows outside the official window were excluded by the authoritative analysis.
- Mode means and sample standard deviations were calculated from three run-level values per mode.

## Resource-monitoring scope

The runner invokes `pidstat` for the camera-relay PID and detector PID. The authoritative analysis sums CPU percentage and RSS across those two monitored processes at each timestamp, then summarizes the samples for the official run. These values therefore describe the monitored detector-plus-relay process set on the simulation host. They do not measure physical UAV energy, battery use, total platform power, or Raspberry Pi performance.

## Unresolved issues

- The D4 detail file has one duplicated frame ID (`frame_0033.png`), while the authoritative summary reports 61 evaluation frames. The export reports the authoritative metrics without deduplication and records both counts above.
- The RNG3 Run IDs do not match their verified actual modes. The mismatch is naming-only and is recorded above.

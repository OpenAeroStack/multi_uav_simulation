# Provenance and verification

## Mode determination

Actual processing mode was established from three independent saved sources for every selected run: the detector startup log, camera-relay startup log, and metrics CSV filename/logger startup log. All three agree for all six runs.

| RNG | Actual mode | Run ID | Detector | Relay | Metrics | Run ID matches mode |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Edge | `f_edge_01_primary1` | EDGE | EDGE | edge | yes |
| 1 | Ground | `f_ground_q5_01_primary` | GROUND | GROUND, JPEG 5 | ground | yes |
| 2 | Ground | `f_ground_q5_02_primary` | GROUND | GROUND, JPEG 5 | ground | yes |
| 2 | Edge | `f_edge_02_primary` | EDGE | EDGE | edge | yes |
| 3 | Ground | `f_edge_03_final_v2` | GROUND | GROUND, JPEG 5 | ground | no |
| 3 | Edge | `f_ground_q5_03_final_v3` | EDGE | EDGE, reliable output | edge | no |

The RNG3 mismatch is limited to Run ID naming. Since detector, relay, metrics, metadata mode, JPEG configuration, and namespace behavior agree, the artifacts are registered by their actual processing modes without renaming the originals.

## Configuration audit

All selected metadata records RNG run, 60-second duration, 1 Hz rate, confidence 0.25, fixed target pose, nine-row minimum warm-up, 15-second settling, five stable rows, the 0.70–1.30 second stability range, explicit official start/end rows, official sent/processed counts, and GPS before/after. Every Ground relay log records JPEG quality 5. All six resource-monitoring files are present.

The runner explicitly launches the detector with `publish_debug:=false`; this setting is not repeated in per-run metadata or detector startup logs. Yaw 102.6 degrees was externally controlled by `scripts/mission/goto_comparison_pose.py`; the saved artifacts do not provide independent per-run yaw measurement.

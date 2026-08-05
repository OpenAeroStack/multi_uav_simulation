# Provenance verification

## Fixed experimental conditions

- Latitude 6.079430, longitude 80.193085, relative altitude 25.0 m.
- Yaw 102.6 degrees, externally controlled using `scripts/mission/goto_comparison_pose.py --yaw-deg 102.6`.
- Relay rate 1.0 Hz; official measurement duration 60 seconds.
- First 9 processed metrics rows retained as warm-up and excluded from analysis.
- Ground JPEG quality 5; YOLO confidence threshold 0.25; debug image publication disabled.

## Selected runs and order

| RNG | First | Second |
| --- | --- | --- |
| 1 | `f_edge_01_final_v2` | `f_ground_q5_01_final_v4` |
| 2 | `f_ground_q5_02_final_v2` | `f_edge_02_final` |
| 3 | `f_edge_03_final` | `f_ground_q5_03_final` |

## Verification results

Each selected run has exactly one metrics CSV, metadata file, detector log, metrics logger log, camera relay log and pidstat resource file. Metadata Run ID, mode and RNG match the registry. Every metadata file records 60 seconds, 1.0 Hz, nine warm-up frames, confidence 0.25, the fixed target coordinates and relative-altitude label. Ground metadata records JPEG quality 5; edge records `n/a`.

| RNG | Mode | Official sent | Official processed | GPS before/after | Status |
| --- | --- | ---: | ---: | --- | --- |
| 1 | edge | 60 | 60 | present | verified with limitations below |
| 1 | ground | 60 | 56 | present | verified with limitations below |
| 2 | ground | 60 | 35 | present | verified with limitations below |
| 2 | edge | 60 | 60 | present | verified with limitations below |
| 3 | edge | 60 | 60 | present | verified with limitations below |
| 3 | ground | 60 | 42 | present | verified with limitations below |

Detector logs record model loading with confidence threshold 0.25 and the correct processing mode. Relay logs record 1.0 Hz and the correct edge RELIABLE or ground BEST_EFFORT output QoS; ground logs record JPEG quality 5. Metrics logs record the correct mode and Run ID. Absence of the detector's debug-publisher startup message is consistent with `publish_debug=false`, but the launch argument itself was not saved in metadata.

The saved GPS latitude/longitude readings are within approximately one metre of the fixed target and show negligible before/after horizontal drift; exact calculated drift is in `processed/run_summary.csv`. The NavSat `altitude` field must not be relabeled as relative altitude. Relative altitude 25.0 m is established by the runner target metadata and the pose controller, while the saved NavSat field retains its source semantics.

## Missing or indirect provenance

- Individual-run yaw telemetry was not written into metadata. The fixed yaw is recorded as externally controlled, not as a per-run measured value.
- The runner's `Detection DDS endpoints matched` console line was not redirected into a selected per-run artifact. Successful frame flow after the mandatory gate provides indirect evidence, but the line itself cannot be verified from saved files.
- `publish_debug=false` is not explicitly recorded in metadata; logs contain no debug publisher announcement.
- The separate earlier manual `ros2 topic echo --once /ap/v1/navsat` timeout is not part of these saved official runs and is not used to reject them.

No selected artifact is missing or ambiguous.

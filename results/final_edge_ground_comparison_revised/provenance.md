# Provenance audit

The exact six requested IDs were located in the original raw directories and registered without relabeling. Artifact availability and mode evidence are:

| Requested run | Expected | Saved evidence | Artifact result |
| --- | --- | --- | --- |
| `f_edge_01_final_v4` | edge | Metadata, CSV name, detector, logger, and relay all say edge | Complete; verified |
| `f_ground_q5_01_final_v7` | ground | Metadata, CSV name, detector, logger, and relay all say edge | Complete; mode conflict |
| `f_ground_q5_02_final_v3` | ground | CSV name and all node logs say ground | Metadata and resource file missing; official bounds unavailable |
| `f_edge_02_final_v2` | edge | Metadata, CSV name, detector, logger, and relay all say ground | Complete; mode conflict |
| `f_edge_03_final_v2` | edge | Metadata, CSV name, detector, logger, and relay all say ground | Complete; mode conflict |
| `f_ground_q5_03_final_v3` | ground | Metadata, CSV name, detector, logger, and relay all say edge | Complete; mode conflict |

For the five runs with metadata, the analysis checks run ID, RNG run, 60-second duration, 1 Hz rate, confidence 0.25, mode-appropriate JPEG setting, warm-up/stabilization fields, official bounds and counts, and both GPS messages. The saved metadata does not explicitly record `publish_debug=false`, yaw telemetry, or a textual “steady-state delivery confirmed” line. The stabilization parameters and official boundary fields demonstrate that the revised runner reached the post-stability recording stage, but this is not represented as direct saved console evidence.

Yaw was externally commanded by `scripts/mission/goto_comparison_pose.py --yaw-deg 102.6`; it is not claimed as per-run measured yaw. Original artifacts remain unchanged.

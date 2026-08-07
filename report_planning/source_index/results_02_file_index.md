# `results_02/` Source Index

## NetAnim

| Relative path | File type | Purpose | Status | Related experiment |
|---|---|---|---|---|
| `results_02/netanim/three_uav_20260725_171432_82636.xml` | XML | ~67.28 s four-node mobility | Supporting; malformed last value | NetAnim run 1 |
| `results_02/netanim/three_uav_20260725_174947_95620.xml` | XML | ~40.10 s four-node mobility | Supporting; malformed last node ID | NetAnim run 2 |
| matching `*_incomplete.xml` | Truncated XML | Interrupted originals | Excluded | Both runs |
| `results_02/netanim/latest.xml` | Symbolic link | Points to newer final XML | Convenience only | NetAnim run 2 |

## Corrected dynamic trials

The following pattern occurs in `dynamic_trial1`, `dynamic_trial2`, and `dynamic_trial3`.

| Relative path pattern | File type / main columns | Purpose | Status | Related experiment |
|---|---|---|---|---|
| `results_02/network_evaluation/dynamic_trial*/bag/bag_info.txt` | Text metadata | Duration, database size, message counts and topic list | Authoritative metadata | Dynamic trials |
| `.../bag/network_data/metadata.yaml` | ROS 2 bag metadata | Topic/type/count and storage description | Authoritative metadata | Dynamic trials |
| `.../bag/network_data/network_data_0.db3` | SQLite ROS bag | Raw messages | Authoritative raw | Dynamic trials |
| `.../extracted/positions.csv` | time, node ID, x/y/z | Node trajectories | Authoritative derived | Dynamic link |
| `.../extracted/network_links.csv` | time, metric, source, destination, value | RSSI/SNR/obstacle-loss long table | Authoritative derived | Dynamic link |
| `.../extracted/cluster_assignments.csv` | time, JSON assignment | roles, parent, route, score, status | Authoritative derived | Clustering |
| `.../extracted/cluster_roles.csv` | time, role, UAV ID | Primary/backup timeline | Authoritative derived | Clustering |
| `.../extracted/cluster_scores_raw.csv` | time, raw score array | Candidate scoring components | Authoritative derived | Clustering |
| `.../extracted/cluster_events.csv` | time, JSON event | Election/reselection events | Authoritative with startup caveat | Clustering |
| `.../extracted/summary.csv` | duration, samples/events/changes, initial/final CH, primary-time percentages | Trial summary | Authoritative with definitions caveat | Clustering |
| `.../logs/initial_*.txt`, `final_*.txt`, `cluster_events.txt` | ROS text snapshots | Human-readable state/event evidence | Supporting | Dynamic trials |

Bag metadata: corrected Trial 1 is 69.842 s, 2,538 messages, 297.2 KiB; Trial 2 is 69.800 s, 2,567 messages, 301.2 KiB; Trial 3 is 69.781 s, 2,546 messages, 297.2 KiB. Topics are `/clock`, positions, obstacle loss, RSSI, SNR, assignment, primary CH, backup CH, scores, and events.

## Original Trial 1

| Relative path | File type | Purpose | Status | Related experiment |
|---|---|---|---|---|
| `results_02/network_evaluation/dynamic_trial1_original/` | Same bag/CSV/log family | Older 119.835 s, 4,363-message, 70-snapshot trial | Separate/superseded for comparable analysis | Original dynamic T1 |

## LOS evidence

| Relative path | File type | Purpose | Status | Related experiment |
|---|---|---|---|---|
| `results_02/network_evaluation/los_stationary_trial1/bag/` | ROS bag, 29.778 s, 707 messages, 114.4 KiB | Stationary positions and physical-link metrics | Supporting | Stationary LOS |
| `results_02/network_evaluation/los_trial1/logs/initial_positions.txt` | ROS text | Initial four-node position snapshot | Supporting | Manual LOS |
| `results_02/network_evaluation/los_trial1/logs/los_dynamic_distance_5_samples.txt` | Text | Five manual link samples over ~30 s | Preliminary/non-normalised | Manual LOS |

## Integration and configuration

| Relative path | File type | Purpose | Status | Related experiment |
|---|---|---|---|---|
| `results_02/network_evaluation/cluster_event_qos.yaml` | YAML | Reliable, volatile, keep-last depth-100 QoS | Supporting configuration | Clustering |
| `.../logs/namespace_addresses_clean.txt` | Text | Namespace and IP mapping | Preferred supporting log | Integration |
| `.../logs/bridge_tap_configuration_clean.txt` | Text | Four bridges/TAPs up with carrier | Preferred supporting log | Integration |
| `.../logs/bridge_forwarding_status.txt` | Text | Bridge forwarding states | Supporting with repeated-line issue | Integration |
| non-clean namespace/bridge logs | Text | Raw command output | Supporting raw | Integration |

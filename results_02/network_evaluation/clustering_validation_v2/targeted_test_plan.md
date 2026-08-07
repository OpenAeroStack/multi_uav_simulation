# Targeted Clustering Test Plan

Do not run these tests until the contemporaneous `dynamic_cluster_manager` source/launcher is restored into a controlled test branch. No safe deterministic metric-injection hook was found in the current checkout.

## 1. Controlled primary-handover test

- **Purpose:** trigger `better_candidate_stable` and verify one canonical primary transition.
- **Initial state:** launch the discovered `dynamic_cluster_manager` in the three-UAV city pipeline; wait beyond startup and the configured 10 s primary hold with valid `/ns3_link_snr`, `/ns3_link_rssi`, `/link_obstacle_loss`, and `/uav_world_positions` inputs.
- **Controlled trigger:** keep one eligible challenger more than the configured 0.12 score margin above the current primary for at least three consecutive 2 s elections. Existing mission movement is not deterministic enough; a minimal test-only publisher/hook for the four metric topics is needed if geometry cannot guarantee this.
- **Expected topics/event:** `/cluster/assignment`, `/cluster/primary_ch`, `/cluster/backup_ch`, `/cluster/scores`, `/cluster/event`; event reason `better_candidate_stable` with matching old/new IDs.
- **Pass criteria:** exactly one sampled canonical primary transition after initialization; three winning epochs and holding period evident; a valid new backup and complete assignment published; event matches state change.
- **Record:** challenger/current scores, epochs, hold time, role timeline and transition latency.
- **Duration:** about 30–45 s after readiness.
- **Save:** ROS bag, bag metadata, manager log, injected-metric log/config, extracted assignments/roles/scores/events/links, canonical v2 output.
- **Startup control:** begin trigger only after 0.01 s exclusion, metrics readiness, a stable baseline and the 10 s hold.

## 2. Emergency failover test

- **Purpose:** characterize the implemented `primary_link_failure` path and expose whether the stored backup is promoted.
- **Initial state:** stable primary and eligible backup, with all required metric streams fresh.
- **Controlled trigger:** drive only the active primary's GCS SNR below the implemented -2 dB threshold while keeping the backup eligible. A deterministic test metric publisher/hook is required; stopping all metrics would only exercise the global stale-data guard.
- **Expected topics/event:** same cluster topics; event reason `primary_link_failure`, complete old/new IDs, then recomputed backup.
- **Pass criteria for implemented path:** next election changes primary to the highest-ranked eligible candidate, publishes a complete state/event, and selects a distinct valid backup or explicitly reports backup 0 when none exists. Do not call this guaranteed backup promotion unless the old backup is demonstrably selected by policy.
- **Record:** threshold crossing, election-to-transition delay, old backup versus new primary, scores, data freshness and any no-candidate outcome.
- **Duration:** about 20–30 s after stable baseline.
- **Save:** same artifacts as handover test, plus failure-injection timestamps.
- **Startup control:** inject only after stable assignment and metric readiness; retain a pre-trigger baseline window.

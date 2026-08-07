# Dynamic Link and Clustering Results

## Purpose
Evaluate mobile link quality, cluster assignments, routes, candidate scores, and CH stability.

## Source directories
`results_02/network_evaluation/dynamic_trial1/`, `dynamic_trial2/`, `dynamic_trial3/`; report-authoritative derived analysis in `clustering_validation_v2/`; original Trial 1 is separate.

## Experiment design
Three comparable ~70 s mobile trials recording four-node positions/link metrics and 45 cluster snapshots; one older ~120 s run records 70 snapshots.

## Independent experimental unit
One complete dynamic trial. Link rows and time samples are repeated observations.

## Available runs
Corrected T1 69.842 s/2,538 bag messages; T2 69.800 s/2,567; T3 69.781 s/2,546. Original T1 119.835 s/4,363.

## Main files
Per trial: bag metadata/database; `positions.csv`, `network_links.csv`, `cluster_assignments.csv`, `cluster_roles.csv`, `cluster_scores_raw.csv`, `cluster_events.csv`, `summary.csv`; initial/final/event logs.

## Main measured metrics
Position, RSSI, SNR, obstacle loss, candidate score, parent/route, primary/backup role, election event and duration.

## Verified numerical results
Each corrected trial contains 45 complete assignments, of which 35 remain after the common 0.01 s clustering-relative initialization interval. Primary: T1 UAV3 100%, T2 UAV3 100%, T3 UAV1 100%, with zero canonical primary transitions. Canonical backup transitions: 2, 4, 3. Total events: 3, 12, 12; initialization/steady split: 0/3, 8/4, 9/3. Mean RSSI: -58.770, -59.516, -62.373 dBm; mean SNR: 35.230, 34.484, 31.627 dB; mean obstacle loss: 2.072, 1.854, 2.705 dB.

## Results suitable for the main report
Trial summary, canonical role timeline, trial-level link-quality comparison, corrected Trial 1 GCS-to-UAV RSSI timeline, and representative initial/final role topology. The subsection is drafted at `report_export/sections/chapter5/dynamic_link_clustering_validation.tex`.

## Results better suited to an appendix
Raw scores, full event payloads, ROS topic counts, and original Trial 1.

## Known issues and limitations
Legacy event and role-topic change totals use different units from the canonical assignment-state metric. Controlled handover code existed but was untriggered. The failure path was partial rather than guaranteed backup promotion, and no dedicated handover/failover test was found.

## Recommended Chapter 5 destination
Section 4.

## Processing still required
The canonical aggregation, figures and Chapter 5 subsection are complete. Only final report-integration consistency checks remain. Execute the versioned targeted handover/failover tests only if they become a separate project requirement; do not infer their outcome.

## Suggested tables
Per-trial duration/rows/CH/change/link means; event classification.

## Suggested figures
Use the four registered exports under `report_export/images/chapter5/clustering/`: `cluster_roles_timeline`, `link_quality_by_trial`, `representative_rssi_timeline`, and `representative_topology_initial_final`.

## Claims supported by the evidence
Primary retention was 100% with zero canonical primary transitions; canonical backup transitions were 2/4/3; assignment/score/role/event publication and dynamic link variation were recorded. Initial election, scoring, primary selection and backup selection/reselection were implemented and observed.

## Claims not supported by the evidence
That controlled handover was experimentally validated, that emergency failover guarantees immediate stored-backup promotion, that failover recovery time was measured, or that three three-UAV trials establish larger-swarm behaviour.

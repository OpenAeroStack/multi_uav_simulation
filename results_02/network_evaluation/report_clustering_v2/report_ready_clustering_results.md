# Dynamic Link and Clustering Validation

## Trial Configuration

The comparison used three corrected dynamic trials, each lasting about 70 s. A common 0.01 s initialization interval was removed before transitions were counted, leaving 35 complete assignment states in each trial. The older original Trial 1 was excluded because its duration and sampling made it unsuitable for direct comparison with the corrected runs.

## Dynamic Link Quality

| Trial | Mean RSSI (dBm) | Mean SNR (dB) | Mean obstacle loss (dB) |
|---|---:|---:|---:|
| Trial 1 | -58.770 | 35.230 | 2.072 |
| Trial 2 | -59.516 | 34.484 | 1.854 |
| Trial 3 | -62.373 | 31.627 | 2.705 |

Trial 1 had the strongest average RSSI and SNR, with Trial 2 close behind. Trial 3 recorded the weakest mean RSSI and SNR and the largest mean obstacle loss. These are trial summaries, so they do not show every short degradation. The corrected Trial 1 GCS-to-UAV timeline makes those temporary changes visible without mixing in UAV-to-UAV links.

The initial/final topology uses the first and last canonically retained Trial 1 assignments. Positions were selected from the closest available timestamp, and the plot shows roles only rather than inferred communication paths.

## Primary Cluster-Head Stability

| Trial | Primary | Retention (%) | Canonical primary transitions |
|---|---|---:|---:|
| Trial 1 | UAV3 | 100 | 0 |
| Trial 2 | UAV3 | 100 | 0 |
| Trial 3 | UAV1 | 100 | 0 |

The experiments demonstrated stable primary leadership under the recorded mobility conditions. They did not demonstrate a primary handover because the selected primary never changed in any corrected trial.

## Backup Cluster-Head Reselection

Canonical backup transitions were 2, 4 and 3 in Trials 1, 2 and 3. These counts came from changes between complete assignment states after initialization. Repeated event publications were not treated as separate transitions, and the first retained assignment provided the baseline. Event-message totals and old `backup_changes` values therefore differ from the canonical counts.

## Implementation versus Experimental Validation

Initial election, scoring, primary selection, and backup selection/reselection were implemented and observed. Controlled primary switching also existed and used score-margin, consecutive-win and holding-time checks, but the path was not triggered in the corrected runs. The evidence therefore validates stable leadership rather than handover behaviour.

The primary-link-failure path was only partial. It selected the best eligible candidate rather than guaranteeing immediate promotion of the stored backup. No dedicated emergency-failover experiment was found, and no recovery time was measured.

## Claims Supported

- Dynamic position and link-quality measurements were produced in all three corrected trials.
- Initial primary and backup selection was observed.
- Primary retention was 100%, with zero canonical primary transitions.
- Canonical backup transitions were 2, 4 and 3.
- Candidate scoring and backup reselection were implemented and observed.

## Claims Not Supported

- Primary switching or handover was experimentally validated.
- Emergency failover or immediate stored-backup promotion was demonstrated.
- Failover recovery time was measured.
- The implementation provides fully validated fault tolerance.
- Event-message totals are equivalent to canonical role transitions.
- Three three-UAV trials establish behaviour for larger swarms.

## Main-Report Content

Use the compact trial and link-quality tables, the canonical role timeline, the trial-level link comparison, the corrected Trial 1 RSSI timeline, and the initial/final role topology. Keep the distinction between implementation and experimental validation in the main text.

## Appendix-Only Content

The full transition audit, raw score arrays, event payloads, startup events, detailed extracted CSVs, ROS bag metadata and original Trial 1 are better kept as supporting material.


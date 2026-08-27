# Clustering Control-Flow Verification

## Source Implementation

The executed clustering implementation is `ros2/uav_controller/uav_controller/dynamic_cluster_manager.py`. It is present in the current working tree at `09cff0a6ab72a62bfeb043791a775de214ee70c1` (`dynamic-data`). The file was introduced by `4f7002c93ba26064b682c2f2fe40c60fb05ffb33`; `git diff` confirms that the file is unchanged between that commit and the current commit. No historical source recovery was therefore required.

- Class: `DynamicClusterManager`, lines 33–802.
- Configuration and timer creation: `__init__`, lines 34–227, especially lines 40–50, 71–95, and 220–223.
- Input freshness/completeness gate: `required_links` and `metrics_ready`, lines 322–361.
- Candidate scoring and eligibility: `mobility_stability` and `calculate_score`, lines 363–504.
- Assignment construction: `create_assignments`, lines 506–587.
- State and event publication: `publish_state`, lines 589–668.
- Election, normal switching, failure replacement, and backup recalculation: `election_callback`, lines 670–802.

The requested `results_02/network_evaluation/...` directories are absent from the current checkout. They were inspected read-only from Git history. Commit `5e08a42c24ada6bb77bd8d37e10979de0e65e230` contains the three corrected raw trials, and commit `62cb0af1a62b0fd0c13d9cd07f87605205a32933` contains `clustering_validation_v2`. No checkout or working-tree change was used to inspect them.

## Normal Controlled Primary Switching

| Behaviour | Implemented? | Value/Condition | Code evidence |
|---|---|---|---|
| Election period | Yes | Default 2.0 s; the timer invokes `election_callback` every configured `election_period` | Lines 41, 71–73, 220–223 |
| Periodic score reevaluation and ranking | Yes | Every ready election calculates all UAV scores, retains only eligible candidates, sorts by descending score, and breaks equal-score ties by ascending UAV ID | Lines 684–701 |
| Incumbent treatment | Yes | The highest-ranked candidate is only a proposal. If it differs from the incumbent and the incumbent has not failed, switching is subject to hold, margin, and consecutive-win checks | Lines 703–765 |
| Switching margin | Yes | Dimensionless score margin `0.12`; the strict condition is `challenger_score > current_score + 0.12` | Lines 44, 80–82, 742–745 |
| Consecutive wins | Yes | Three qualifying ready election epochs by the same proposed challenger; assignment occurs when `challenger_wins >= 3` | Lines 45, 83–85, 747–758 |
| Holding period | Yes | At least 10.0 s since the last primary change: elapsed time `>= minimum_ch_hold` | Lines 43, 77–79, 737–740, 767–768 |
| Challenger counter update | Yes | The count increments when the same proposed challenger again satisfies both hold and margin; a different qualifying challenger starts at one | Lines 747–752 |
| Challenger counter reset | Yes | Reset after a controlled switch, on failure replacement, when hold or margin is false, or when there is no distinct proposed challenger. A stale-data early return pauses the counter rather than resetting it | Lines 719–723, 754–765; stale return at 670–682 |
| Primary assignment | Yes | A distinct proposed challenger becomes primary when hold and margin are satisfied and its win count reaches at least three | Lines 725–758 |
| Controlled-switch event | Yes | A role change is published with reason `better_candidate_stable` | Lines 754–758, 646–668, 782–802 |
| Backup recalculation | Yes | After the primary decision, the first ranked eligible candidate other than the resulting primary becomes backup; otherwise backup is `0` | Lines 770–780 |

The constants are not dormant: all three are read into instance fields and used in the normal primary-update branch. “Consecutive” here means consecutive qualifying, metrics-ready election callbacks. A stale callback returns before the counter is updated or reset.

## Primary-Link Failure Handling

| Behaviour | Implemented? | Exact behaviour | Code evidence |
|---|---|---|---|
| Primary GCS-SNR check | Yes | Uses the current primary's `score_details[primary_ch]["gcs_snr_db"]`, which comes from the SNR link `(0, primary_ch)` | Lines 399–405, 494–498, 707–713 |
| Failure threshold | Yes | Default exactly `-2.0 dB`; failure is a strict `<` comparison, so exactly `-2.0 dB` does not trigger it | Lines 49, 90–92, 709–713 |
| Bypass of holding period | Yes | The `current_failed` branch assigns the proposed primary before the normal branch computes `held_long_enough` | Lines 719–725 versus 737–740 |
| Bypass of margin | Yes | The failure branch assigns directly and does not evaluate `clearly_better` | Lines 719–725 versus 742–747 |
| Bypass of consecutive wins | Yes | The failure branch assigns directly and resets the challenger state; it does not enter the win-count branch | Lines 719–723 versus 747–758 |
| Replacement candidate selection | Yes | `self.primary_ch = proposed_primary`, where `proposed_primary = candidates[0]`, the highest-scoring eligible candidate (lowest UAV ID on an exact score tie) | Lines 689–701, 719–721 |
| Explicit stored-backup promotion | No | There is no `self.primary_ch = self.backup_ch` assignment. The old stored backup is not referenced by the failure decision | Lines 703–723 |
| Stored-backup preference | No | Candidate sorting uses only score and UAV-ID tie-breaking; prior backup status is absent from the key and failure branch | Lines 689–701, 719–723 |
| Can the old backup become primary? | Yes, conditionally | It becomes primary only if it is `candidates[0]`, i.e. the highest-ranked eligible candidate under the score/ID ordering | Lines 689–701, 719–721 |
| Can another UAV become primary instead? | Yes | Any other eligible UAV ranked above the old backup is `candidates[0]` and is assigned instead | Lines 689–701, 719–721 |
| Backup recalculation | Yes | After failure assignment, candidates are filtered against the new primary and the highest remaining candidate becomes backup | Lines 767–780 |
| No eligible candidate | Yes | `proposed_primary` is `0`; the failed primary is replaced by `0`, backup also becomes `0`, assignments have `NO_GATEWAY`/disconnected state, and the primary change is published | Lines 523–568, 599–605, 701, 719–721, 770–800 |
| Failure event | Yes | A changed state is published with reason `primary_link_failure` and old/new primary and backup IDs | Lines 719–723, 646–668, 782–802 |

With the default thresholds, a failed primary (`GCS SNR < -2.0 dB`) cannot remain eligible (`GCS SNR >= 3.0 dB`). The exact implementation is therefore equivalent to `new_primary = best_eligible_candidate`, not `new_primary = current_backup`.

## Stale Measurement Behaviour

The clustering gate checks the most recent update time for the global SNR, obstacle-loss, and position streams. Each must have been updated within the default 5.0 s timeout; a missing stream yields `waiting_for_<metric>` and an old stream yields `stale_<metric>`. RSSI is subscribed and reported but is not included in this freshness gate. Lines 42, 74–76, and 331–342 provide this evidence.

The gate also requires every pair among GCS/UAV IDs `0..num_uavs` to exist in both the SNR and obstacle-loss dictionaries. Missing pairs produce `missing_snr_links=...` or `missing_obstacle_links=...` (lines 322–359). It does not separately verify that every UAV has a position entry once the global position stream is fresh.

If readiness fails, `election_callback` logs a throttled warning and returns at lines 670–682. It does not score candidates, check primary failure, promote the backup, replace the primary, recalculate the backup, or publish a new state/event for that callback. Existing role fields and any challenger count remain in memory unchanged. Thus stale required metrics stop/prevent that election; they do not trigger emergency promotion or failover.

## Candidate Eligibility

An election is attempted only after the global readiness checks above pass. For an individual UAV, `calculate_score` marks it eligible when both conditions hold (lines 489–492):

1. Its GCS SNR is `>= 3.0 dB` (`candidate_min_gcs_snr_db`, lines 48, 87–89).
2. It has at least one other UAV link with SNR `>= 5.0 dB` (`member_min_snr_db`, lines 50, 93–95, 414–447).

Eligible candidates are ranked by a valid finite score computed from normalized GCS SNR, neighbor quality/coverage, relative-mobility stability, and obstacle robustness with normalized weights 0.40/0.30/0.20/0.10 (lines 52–62, 97–127, 399–504). Global readiness ensures all required SNR and obstacle links exist and the SNR/obstacle/position streams are fresh before these scores are used. RSSI is not a score term and is not an eligibility condition.

## Experimental Evidence

Canonical transitions below use complete `/cluster/assignment` states after the validation-v2 0.01 s initialization exclusion. Raw event totals and the legacy sampled-topic `backup_changes` field are not substituted for canonical transitions.

| Trial | Primary transitions | Backup transitions | Failure path triggered? | Controlled switch triggered? |
|---|---|---|---|---|
| `dynamic_trial1` | UAV3 throughout; 0 canonical transitions | UAV1 → UAV2 at 12.898971 s → UAV1 at 28.899355 s; 2 canonical transitions | No; active-primary raw GCS-SNR minimum 5.046 dB and no `primary_link_failure` event | No; no primary transition or `better_candidate_stable` event |
| `dynamic_trial2` | UAV3 throughout; 0 canonical transitions | UAV1 → UAV2 at 8.661171 s → UAV1 at 22.661242 s → UAV2 at 56.661139 s → UAV1 at 64.661178 s; 4 canonical transitions | No; active-primary raw GCS-SNR minimum 9.457 dB and no `primary_link_failure` event | No; no primary transition or `better_candidate_stable` event |
| `dynamic_trial3` | UAV1 throughout; 0 canonical transitions | UAV2 → UAV3 at 2.164162 s → UAV2 at 28.164132 s → UAV3 at 36.164247 s; 3 canonical transitions | No; active-primary raw GCS-SNR minimum 16.799 dB and no `primary_link_failure` event | No; no primary transition or `better_candidate_stable` event |

Evidence sources are the historical `cluster_assignments.csv`, `cluster_events.csv`, `network_links.csv`, and `clustering_validation_v2/{canonical_trial_summary.csv,transition_audit.csv}` files. Across corrected trials, event reasons are only `initial_election` and/or `backup_reselected`; neither primary-switch reason appears. A filename and content search under `results/` and historical `results_02/` found no dedicated handover/failover result. The validation package explicitly supplies only a future `targeted_test_plan.md`, not a completed failure experiment. Therefore failure-triggered replacement and controlled primary handover are implemented and reachable but neither was triggered, observed, or experimentally validated in these trials.

## Final Verdict

1. **TRUE — “The primary-link failure threshold is -2 dB.”** The configured default is exactly `-2.0 dB`, used in the current-primary GCS-SNR test. More precisely, failure occurs only when SNR is strictly below it.
2. **TRUE — “The failure path bypasses the normal switching margin.”** Direct assignment at lines 719–721 precedes and excludes the normal margin branch.
3. **TRUE — “The failure path bypasses the consecutive-win requirement.”** It assigns immediately on that ready election callback and resets, rather than increments, the challenger counter.
4. **TRUE — “The failure path bypasses the minimum holding period.”** The failure branch never evaluates `held_long_enough`.
5. **FALSE — “The stored backup is immediately promoted when the primary fails.”** The code assigns `proposed_primary`, not `backup_ch`; there is no explicit backup-promotion operation.
6. **TRUE — “The failure path selects the best currently eligible candidate.”** `proposed_primary` is the first candidate after descending-score/ascending-ID sorting and is assigned on failure.
7. **TRUE — “The stored backup can be selected only if it is the best eligible candidate.”** The old backup receives no role preference; it is selected only when it happens to rank first under the same candidate ordering.
8. **TRUE — “The backup is recalculated after a primary change.”** Lines 770–780 select the best remaining eligible candidate after every primary decision.
9. **FALSE — “Stale measurements trigger emergency backup promotion.”** Staleness causes an early return before the failure or election logic.
10. **TRUE — “Stale measurements stop or prevent the election instead.”** A missing or older-than-5.0-s required stream makes `metrics_ready` false and the callback returns.
11. **TRUE — “Normal controlled primary switching is fully implemented.”** The executed path uses the 0.12 score margin, three wins, and 10 s hold and assigns/publishes a new primary when all pass. This verdict means implemented and reachable, not observed or validated.
12. **FALSE — “Normal controlled primary switching was triggered in the recorded trials.”** Every corrected trial has zero canonical primary transitions and no `better_candidate_stable` event.
13. **FALSE — “Emergency/failure-triggered primary replacement was experimentally validated.”** Active-primary GCS SNR remained above the trigger, no `primary_link_failure` event exists, and no dedicated completed failover experiment was found.

## Proposed Report Wording Review

### Proposed wording A — Primary-Link Failure Handling

**Technically correct with minor wording changes.** The revision below states the strict threshold comparison, names the streams covered by “stale,” and separates absence of a dedicated test from absence of a trigger in the final trials.

> A separate path is used when the current primary's GCS-link SNR falls strictly below the configured failure threshold of -2 dB.
>
> When this condition is reached during a metrics-ready election, the normal switching margin, minimum holding period, and consecutive-win requirements are bypassed. The clustering node selects the highest-scoring currently eligible candidate as the new primary.
>
> This differs from a strict backup-promotion mechanism. The stored backup is not given explicit preference and is not guaranteed to be promoted; it becomes primary only if it is the highest-ranked eligible candidate at that time. Another eligible UAV may therefore be selected instead. After the primary decision, the backup role is recalculated from the remaining eligible candidates. If no eligible replacement exists, both primary and backup are set to zero and the published state reports no gateway.
>
> The clustering implementation also requires recent global SNR, obstacle-loss, and position updates, together with complete required SNR and obstacle-link data, before an election proceeds. If these measurements are missing or stale, the callback returns without performing an election or promoting the stored backup.
>
> This failure-handling path is implemented, but it was not triggered during the three corrected dynamic trials, and no dedicated completed failure experiment was found. It is therefore described as implemented behaviour rather than experimentally validated failover.

### Proposed wording B — Overall Dynamic Clustering Behaviour

**Technically correct with minor wording changes.** The revision avoids saying scores necessarily change at every epoch, includes obstacle data, and uses the precise trigger/validation distinction.

> The implemented clustering process combines candidate scoring, initial primary and backup selection, periodic reevaluation, and controlled primary switching. Once the required measurements are ready, eligible UAVs are ranked and the highest-scoring candidate becomes the primary cluster head. The highest-ranked remaining eligible UAV becomes the backup, or the backup is set to zero if none remains.
>
> During subsequent metrics-ready election epochs, candidate scores are recomputed from the latest SNR, obstacle, and mobility inputs. A different highest-ranked candidate does not cause an immediate primary change: it must exceed the incumbent by the configured 0.12 score margin, the incumbent must have been held for at least 10 s, and the same challenger must satisfy those conditions for three qualifying election epochs. The backup is recalculated each ready epoch and can change while the primary remains stable.
>
> If the current primary's GCS SNR falls strictly below the configured -2 dB failure threshold, the normal margin, holding-period, and consecutive-win restrictions are bypassed. The highest-ranked currently eligible candidate is selected; the implementation does not guarantee direct promotion of the stored backup.
>
> The three corrected dynamic trials showed stable primary selection and canonical backup-role changes of 2, 4, and 3. Neither controlled primary handover nor failure-triggered primary replacement occurred in those trials, and no dedicated completed failure experiment was found. Those paths are implemented but are not experimentally validated by the stored results.

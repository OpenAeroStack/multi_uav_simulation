# Clustering Source Audit

All CSV `time_s` values are seconds elapsed from the first ROS-bag message, as defined by the contemporaneous extractor at `scripts/extract_dynamic_bag.py` commit `09cff0a`, lines 163–179. They are not wall-clock timestamps.

| Trial | First clustering timestamp | Cutoff timestamp | Baseline assignment | Excluded/retained | Duplicate timestamps | Complete assignments |
|---|---:|---:|---:|---:|---|---|
| dynamic_trial1 | 0.001640 | 0.011640 | 0.899224 (row 12) | 10/35 | no | 45/45 |
| dynamic_trial2 | 0.007503 | 0.017503 | 0.661212 (row 12) | 10/35 | no | 45/45 |
| dynamic_trial3 | 0.000000 | 0.010000 | 0.164073 (row 12) | 10/35 | no | 45/45 |

## Schemas

- `cluster_assignments.csv`: `time_s` plus JSON `assignment`; JSON contains `primary_ch`, `backup_ch`, `epoch`, `status`, `num_uavs`, and a complete three-UAV assignment list with roles/parents/routes/scores/GCS SNR.
- `cluster_roles.csv`: `time_s`, `role`, `uav_id`; separate primary and backup topic publications, used only as corroboration.
- `cluster_events.csv`: `time_s` plus JSON `event`; payload contains epoch, reason, old/new primary and old/new backup. `reason` is the event type; there are no separate source/destination role fields.
- `summary.csv`: bag duration, role/score sample counts, event messages, legacy sampled-topic change counts, initial/final roles and primary-time percentages.

Assignment publication repeats complete states at the election rate. Consecutive repeated role states are expected publications, not duplicate CSV timestamps. Full messages can differ in scores even when the role state is unchanged. Event and assignment timestamps differ slightly because they are separate ROS messages.

## Reconciliation of legacy, event, and canonical counts

- Trial 1: legacy `backup_changes=8` consists of six pre-cutoff sampled state changes plus two canonical changes. Three steady events exist, but the 0.899 s event coincides with the first retained assignment and establishes the baseline, leaving two canonical transitions.
- Trial 2: legacy `backup_changes=7` consists of three pre-cutoff sampled state changes plus four canonical changes. Eight initialization events were published faster than the extracted assignment/role-state sampling represented them; four steady events match the four canonical transitions.
- Trial 3: legacy `backup_changes=3` equals the three canonical changes. Nine initialization events occurred before the retained baseline but were not represented as sampled role changes; three steady events match the canonical transitions.

All event payloads are unique within their trial because their epochs and/or old/new states differ. Thus the discrepancy is not byte-identical event duplication; it is initialization exclusion, baseline treatment, and different topic publication/sampling timing.

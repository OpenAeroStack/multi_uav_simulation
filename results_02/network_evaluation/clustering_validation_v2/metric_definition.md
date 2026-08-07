# Canonical Cluster-Role Transition Metric

`canonical_primary_transitions` is the number of primary identifier changes between consecutive complete, chronologically ordered cluster-assignment states after initialization. `canonical_backup_transitions` is defined identically for the backup identifier. The first retained state establishes the baseline and is not a transition.

The deterministic initialization interval is the first 0.01 s relative to the earliest recorded clustering timestamp in each bag-derived dataset. The first complete assignment at or after that cutoff is the baseline. Exact consecutive `(primary_ch, backup_ch)` duplicates are collapsed; repeated publications, empty-to-valid initialization, and raw event messages are not counted. Trials are never concatenated.

The sampled `/cluster/assignment` state is authoritative. `/cluster/event` is supporting evidence because events can occur during initialization, before the retained baseline, or at a different sampling timestamp.

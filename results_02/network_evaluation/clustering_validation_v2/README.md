# Clustering Validation v2

Legacy `backup_changes` counted changes on separately published backup-role samples, while event totals included initialization and a baseline event; neither is the canonical report metric. This package defines transitions from complete `/cluster/assignment` states after a uniform 0.01 s clustering-relative initialization interval. The first retained state is the baseline, repeated role states collapse, and identifier changes are counted once.

The analysis uses only corrected `dynamic_trial1`, `dynamic_trial2`, and `dynamic_trial3`; `dynamic_trial1_original` is excluded from the main comparison. Rerun from the repository root with `python3 results_02/scripts/analyze_clustering_validation_v2.py`.

Use `canonical_trial_summary.csv/.md` and `figures/cluster_roles_timeline.*` in the final report. Use `transition_audit.csv` for traceability. `implementation_status.*` separates static implementation evidence (the contemporaneous committed source snapshot) from observed and experimentally validated behavior.

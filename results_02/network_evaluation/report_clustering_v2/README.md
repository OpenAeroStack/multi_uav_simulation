# Report Clustering v2

This directory is the report-ready package for the dynamic link and clustering subsection of Chapter 5. It uses only corrected `dynamic_trial1`, `dynamic_trial2` and `dynamic_trial3`. The older `dynamic_trial1_original` dataset is not part of the comparison.

Canonical primary and backup transitions come directly from `clustering_validation_v2/canonical_trial_summary.csv` and its transition audit. The old `backup_changes` field and raw event-message totals are not used as transition counts. The existing canonical role timeline was copied without modification.

`link_quality_summary.csv` contains trial-level descriptive statistics calculated from each corrected `network_links.csv`. Standard deviations are sample standard deviations. Link rows are repeated measurements within a trial and are not treated as independent experimental runs.

The representative RSSI figure uses only GCS-to-UAV rows (`source=0`, `destination=1,2,3`) from corrected Trial 1; reverse and UAV-to-UAV links are not averaged into it. For the topology figure, the first and last assignment states retained by the canonical 0.01 s rule were selected. Each was paired with the nearest complete position timestamp. No communication edges were inferred.

Rebuild the derived CSVs and figures from the repository root with:

```bash
MPLCONFIGDIR=/tmp/matplotlib-clustering-report \
  python3 results_02/scripts/build_report_clustering_v2.py
```


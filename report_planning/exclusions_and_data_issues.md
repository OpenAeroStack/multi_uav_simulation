# Exclusions and Data Issues

| Issue | Affected files | Effect on report | Required handling | Status |
|---|---|---|---|---|
| RNG3 Run IDs invert actual modes | primary registry/provenance and linked RNG3 files | Filename-only grouping would swap modes | Use verified `actual_mode`; describe as provenance, not a result | Documented |
| Earlier final-comparison selections | non-primary final-comparison directories and primary `exclusions.csv` | Mixing protocols/runs invalidates final pairing | Use `_primary` only; retain provenance | Excluded |
| D4 duplicate frame ID | Original `ground_truth.csv` repeats `frame_0033.png`; original details has 61 rows/60 unique IDs | Original metrics weighted one frame twice | Use report-authoritative `d4_unique_frames_v2/`; retain original files as superseded provenance | Resolved |
| B2/B3 detailed run-level logs not separately retained | aggregate values in `results/MANIFEST.md` | RTT/load results remain descriptive rather than directly reprocessable | Cite once as manifest-supported descriptive results; do not describe them as unsupported | Documented |
| Retained B1 raw snapshot ends early | `ns3_snr_obstaclefree_20260804_220047.csv` ends at 740.2914 s, before four configured windows | Per-distance observed means cannot be reproduced from the saved snapshot | Use the original saved figures and manifest aggregate; leave unavailable per-distance observations blank in the v2 summary | Documented |
| Two B4 raw log labels contain stale distance text | raw labels `d01_69_9m` and `d01_63_09m` versus processed labels `d01_69_5m` and `d01_83_09m` | Filename-only matching would attach the wrong stated distance | Match by unique TAP sent/received counters; preserve both labels and use processed distances in the report | Resolved in v2 summary |
| Phase-C invalid, contaminated and incomplete attempts | header-only C2 `002437`/`003907`; contaminated C2 `004614`; incomplete C3 `011330` | Cannot be used in the four-condition descriptive comparison | Exclude; use verified selected files `012216`, `001716`, `012804` and `011400` | Selection resolved and registered in `selected_runs.csv` |
| Duplicate q10-clean label | phase-D processed CSVs | Ambiguous citation by label | Cite timestamp from delivery summary | Documented |
| Original dynamic Trial 1 differs | `dynamic_trial1_original/` | 119.835 s/70 samples not comparable to corrected ~70 s/45 | Keep separate/appendix | Appendix only |
| Startup cluster-event bursts | corrected T2/T3 events | Inflated switching impression | Canonical v2 excludes first 0.01 s relative to first clustering timestamp | Resolved |
| Event messages and legacy backup changes differ from canonical states | old summaries/events | Ambiguous transition counts | Use canonical assignment-state counts 2/4/3; retain event/legacy values as provenance | Resolved |
| Controlled primary handover untriggered | corrected trials and contemporaneous manager source | Cannot claim experimental handover validation | State that the score-margin/consecutive-win/holding-time path existed but zero canonical primary transitions occurred | Documented in drafted subsection |
| Emergency failover partial and untested | manager failure path and stored trials | Cannot claim guaranteed backup promotion, failover validation or recovery time | Describe threshold-driven best-candidate replacement accurately; immediate stored-backup promotion was not guaranteed | Documented in drafted subsection |
| Incomplete NetAnim XML | both `_incomplete.xml` | Cannot be used | Exclude | Excluded |
| Malformed final NetAnim updates | both final XMLs | Last update is semantically invalid | Visualise only before final update; do not edit source | Appendix only |
| No NetAnim packet/link events | all NetAnim XMLs | Cannot support performance claims | Mobility visualisation only | Documented |
| Manual LOS different format | `los_trial1/logs/` | Not directly poolable with extracted CSV trials | Appendix or derive a traceable CSV later | Requires human decision |
| Bridge forwarding log repeats ports | `bridge_forwarding_status.txt` | Raw presentation is misleading | Prefer clean bridge/TAP log | Documented |
| CPU/RSS are detector + relay host demand | selected Phase-E logs | Not power, energy, battery, or whole-system use | Label scope exactly; logger is not monitored | Resolved |
| Yaw externally commanded | primary provenance | No independent per-run yaw measurement | State configuration limitation | Documented |

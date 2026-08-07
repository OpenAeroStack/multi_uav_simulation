# Phase C — Middleware and Telemetry

## Purpose
Assess telemetry continuity under mission/no-vision, static baseline, Edge and Ground load.

## Source directories
`results/phase_c_middleware/` and `results/scripts/telemetry_health.py`.

## Experiment design
Nominal 120 s recordings summarised by message count, rate and inter-message gaps.

## Independent experimental unit
One complete condition recording; messages are within-run observations.

## Available runs

| Condition | Raw filename | Matching summary row | Status |
|---|---|---|---|
| C0 mission/no vision | `telemetry_c0_mission_baseline_20260805_012216.csv` | 274 messages, 2.28 Hz, 2243.2 ms max | Included |
| C1 static/no vision | `telemetry_c1_baseline_20260805_001716.csv` | 253, 2.11 Hz, 1994.1 ms | Included |
| C2 Edge | `telemetry_c2_edge_load_20260805_012804.csv` | 247, 2.06 Hz, 2354.9 ms | Included |
| C3 Ground | `telemetry_c3_ground_load_20260805_011400.csv` | 184, 1.53 Hz, 3788.4 ms | Included |
| C2 early attempts | timestamps `002437`, `003907` | no rows | Excluded: header-only |
| C2 contaminated | timestamp `004614` | 184, 1.53 Hz, 7766.9 ms | Excluded per manifest: concurrent logger |
| C3 incomplete | timestamp `011330` | 7, 0.06 Hz, 3989.2 ms | Excluded: incomplete |

## Main files
Eight raw CSVs and `processed/c_telemetry_summary.csv`.

## Main measured metrics
Message count, mean rate, and mean/median/p95/p99/maximum gap.

## Verified numerical results
Selected values are listed above; counts uniquely connect each non-empty raw file to its summary row.

## Results suitable for the main report
One four-condition table with messages, rate and maximum gap. The subsection is drafted at `report_export/sections/chapter5/middleware_telemetry_validation.tex`.

## Results better suited to an appendix
Full gap statistics and excluded-run mapping.

## Known issues and limitations
One selected run per condition prevents inferential claims. Ground degradation is associated with the condition but causality is not independently replicated.

## Recommended Chapter 5 destination
Section 5.

## Processing still required
The selected mapping, presentation table, optional figure and report subsection are complete. Only final report-integration consistency checks remain.

## Suggested tables
Selected C0–C3 message count/rate/max-gap table.

## Suggested figures
Optional `telemetry_rate_and_gap` plot under `report_export/images/chapter5/middleware/`; the LaTeX remains easy to use as a table-only section if page space is limited.

## Claims supported by the evidence
Telemetry remained active in all selected conditions; selected Ground has the lowest rate and largest maximum gap.

## Claims not supported by the evidence
A statistically general causal effect across independent replications.

## Report-ready package
`results/phase_c_middleware/report_validation_v2/` records the exact four selected filenames, recalculated timestamp statistics, final table, provenance and figure. One complete run represents each condition, so all comparisons remain descriptive.

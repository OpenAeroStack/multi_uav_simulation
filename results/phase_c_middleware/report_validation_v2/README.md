# Phase C Middleware Validation v2

This package contains the report-ready telemetry comparison for four selected operating conditions. No experiment was rerun and the original raw and processed Phase C files were not changed.

The selected files are identified in `selected_runs.csv`. Selection follows the Phase C planning record: C0 `012216`, C1 `001716`, C2 `012804` and C3 `011400`. Header-only C2 attempts, contaminated C2 `004614`, and incomplete C3 `011330` are excluded.

## Metric verification

`telemetry_summary.csv` was recalculated from the raw arrival timestamps. Message count is the number of data rows. Inter-message statistics use differences between consecutive `arrival_time_s` values. The stored mean rate uses the original configured 120 s capture duration, matching `processed/c_telemetry_summary.csv`; `observed_span_s` separately records the time between the first and last received messages.

The recalculated interval values were rounded to one decimal place for comparison with the existing summary. They reproduce the selected summary rows: mean intervals of 439.2, 473.7, 465.0 and 643.5 ms, and maximum gaps of 2243.2, 1994.1, 2354.9 and 3788.4 ms. Small differences beyond one decimal place only reflect the original summary's rounding.

Rebuild the derived CSVs and figure from the repository root with:

```bash
MPLCONFIGDIR=/tmp/matplotlib-telemetry-v2 \
  python3 results/scripts/build_telemetry_validation_v2.py
```


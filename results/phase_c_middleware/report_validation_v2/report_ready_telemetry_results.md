# Middleware and Telemetry Validation

## Selected Conditions

Four representative conditions were selected: mission operation without vision processing, a static no-vision baseline, Edge processing and Ground processing. One complete run was retained for each condition. Header-only C2 attempts, the contaminated C2 `004614` run and the incomplete C3 `011330` run were left out according to the existing Phase C selection record.

## Telemetry Activity

| Condition | Messages | Mean rate (Hz) | Maximum gap (ms) |
|---|---:|---:|---:|
| C0 mission/no vision | 274 | 2.28 | 2243.2 |
| C1 static/no vision | 253 | 2.11 | 1994.1 |
| C2 Edge | 247 | 2.06 | 2354.9 |
| C3 Ground | 184 | 1.53 | 3788.4 |

Telemetry remained active in all four selected conditions. The values were verified directly from the corresponding timestamped raw files and matched the retained rows in the Phase C combined summary.

## Descriptive Comparison

C0 produced 2.28 Hz, C1 produced 2.11 Hz and Edge produced 2.06 Hz. These three rates remained fairly close. Ground produced 1.53 Hz and also had the largest maximum message gap, at 3788.4 ms.

The Ground condition showed the largest degradation in telemetry continuity among the selected runs. Because one run was retained for each condition, this is a descriptive result rather than a statistical comparison.

## Relationship to Application Traffic

The result is consistent with additional contention or processing load during Ground operation, but the experiment does not isolate a single cause. In particular, it does not prove that Ground image transfer alone produced the difference.

## Claims Supported

- Telemetry remained active in all selected conditions.
- The selected Ground run had the lowest mean message rate.
- The selected Ground run had the largest maximum message gap.
- Edge telemetry remained closer to the no-vision conditions than Ground telemetry.

## Claims Not Supported

- Telemetry completely failed during Ground processing.
- Ground image transfer was the only cause of the observed difference.
- The condition differences were statistically significant.
- Individual messages were independent experimental trials.
- One run per condition establishes behaviour across all workloads or environments.

## Main-Report Content

Use the four-condition table and concise descriptive interpretation. The rate-and-gap figure can be included when page space permits, but the table contains the complete main result.

## Appendix-Only Content

The first and last timestamps, observed spans, minimum and mean intervals, excluded-run mapping, and raw arrival rows are supporting details suitable for an appendix or evidence archive.


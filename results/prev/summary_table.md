# Validation Summary

| Metric | Real Flight (Primary, 0-30m) | Simulation (0-28m) |
|---|---:|---:|
| Distance range tested | 0-30 m | 0-28 m |
| Max dropout observed | 0.00% | Excluded — packet counts are dominated by low-level 802.11 traffic |
| Min SNR observed | Not measured | 37.59 dB |
| SNR margin above 12dB threshold at max tested distance | Not measured | 25.59 dB (37.59 dB − 12 dB) |
| One-line conclusion | No application-message dropout was observed over the replicated flight range. | Simulated SNR remained comfortably above the reference threshold over the same path. |

## Data source

Both primary datasets derive from the same real-world flight, `2026-08-31 18-03-14.tlog`. The real result comes from direct telemetry-log analysis. The simulated result comes from replaying that flight's exact GPS path through Gazebo, ArduPilot SITL, and ns-3.

## Supplementary data

`real_dropout_vs_distance_SUPPLEMENTARY_different_flight_0-110m.csv` is from a different flight and day. It is included only as supplementary context showing that the real-world link also stayed clean over a longer range; it is not part of the primary validation claim. Its maximum calculated dropout was 2.68% (the non-zero maximum occurs in the sparsely sampled 110 m bin).

## Limitations

- Simulated SNR and signal values assume 5180 MHz WiFi PHY parameters that were not independently verified against the real drone's actual radio hardware.
- Simulated dropout-rate figures were dominated by low-level 802.11 protocol traffic rather than application-layer telemetry and were therefore excluded from this comparison.

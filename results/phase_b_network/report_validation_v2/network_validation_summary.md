# Network Validation Summary

## Analytical Signal-Level Validation

B1 compared the received signal power reported by NS-3 with the Friis prediction at five confirmed positions from 58.5 m to 113.8 m. The completed analysis contained 150,393 packet samples from nodes 0 and 1. It gave an R² of 0.860, an RMSE of 0.77 dB and a mean bias of -0.77 dB. The close shape and nearly constant residual offset show that the simulated signal followed the expected distance-dependent trend reasonably well.

There is a storage limitation in the current checkout. Its B1 raw CSV stops at 740.2914 s, while four configured distance windows occur later. The original figures and aggregate statistics preserve the completed result, but the individual observed means cannot be safely rebuilt from this shortened copy. They are therefore left blank in `b1_summary.csv`.

## Emulation-Path Timing Characteristics

The Phase A control measurements showed a mean RTT of 0.032 ms on the management veth path. Moving through the NS-3/TAP path produced means of 158.280 ms when colocated, 160.700 ms at the operational position and 152.310 ms in the scheduling-control run. This was treated as an emulation-path timing characteristic; the tests did not isolate its exact internal source, and it was not wireless propagation delay.

| Distance (m) | Mean RTT (ms) | Minimum (ms) | Maximum (ms) | Loss (%) |
|---:|---:|---:|---:|---:|
| 58.5 | 159.8 | 129 | 197 | 0 |
| 69.5 | 153.3 | 119 | 185 | 0 |
| 83.9 | 159.4 | 130 | 194 | 0 |
| 97.7 | 159.9 | 135 | 204 | 0 |
| 113.8 | 158.1 | 127 | 212 | 0 |

The B2 means stayed within 153.3--159.9 ms, with no clear increase over the tested distance range. Against the Phase A controls, the small change in physical propagation time was hidden by the much larger NS-3/TAP timing floor. This result is limited to the tested distances and setup.

The B2 and B3 aggregate results were preserved in the project experiment manifest, although their detailed run-level logs were not retained separately.

## Throughput and Packet-Loss Behaviour

| Offered load (Mbps) | Received throughput (Mbps) | Packet loss (%) |
|---:|---:|---:|
| 2 | 1.69 | 11.0 |
| 5 | 2.85 | 40.0 |
| 8 | 3.02 | 59.6 |

Useful throughput increased as more traffic was offered, but it approached about 3 Mbps under this configuration. Beyond that point, extra traffic mostly appeared as packet loss. It is best viewed as a soft goodput ceiling for this test, not a general capacity figure for 802.11.

The B4 runs used a 2 Mbps offered load for 10 s at each position. TAP-counter loss ranged from 13.25% to 13.96%, while iperf reported 11--12%. Neither series showed a strong distance trend. Within this range, the chosen traffic load and emulation setup had a more visible effect than the modest distance increase.

## Claims Supported

- The simulated received signal power followed the expected distance-dependent Friis trend.
- RTT over the tested range was dominated by the measured emulation-path timing floor.
- Useful throughput approached approximately 3 Mbps under the selected setup.
- Offered loads above this level mainly increased packet loss.
- B4 loss did not show a strong distance trend over the tested range.

## Claims Not Supported

- The measured RTT represents only physical wireless propagation delay.
- 3 Mbps is the universal capacity of the Wi-Fi standard.
- The results represent every UAV distance or environment.
- B2 and B3 have complete independently archived run-level provenance.
- The tested results prove behaviour for large UAV swarms.


# Phase B — Network Characterisation

## Purpose
Validate analytical propagation behavior and characterise RTT, throughput, packet loss, and LOS links.

## Source directories
`results/phase_b_network/`, `results/scripts/`, and `results_02/network_evaluation/los_*`.

## Experiment design
Five-distance obstacle-free SNR session; five 10 s sub-ceiling B4 loss runs; manifest-described RTT and offered-load tests; one stationary LOS bag and one manual five-sample LOS test.

## Independent experimental unit
Each complete distance/load run. Packets and link rows are within-run samples.

## Available runs
One B1 session over five positions, five B4 runs, three manifest-described B3 loads, five B2 positions, and two differently formatted LOS experiments.

## Main files
SNR CSV, two B1 plots, `b4_loss_summary.csv`, five iperf logs, paired TAP logs, LOS bag metadata/database, manual LOS text, and processing scripts.

## Main measured metrics
Observed and predicted received signal power, residual, R²/RMSE/bias, RTT, throughput/goodput, TAP/application loss, distance, and obstacle loss.

## Verified numerical results
Manifest reports B1 R²=0.860, RMSE=0.77 dB, bias=-0.77 dB over 58.5–113.8 m. B2 mean RTT is 153.3–159.9 ms with 0% reported loss. B3 received throughput is 1.69/2.85/3.02 Mbps at 2/5/8 Mbps offered, with 11.0/40.0/59.6% loss. B4 TAP packet loss is 13.25–13.96%; iperf loss is 11–12%.

## Results suitable for the main report
B1 and B4 are directly traceable report results. B2 and B3 are manifest-supported descriptive results. The subsection, tables and figures are drafted in `report_export/sections/chapter5/network_validation.tex` and `results/phase_b_network/report_validation_v2/`.

## Results better suited to an appendix
Raw iperf/TAP logs and manual LOS samples.

## Known issues and limitations
B1 distances are positions within one session, not five independent stochastic replications. The retained B1 raw snapshot ends before four configured windows, so the original figures and aggregate are used without fabricating per-distance observations. B2/B3 detailed run-level logs were not retained separately. LOS formats differ.

## Recommended Chapter 5 destination
Section 3.

## Processing still required
The network subsection is drafted. Only a consistency check during final report integration remains; LOS should be normalised only as a new derived artifact if it is later selected.

## Suggested tables
B1 fit statistics; B2 RTT; B3 offered load/goodput/loss; B4 distance loss.

## Suggested figures
Use the five registered exports: the two existing B1 plots, `b2_rtt_vs_distance`, `b3_throughput_and_loss`, and `b4_loss_vs_distance`.

## Claims supported by the evidence
Friis-based distance decay agrees reasonably over the tested range; B4 sub-ceiling loss is nearly flat across its five positions.

## Claims not supported by the evidence
Universal channel capacity, independence of packet samples, or a causal attribution of all loss to Nakagami fading.

# Phase A — Measurement Validation

## Purpose
Validate timing apparatus, warm-up, serial dependence, and logger overhead.

## Source directories
`results/phase_a_apparatus/` and `results/scripts/`.

## Experiment design
One management-veth baseline and three NS-3-path timing conditions; one stationary Edge loopback logger run; offline Welch and ACF analyses.

## Independent experimental unit
A complete timing condition or logger run. Individual pings/frames are repeated observations, not replications.

## Available runs
Four current RTT conditions (100 attempts each), three older v1 conditions, and one metrics run (manifest n=86).

## Main files
`rtt_summary.csv`, matching `rtt_samples_*.csv`, loopback metrics CSV, two PNGs, and `rtt_measure.sh`, `welch_warmup.py`, `acf_plot.py`, `rtf_monitor.sh`.

## Main measured metrics
RTT mean/median/SD/percentiles/loss; processing overhead; warm-up trend; frame-lag ACF; real-time factor where produced.

## Verified numerical results
Current means: management 0.032 ms; NS-3 colocated 158.280 ms; operational 160.700 ms; `chrt` 152.310 ms; zero loss. Manifest: loopback overhead 3.7 ± 0.5 ms, n=86.

## Results suitable for the main report
Emulation-floor table and warm-up figure.

## Results better suited to an appendix
ACF and scheduling investigation details.

## Known issues and limitations
`rtt_summary_v1.csv` is superseded. Warm-up/ACF raw input is not clearly registered. The Wi-Fi PHY computational-cost explanation is a hypothesis.

## Recommended Chapter 5 destination
Sections 1–2.

## Processing still required
Confirm plot provenance, locate any real-time-factor output, and select warm-up exclusion.

## Suggested tables
Four-condition timing summary; logger overhead.

## Suggested figures
Existing Welch plot; ACF in appendix.

## Claims supported by the evidence
Management overhead is negligible relative to the measured NS-3/TAP path floor; `chrt` did not remove it.

## Claims not supported by the evidence
The exact internal cause of the floor or generalisation to other hardware.

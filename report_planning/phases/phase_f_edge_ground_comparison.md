# Phase F — Edge versus Ground Comparison

## Purpose
Compare live completion, latency, inference/codec timing, and detector-plus-relay host resources.

## Source directories
`results/final_edge_ground_comparison_primary/`, linked Phase-F raw logs, and linked Phase-E resource logs.

## Experiment design
Three RNG-matched sessions, one Edge and one Ground run each, fixed pose, 1 Hz relay, 60 s official window, confidence 0.25 and Ground JPEG Q5. Metadata-recorded official start/end rows exclude warm-up/stabilisation; official latency outliers remain.

## Independent experimental unit
One complete run. There are three independent runs per mode; frames are not independent repetitions.

## Available runs
Six verified primary runs listed in `selected_runs.csv`; RNG3 filenames are inverted but detector, relay, metrics and metadata agree on actual modes.

## Main files
Registry, provenance, exclusions/checksums, raw links, run/mode/paired/latency summaries, combined D4/Phase-F table, report drafts and 11 PNG/PDF figure pairs.

## Main measured metrics
Processed-frame ratio; median/p95 pipeline latency; inference, compression and decode time; detector-plus-relay CPU/RSS.

## Verified numerical results
Edge processed ratio 1.0000 ± 0.0000; Ground 0.7667 ± 0.0727. Mean of run medians: 182.4050 versus 507.8583 ms. Mean run p95: 421.2535 versus 562.9742 ms. Inference: 54.5237 versus 51.5667 ms. Ground compression/decode: 2.5076/2.1362 ms. CPU/RSS values are in Phase E.

## Results suitable for the main report
Final selected summary tables and four chosen figures: D4 accuracy, processed-frame ratio, median latency, and CPU utilisation.

## Results better suited to an appendix
Other paired/per-RNG plots, latency validation, exclusions, checksums and mode evidence.

## Known issues and limitations
Three repetitions per mode; no significance testing; RNG3 naming mismatch; externally commanded yaw; host resource scope; Edge and Ground completion ratios have mode-specific meanings.

## Recommended Chapter 5 destination
Section 6. **Already drafted; consistency check only.**

## Processing still required
Verify drafted values/figure references against primary processed files; do not rerun or replace selections.

## Suggested tables
Six-run registry and mode-level completion/latency/timing/resource table.

## Suggested figures
The four selected existing figures above.

## Claims supported by the evidence
Descriptive differences for these three controlled RNG pairs using official metadata windows.

## Claims not supported by the evidence
Population-level significance, pure radio propagation timing, energy/battery effects, or generalisation beyond this host/configuration.

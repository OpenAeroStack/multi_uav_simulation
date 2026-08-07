# Phase E — Resource Measurements

## Purpose
Compare CPU and resident memory for selected Edge/Ground runs.

## Source directories
`results/phase_e_resources/raw/`, registry links under the primary comparison, and the runner/analysis scripts.

## Experiment design
`pidstat` sampled two PIDs once per second during each official 60 s window.

## Independent experimental unit
One selected complete Phase-F run; three per mode.

## Available runs
Six authoritative registry-linked logs plus smoke, incomplete and superseded resource logs.

## Main files
Six `resources.txt` symlinks in the primary registry, their source logs, `run_comparison_once.sh`, and final run/mode summaries.

## Main measured metrics
Summed detector-plus-camera-relay CPU percentage and RSS per timestamp; run mean and peak; mode mean ± sample SD.

## Verified numerical results
Edge mean CPU 56.2167% (SD 2.5564), RSS 952.3228 MB (SD 5.1138). Ground mean CPU 42.4413% (SD 1.4700), RSS 963.3388 MB (SD 2.5758), based on three run-level values per mode.

## Results suitable for the main report
CPU/RSS rows in the final comparison table and selected CPU figure.

## Results better suited to an appendix
Raw pidstat traces and peak values.

## Known issues and limitations
Only detector and relay are monitored; metrics logger and other system processes are excluded. These are single-host process measurements, not power, battery, or physical-UAV energy.

## Recommended Chapter 5 destination
Section 6, already drafted, with Section-7 limitation.

## Processing still required
Consistency review only.

## Suggested tables
Run- and mode-level CPU/RSS.

## Suggested figures
Existing mean CPU figure; RSS can remain tabular or appendix.

## Claims supported by the evidence
Detector-plus-relay host CPU/RSS differed descriptively across the three selected pairs.

## Claims not supported by the evidence
Whole-system demand, energy use, battery life, or physical onboard hardware performance.

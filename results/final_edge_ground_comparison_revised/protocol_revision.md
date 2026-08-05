# Protocol revision

The initial six-run dataset exposed a startup DDS queue burst. In the initial RNG1 edge run, queued messages produced a descending latency staircase as several detections arrived in a burst. The runner was therefore revised before collecting the requested replacement runs.

The revised gate retained a minimum of nine processed warm-up rows, followed by 15 seconds of post-warm-up settling. It then required five consecutive rows with inter-arrival times in the 0.70–1.30 second range and recorded explicit official CSV start and end row boundaries. The intended design was repeated for all six requested primary run IDs under the same fixed experimental settings.

The initial curated dataset remains preserved in `../final_edge_ground_comparison_initial_v1/` for transparency and is not used by the revised analysis script. Official-window latency outliers are retained; the gate prevents known startup backlog from entering the window and is not an outlier-removal rule.

The provenance audit discovered conflicts in the saved revised artifacts. Four requested IDs contain the opposite recorded mode and one lacks the metadata required to establish its official boundaries. Accordingly, the analysis reports available run diagnostics but does not treat those records as verified primary mode repetitions.

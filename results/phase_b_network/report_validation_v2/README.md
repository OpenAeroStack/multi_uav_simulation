# Phase B Network Validation v2

This folder contains the report-ready network validation tables and figures used for Chapter 5. No experiment was rerun. The package was assembled from the stored Phase A and Phase B evidence and from the aggregate B2/B3 values recorded in `results/MANIFEST.md`.

## Contents

- `b1_summary.csv`: the five confirmed B1 distances and Friis predictions, with the retained raw-snapshot status.
- `phase_a_timing_floor_summary.csv`: the current authoritative emulation-path RTT summary.
- `b2_manifest_summary.csv` and `b3_manifest_summary.csv`: descriptive aggregate results transcribed from the experiment manifest.
- `b4_summary.csv`: processed TAP loss joined to the matching iperf receiver logs.
- `network_validation_summary.md`: a compact interpretation of the results.
- `report_values.md`: numerical cross-check for the LaTeX section.
- `figures/`: the two preserved B1 figures and newly prepared B2--B4 report plots.

`source_map.md` records the evidence level and limitations for each result. In particular, the B1 raw CSV saved in this checkout ends at 740.2914 s, before four of the five configured analysis windows. For that reason, the per-distance observations were not reconstructed from the shortened snapshot. The completed five-distance statistics and original figures remain the report evidence, while blank fields in `b1_summary.csv` avoid fabricating unavailable values.

The derived files can be rebuilt without running NS-3:

```bash
MPLCONFIGDIR=/tmp/matplotlib-network-v2 \
  python3 results/scripts/build_network_validation_v2.py
```


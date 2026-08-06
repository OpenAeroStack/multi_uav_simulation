# Definitive primary edge-versus-ground comparison

This directory contains the final curated six-run Phase F dataset: three independent RNG sessions with one Edge and one Ground run per session. Original artifacts remain in `results/phase_f_comparison/raw/` and `results/phase_e_resources/raw/`; `raw_links/` contains only relative symbolic links.

Regenerate all processed tables, figures, and the report with:

```bash
python3 results/scripts/analyze_final_edge_ground_primary.py
```

The script reads only `selected_runs.csv` for Phase F run selection and slices every metrics CSV using its metadata-recorded `official_start_csv_row` and `official_end_csv_row`. Startup, stabilization, and shutdown rows are excluded. Official-window latency outliers are retained.

RNG3 has inverted Run ID labels. The manifest records actual modes established independently from detector, relay, and metrics evidence; original filenames are not changed. See `provenance.md`.

Each complete run is an independent experimental unit. Cross-run summaries use the three run-level values per mode rather than treating frames as independent repetitions.

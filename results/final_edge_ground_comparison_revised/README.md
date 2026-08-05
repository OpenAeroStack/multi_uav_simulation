# Revised edge-versus-ground comparison

This directory curates the six requested revised run IDs without changing the original artifacts. Files under `raw_links/` are symbolic links to `phase_f_comparison/raw/` and `phase_e_resources/raw/`. `checksums.sha256` hashes the original files.

Provenance verification found that four requested run IDs record the opposite mode in their metadata, CSV names, and node logs. One further run lacks metadata and resource monitoring. These issues are retained visibly in `selected_runs.csv`, `provenance.md`, and the processed outputs; they are not silently corrected or guessed. Consequently, the requested selection does not contain three verified runs per mode and cannot support the intended primary paired comparison as saved.

Regenerate the outputs from only the selected registry with:

```bash
python3 results/scripts/analyze_final_edge_ground_revised.py
```

Each run is the independent experimental unit. Frame rows are used only to summarize a run's metadata-defined official window. Mode summaries and pairwise differences include only runs whose expected and observed provenance agree and whose official boundaries can be verified.

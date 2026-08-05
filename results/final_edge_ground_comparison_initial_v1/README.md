# Curated final edge-versus-ground comparison

This directory registers the six official runs: edge and ground for RNG runs 1, 2 and 3. Exact IDs, execution order and artifact paths are in `selected_runs.csv`.

The original immutable experiment artifacts remain in:

- `results/phase_f_comparison/raw/`
- `results/phase_e_resources/raw/`

`raw_links/` contains relative symbolic links so the curated view does not duplicate or alter raw evidence. Do not edit raw files through these links.

## Integrity

From the repository root, verify every selected original artifact with:

```bash
sha256sum -c results/final_edge_ground_comparison/checksums.sha256
```

## Regenerate analysis

```bash
python3 results/scripts/analyze_final_edge_ground.py
```

The script reads only the six entries in `selected_runs.csv`. It excludes the first nine processed rows, uses exactly the recorded official processed-frame count, and excludes shutdown rows. Processed outputs are written under `processed/`.

Non-selected diagnostic, smoke, retry and superseded artifacts are listed in `exclusions.csv`; none were deleted or modified.

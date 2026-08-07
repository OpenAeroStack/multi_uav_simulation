# D4 Unique-Frame Analysis v2

**Corrected report analysis based on 60 unique labelled frames.**

This versioned analysis corrects the original 61-row result while preserving every original raw image, label, processed output, figure and script. `frame_0033.png` was duplicated in ground truth, causing the original evaluator to process the same image twice. The rule is filename as the unique key: retain the first valid occurrence and exclude the later duplicate for both Edge/raw and Ground/JPEG Q5.

Sources are listed in `source_files.md`; the row-level evidence is in `deduplication_audit.md` and `.csv`. Rerun from the repository root with:

```bash
python3 results/scripts/recompute_d4_unique_frames.py
```

Use `accuracy_summary_unique.csv`, `accuracy_comparison_unique.md`, and the figures in `figures/` for the final report. Keep the original 61-row files as superseded provenance.

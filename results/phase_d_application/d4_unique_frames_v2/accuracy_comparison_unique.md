# Corrected D4 Accuracy Comparison

The original stored analysis evaluated 61 rows, including `frame_0033.png` twice. The corrected analysis retains the first valid occurrence and evaluates the same 60 unique frame IDs for both modes. The numerical changes below are corrections, not claims of statistical significance.

| Representation | Frames | TP | FP | FN | Precision | Recall | F1 | Exact frames | Exact-count rate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Edge/raw | 60 | 135 | 19 | 29 | 0.876623 | 0.823171 | 0.849057 | 34 | 0.566667 |
| Ground/JPEG Q5 | 60 | 29 | 0 | 135 | 1.000000 | 0.176829 | 0.300518 | 26 | 0.433333 |

`metric_changes_from_original.csv` provides the original-versus-corrected values.

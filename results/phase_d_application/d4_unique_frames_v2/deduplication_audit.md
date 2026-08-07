# D4 Deduplication Audit

`frame_0033.png` occurs once in the capture index, twice in ground truth, and twice in the combined details (therefore twice for both Edge/raw and Ground/Q5). The two label rows have the same count. Prediction counts and TP/FP/FN fields are identical; repeated timing measurements differ. This shows the duplicate entered through the duplicated ground-truth row, which the original evaluator processed twice.

| Source file | Duplicate frame ID | Occurrences | Rows identical? | Handling |
|---|---|---:|---|---|
| `results/phase_d_application/raw/d4_reference_01/frames.csv` | `frame_0033.png` | 1 | n/a | Retain indexed frame |
| `results/phase_d_application/raw/d4_reference_01/ground_truth.csv` | `frame_0033.png` | 2 | Yes: label/count/notes | Retain row 34; exclude row 35 |
| `results/phase_d_application/processed/d4_detection_details_d4_raw_vs_q5_01_20260805_221850.csv` Edge/raw fields | `frame_0033.png` | 2 | Counts yes; timing no | Retain row 34; exclude row 35 |
| `results/phase_d_application/processed/d4_detection_details_d4_raw_vs_q5_01_20260805_221850.csv` Ground/Q5 fields | `frame_0033.png` | 2 | Counts yes; timing/codec no | Retain row 34; exclude row 35 |

The same retained 60-frame set is used for both representations. See `deduplication_audit.csv` for original source row numbers and count fields.

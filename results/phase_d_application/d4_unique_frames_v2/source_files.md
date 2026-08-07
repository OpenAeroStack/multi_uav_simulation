# Source Files

| Source | Purpose |
|---|---|
| `results/phase_d_application/raw/d4_reference_01/frames.csv` | Capture/frame index |
| `results/phase_d_application/raw/d4_reference_01/ground_truth.csv` | Manual ground-truth labels |
| `results/phase_d_application/processed/d4_detection_details_d4_raw_vs_q5_01_20260805_221850.csv` | Stored Edge/raw and Ground/Q5 per-frame detections |
| `results/phase_d_application/processed/d4_detection_summary_d4_raw_vs_q5_01_20260805_221850.csv` | Original 61-row accuracy summary |
| `results/scripts/d4_detection_eval.py` | Original count-based evaluation method |

The original method uses confidence 0.25, COCO person class filtering, count-based matching (`TP=min(gt,pred)`, excess predictions as FP and missing predictions as FN), and exact predicted-count equality. It does not use bounding-box IoU matching.

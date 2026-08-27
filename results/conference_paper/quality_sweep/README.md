# JPEG Quality Sweep Dataset

Source images: `results/phase_d_application/raw/d4_reference_01/`

Source ground truth: `results/phase_d_application/raw/d4_reference_01/ground_truth.csv`

`input/ground_truth_clean.csv` was created because the historical source ground-truth file contains two identical rows for `frame_0033.png`. The duplicate remains preserved in the original Phase D file and is removed only from this conference-paper copy.

Dataset size: 60 unique frames.

## Completed Full JPEG Quality Sweep

The full experiment was run on 2026-08-27 UTC with run ID `conference_qs_full_01`. It evaluated 60 unique labelled frames using YOLOv8n with confidence threshold 0.25, `imgsz=960`, and the COCO person class only (`classes=[0]`). The tested representations were RAW and JPEG qualities 5, 10, 20, 30, 40, 50, 60, 70, 80, and 90.

Every JPEG representation was independently encoded directly from the original PNG image; images were not progressively recompressed between quality levels. Detection metrics are count-based, using predicted and labelled person counts to calculate TP, FP, and FN. They are not IoU or bounding-box matching metrics.

### Major observations

- RAW achieved F1 = 0.849057.
- Q5 achieved F1 = 0.300518 and recall = 0.176829.
- Q40 achieved F1 = 0.857143 and recall = 0.859756.
- Q40 had a mean payload of 89,069.916667 bytes per frame.
- Q40 had a mean compression ratio of 31.084653 relative to the 2,764,800-byte uncompressed 1280x720x3 image.

The small Q40 metric differences above RAW are treated as normal variation in detector output under image transformation and within this limited count-based dataset. They are not evidence that Q40 improves detection relative to RAW.

### Experiment decision

JPEG Q40 is selected as the candidate Ground-processing operating point for subsequent conference-paper experiments because it is the lowest tested quality that recovered approximately RAW-level F1 while retaining a large payload reduction. This operating point may be revisited if later experiments reveal a methodological reason to do so.

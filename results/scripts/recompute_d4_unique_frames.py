#!/usr/bin/env python3
"""Recompute D4 count-based accuracy using one row per unique labelled frame."""

from __future__ import annotations

import csv
import math
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]
PHASE = ROOT / "results/phase_d_application"
RAW = PHASE / "raw/d4_reference_01"
DETAILS = PHASE / "processed/d4_detection_details_d4_raw_vs_q5_01_20260805_221850.csv"
SUMMARY = PHASE / "processed/d4_detection_summary_d4_raw_vs_q5_01_20260805_221850.csv"
FRAMES = RAW / "frames.csv"
GROUND_TRUTH = RAW / "ground_truth.csv"
OUT = PHASE / "d4_unique_frames_v2"
FIGURES = OUT / "figures"


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_rows(path: Path, fields: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def safe_divide(numerator: float, denominator: float) -> float:
    return numerator / denominator if denominator else 0.0


def aggregate(rows: list[dict[str, str]], prefix: str) -> dict[str, float | int]:
    tp = sum(int(row[f"{prefix}_tp"]) for row in rows)
    fp = sum(int(row[f"{prefix}_fp"]) for row in rows)
    fn = sum(int(row[f"{prefix}_fn"]) for row in rows)
    precision = safe_divide(tp, tp + fp)
    recall = safe_divide(tp, tp + fn)
    f1 = safe_divide(2 * precision * recall, precision + recall)
    exact = sum(int(row[f"{prefix}_pred_count"]) == int(row["gt_count"]) for row in rows)
    return {
        "evaluation_frames": len(rows),
        "unique_frames": len({row["filename"] for row in rows}),
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "exact_count_frames": exact,
        "exact_count_rate": safe_divide(exact, len(rows)),
    }


def fmt(value: object, digits: int = 6) -> str:
    return f"{value:.{digits}f}" if isinstance(value, float) else str(value)


def main() -> int:
    for path in (FRAMES, GROUND_TRUTH, DETAILS, SUMMARY):
        if not path.is_file():
            raise FileNotFoundError(path)

    frames = read_rows(FRAMES)
    labels = read_rows(GROUND_TRUTH)
    details = read_rows(DETAILS)
    original_summary = {row["representation"]: row for row in read_rows(SUMMARY)}

    frame_counts = Counter(row["filename"] for row in frames)
    label_counts = Counter(row["filename"] for row in labels)
    detail_counts = Counter(row["filename"] for row in details)
    duplicates = sorted(name for name, count in label_counts.items() if count > 1)
    if duplicates != ["frame_0033.png"]:
        raise ValueError(f"Unexpected duplicate label keys: {duplicates}")

    for name in duplicates:
        matching_labels = [row for row in labels if row["filename"] == name]
        if len({(row["gt_count"], row.get("notes", "").strip()) for row in matching_labels}) != 1:
            raise ValueError(f"Conflicting duplicate labels for {name}")
        matching_details = [row for row in details if row["filename"] == name]
        count_fields = ("gt_count", "raw_pred_count", "raw_tp", "raw_fp", "raw_fn",
                        "q5_pred_count", "q5_tp", "q5_fp", "q5_fn", "original_png_bytes")
        if len({tuple(row[field] for field in count_fields) for row in matching_details}) != 1:
            raise ValueError(f"Conflicting duplicate prediction/count rows for {name}")

    if any(count != 1 for count in frame_counts.values()):
        raise ValueError("Capture index itself contains duplicate frame IDs")

    # Retain the first valid details occurrence in original order.
    seen: set[str] = set()
    unique_details: list[dict[str, str]] = []
    for row in details:
        if row["filename"] not in seen:
            seen.add(row["filename"])
            unique_details.append(row)

    ordered_frame_ids = [row["filename"] for row in frames]
    if [row["filename"] for row in unique_details] != ordered_frame_ids:
        raise ValueError("Unique details do not match the ordered capture index")
    if len(unique_details) != 60 or len(seen) != 60:
        raise ValueError("Corrected evaluation must contain exactly 60 unique frames")

    edge = aggregate(unique_details, "raw")
    ground = aggregate(unique_details, "q5")

    OUT.mkdir(parents=True, exist_ok=True)
    FIGURES.mkdir(parents=True, exist_ok=True)

    unique_index = []
    label_first = {}
    for row in labels:
        label_first.setdefault(row["filename"], row)
    detail_first = {row["filename"]: row for row in unique_details}
    for index, frame in enumerate(frames, start=1):
        name = frame["filename"]
        unique_index.append({
            "evaluation_order": index,
            "filename": name,
            "capture_time_s": frame["capture_time_s"],
            "width": frame["width"],
            "height": frame["height"],
            "encoding": frame["encoding"],
            "gt_count": label_first[name]["gt_count"],
            "original_detail_row": next(i for i, row in enumerate(details, start=2) if row["filename"] == name),
        })
    write_rows(OUT / "unique_frame_index.csv", list(unique_index[0]), unique_index)

    common = ["filename", "gt_count", "notes"]
    edge_fields = common + [field for field in details[0] if field.startswith("raw_")] + ["original_detail_row"]
    ground_fields = common + [field for field in details[0] if field.startswith("q5_")] + ["original_detail_row"]
    edge_rows, ground_rows = [], []
    for row in unique_details:
        original_row = next(i for i, item in enumerate(details, start=2) if item is row)
        edge_rows.append({field: (original_row if field == "original_detail_row" else row[field]) for field in edge_fields})
        ground_rows.append({field: (original_row if field == "original_detail_row" else row[field]) for field in ground_fields})
    write_rows(OUT / "edge_raw_details_unique.csv", edge_fields, edge_rows)
    write_rows(OUT / "ground_q5_details_unique.csv", ground_fields, ground_rows)

    summary_fields = ["representation", "evaluation_frames", "unique_frames", "confidence_threshold",
                      "jpeg_quality", "tp", "fp", "fn", "precision", "recall", "f1",
                      "exact_count_frames", "exact_count_rate"]
    summary_rows = []
    for representation, values, jpeg in (("edge_raw", edge, "n/a"), ("ground_q5", ground, "5")):
        summary_rows.append({
            "representation": representation,
            **{key: fmt(value) for key, value in values.items()},
            "confidence_threshold": "0.250000",
            "jpeg_quality": jpeg,
        })
    write_rows(OUT / "accuracy_summary_unique.csv", summary_fields, summary_rows)

    audit_fields = ["source_file", "original_row_number", "frame_id", "gt_count", "raw_pred_count",
                    "raw_tp", "raw_fp", "raw_fn", "q5_pred_count", "q5_tp", "q5_fp", "q5_fn", "handling"]
    audit_rows = []
    for path, source_rows in ((GROUND_TRUTH, labels), (DETAILS, details)):
        for row_number, row in enumerate(source_rows, start=2):
            if row["filename"] == "frame_0033.png":
                audit_rows.append({
                    "source_file": path.relative_to(ROOT), "original_row_number": row_number,
                    "frame_id": row["filename"], "gt_count": row["gt_count"],
                    "raw_pred_count": row.get("raw_pred_count", ""), "raw_tp": row.get("raw_tp", ""),
                    "raw_fp": row.get("raw_fp", ""), "raw_fn": row.get("raw_fn", ""),
                    "q5_pred_count": row.get("q5_pred_count", ""), "q5_tp": row.get("q5_tp", ""),
                    "q5_fp": row.get("q5_fp", ""), "q5_fn": row.get("q5_fn", ""),
                    "handling": "retained" if row_number == 34 else "excluded later duplicate",
                })
    write_rows(OUT / "deduplication_audit.csv", audit_fields, audit_rows)

    metric_rows = []
    metric_map = [("evaluation_frames", "frames"), ("precision", "precision"), ("recall", "recall"),
                  ("f1", "f1"), ("exact_count_rate", "exact_count_accuracy")]
    for metric, old_field in metric_map:
        old_edge = float(original_summary["edge_raw"][old_field])
        old_ground = float(original_summary["ground_q5"][old_field])
        new_edge = float(edge[metric])
        new_ground = float(ground[metric])
        metric_rows.append({
            "metric": metric, "original_edge_raw": fmt(old_edge), "corrected_edge_raw": fmt(new_edge),
            "edge_change": fmt(new_edge - old_edge), "original_ground_q5": fmt(old_ground),
            "corrected_ground_q5": fmt(new_ground), "ground_change": fmt(new_ground - old_ground),
        })
    write_rows(OUT / "metric_changes_from_original.csv", list(metric_rows[0]), metric_rows)

    audit_md = f"""# D4 Deduplication Audit

`frame_0033.png` occurs once in the capture index, twice in ground truth, and twice in the combined details (therefore twice for both Edge/raw and Ground/Q5). The two label rows have the same count. Prediction counts and TP/FP/FN fields are identical; repeated timing measurements differ. This shows the duplicate entered through the duplicated ground-truth row, which the original evaluator processed twice.

| Source file | Duplicate frame ID | Occurrences | Rows identical? | Handling |
|---|---|---:|---|---|
| `{FRAMES.relative_to(ROOT)}` | `frame_0033.png` | {frame_counts['frame_0033.png']} | n/a | Retain indexed frame |
| `{GROUND_TRUTH.relative_to(ROOT)}` | `frame_0033.png` | {label_counts['frame_0033.png']} | Yes: label/count/notes | Retain row 34; exclude row 35 |
| `{DETAILS.relative_to(ROOT)}` Edge/raw fields | `frame_0033.png` | {detail_counts['frame_0033.png']} | Counts yes; timing no | Retain row 34; exclude row 35 |
| `{DETAILS.relative_to(ROOT)}` Ground/Q5 fields | `frame_0033.png` | {detail_counts['frame_0033.png']} | Counts yes; timing/codec no | Retain row 34; exclude row 35 |

The same retained 60-frame set is used for both representations. See `deduplication_audit.csv` for original source row numbers and count fields.
"""
    (OUT / "deduplication_audit.md").write_text(audit_md, encoding="utf-8")

    source_md = f"""# Source Files

| Source | Purpose |
|---|---|
| `{FRAMES.relative_to(ROOT)}` | Capture/frame index |
| `{GROUND_TRUTH.relative_to(ROOT)}` | Manual ground-truth labels |
| `{DETAILS.relative_to(ROOT)}` | Stored Edge/raw and Ground/Q5 per-frame detections |
| `{SUMMARY.relative_to(ROOT)}` | Original 61-row accuracy summary |
| `results/scripts/d4_detection_eval.py` | Original count-based evaluation method |

The original method uses confidence 0.25, COCO person class filtering, count-based matching (`TP=min(gt,pred)`, excess predictions as FP and missing predictions as FN), and exact predicted-count equality. It does not use bounding-box IoU matching.
"""
    (OUT / "source_files.md").write_text(source_md, encoding="utf-8")

    comparison = f"""# Corrected D4 Accuracy Comparison

The original stored analysis evaluated 61 rows, including `frame_0033.png` twice. The corrected analysis retains the first valid occurrence and evaluates the same 60 unique frame IDs for both modes. The numerical changes below are corrections, not claims of statistical significance.

| Representation | Frames | TP | FP | FN | Precision | Recall | F1 | Exact frames | Exact-count rate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Edge/raw | 60 | {edge['tp']} | {edge['fp']} | {edge['fn']} | {edge['precision']:.6f} | {edge['recall']:.6f} | {edge['f1']:.6f} | {edge['exact_count_frames']} | {edge['exact_count_rate']:.6f} |
| Ground/JPEG Q5 | 60 | {ground['tp']} | {ground['fp']} | {ground['fn']} | {ground['precision']:.6f} | {ground['recall']:.6f} | {ground['f1']:.6f} | {ground['exact_count_frames']} | {ground['exact_count_rate']:.6f} |

`metric_changes_from_original.csv` provides the original-versus-corrected values.
"""
    (OUT / "accuracy_comparison_unique.md").write_text(comparison, encoding="utf-8")

    readme = f"""# D4 Unique-Frame Analysis v2

**Corrected report analysis based on 60 unique labelled frames.**

This versioned analysis corrects the original 61-row result while preserving every original raw image, label, processed output, figure and script. `frame_0033.png` was duplicated in ground truth, causing the original evaluator to process the same image twice. The rule is filename as the unique key: retain the first valid occurrence and exclude the later duplicate for both Edge/raw and Ground/JPEG Q5.

Sources are listed in `source_files.md`; the row-level evidence is in `deduplication_audit.md` and `.csv`. Rerun from the repository root with:

```bash
python3 results/scripts/recompute_d4_unique_frames.py
```

Use `accuracy_summary_unique.csv`, `accuracy_comparison_unique.md`, and the figures in `figures/` for the final report. Keep the original 61-row files as superseded provenance.
"""
    (OUT / "README.md").write_text(readme, encoding="utf-8")

    metric_names = ["Precision", "Recall", "F1-score", "Exact-count rate"]
    edge_values = [edge["precision"], edge["recall"], edge["f1"], edge["exact_count_rate"]]
    ground_values = [ground["precision"], ground["recall"], ground["f1"], ground["exact_count_rate"]]
    x = list(range(len(metric_names)))
    width = 0.36
    fig, ax = plt.subplots(figsize=(9, 5.5))
    bars1 = ax.bar([v - width / 2 for v in x], edge_values, width, label="Edge/raw", color="#2878B5", edgecolor="black")
    bars2 = ax.bar([v + width / 2 for v in x], ground_values, width, label="Ground/JPEG q5", color="#F28E2B", edgecolor="black")
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("Score")
    ax.set_xticks(x, metric_names)
    ax.set_title("Detection Accuracy on 60 Unique Labelled Frames")
    ax.legend()
    ax.grid(axis="y", alpha=0.25)
    for bars in (bars1, bars2):
        for bar in bars:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.015,
                    f"{bar.get_height():.3f}", ha="center", va="bottom", fontsize=9)
    fig.tight_layout()
    fig.savefig(FIGURES / "d4_detection_accuracy_comparison_unique.png", dpi=400)
    fig.savefig(FIGURES / "d4_detection_accuracy_comparison_unique.pdf")
    plt.close(fig)

    print("D4 unique-frame validation")
    print(f"  unique frames: {len(unique_details)}")
    print(f"  duplicate removed: frame_0033.png (later occurrence)")
    print(f"  Edge/raw: TP={edge['tp']} FP={edge['fp']} FN={edge['fn']} P={edge['precision']:.6f} R={edge['recall']:.6f} F1={edge['f1']:.6f} exact={edge['exact_count_rate']:.6f}")
    print(f"  Ground/Q5: TP={ground['tp']} FP={ground['fp']} FN={ground['fn']} P={ground['precision']:.6f} R={ground['recall']:.6f} F1={ground['f1']:.6f} exact={ground['exact_count_rate']:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

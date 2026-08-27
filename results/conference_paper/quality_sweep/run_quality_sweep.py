#!/usr/bin/env python3
"""Run the conference-paper offline JPEG quality sweep."""

from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_IMAGE_DIR = (
    REPO_ROOT / "results/phase_d_application/raw/d4_reference_01"
)
DEFAULT_GROUND_TRUTH = (
    Path(__file__).resolve().parent / "input/ground_truth_clean.csv"
)
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "processed"
DEFAULT_QUALITIES = [5, 10, 20, 30, 40, 50, 60, 70, 80, 90]
EXPECTED_FILENAMES = [f"frame_{index:04d}.png" for index in range(1, 61)]
HISTORICAL_REFERENCE = {
    "RAW": {
        "precision": 0.876623,
        "recall": 0.823171,
        "f1": 0.849057,
        "exact_count_accuracy": 0.566667,
    },
    "Q5": {
        "precision": 1.000000,
        "recall": 0.176829,
        "f1": 0.300518,
        "exact_count_accuracy": 0.433333,
    },
}

DETAIL_FIELDS = [
    "run_id",
    "filename",
    "representation",
    "jpeg_quality",
    "gt_count",
    "pred_count",
    "tp",
    "fp",
    "fn",
    "exact_count",
    "mean_confidence",
    "inference_ms",
    "encoded_size_bytes",
    "raw_image_bytes",
    "compression_ratio",
    "encode_ms",
    "decode_ms",
]

SUMMARY_FIELDS = [
    "representation",
    "jpeg_quality",
    "frames",
    "total_gt",
    "tp",
    "fp",
    "fn",
    "precision",
    "recall",
    "f1",
    "exact_count_accuracy",
    "mean_predicted_count",
    "mean_confidence",
    "mean_payload_bytes",
    "median_payload_bytes",
    "std_payload_bytes",
    "mean_compression_ratio",
    "mean_encode_ms",
    "mean_decode_ms",
    "mean_inference_ms",
    "median_inference_ms",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate RAW and independently encoded JPEG representations using "
            "the historical D4 count-based human-detection metrics."
        )
    )
    parser.add_argument("--image-dir", type=Path, default=DEFAULT_IMAGE_DIR)
    parser.add_argument("--ground-truth", type=Path, default=DEFAULT_GROUND_TRUTH)
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(os.path.expanduser("~/yolo_env/yolov8n.pt")),
    )
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument(
        "--qualities",
        type=int,
        nargs="+",
        default=DEFAULT_QUALITIES,
        metavar="Q",
    )
    parser.add_argument("--run-id", default="conference_quality_sweep_01")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args()


def safe_divide(numerator: float, denominator: float) -> float:
    return numerator / denominator if denominator > 0 else 0.0


def validate_options(args: argparse.Namespace) -> None:
    if not 0.0 <= args.confidence <= 1.0:
        raise ValueError("confidence must be between 0 and 1")
    if args.imgsz <= 0:
        raise ValueError("imgsz must be positive")
    if not args.qualities:
        raise ValueError("at least one JPEG quality is required")
    if any(quality < 0 or quality > 100 for quality in args.qualities):
        raise ValueError("JPEG qualities must be between 0 and 100")
    if len(args.qualities) != len(set(args.qualities)):
        raise ValueError("JPEG qualities must be unique")
    if not args.run_id.strip():
        raise ValueError("run-id must not be empty")


def load_and_validate_dataset(
    image_dir: Path,
    ground_truth_path: Path,
) -> list[dict[str, Any]]:
    image_paths = sorted(image_dir.glob("*.png"))
    image_filenames = [path.name for path in image_paths]

    if len(image_paths) != 60:
        raise ValueError(f"expected exactly 60 PNG images, found {len(image_paths)}")
    if image_filenames != EXPECTED_FILENAMES:
        raise ValueError(
            "PNG filenames must be exactly frame_0001.png through frame_0060.png"
        )
    if len(image_filenames) != len(set(image_filenames)):
        raise ValueError("PNG filenames are not unique")

    with ground_truth_path.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames != ["filename", "gt_count", "notes"]:
            raise ValueError(
                "ground-truth columns must be exactly filename, gt_count, notes"
            )
        source_rows = list(reader)

    if len(source_rows) != 60:
        raise ValueError(
            f"expected exactly 60 ground-truth rows, found {len(source_rows)}"
        )

    entries: list[dict[str, Any]] = []
    for row_number, row in enumerate(source_rows, start=2):
        filename = str(row["filename"]).strip()
        value = str(row["gt_count"]).strip()
        try:
            gt_count = int(value)
        except ValueError as exc:
            raise ValueError(
                f"invalid gt_count at ground-truth row {row_number}: {value!r}"
            ) from exc
        if str(gt_count) != value or gt_count < 0:
            raise ValueError(
                f"gt_count must be a non-negative integer at row {row_number}"
            )
        entries.append(
            {
                "filename": filename,
                "gt_count": gt_count,
                "notes": str(row.get("notes", "")),
            }
        )

    gt_filenames = [entry["filename"] for entry in entries]
    if len(gt_filenames) != len(set(gt_filenames)):
        raise ValueError("ground-truth filenames are not unique")
    if set(gt_filenames) != set(image_filenames):
        missing_images = sorted(set(gt_filenames) - set(image_filenames))
        missing_annotations = sorted(set(image_filenames) - set(gt_filenames))
        raise ValueError(
            "image/annotation mismatch; "
            f"missing images={missing_images}, missing annotations={missing_annotations}"
        )

    return sorted(entries, key=lambda entry: entry["filename"])


def run_detector(
    model: Any,
    image: np.ndarray,
    confidence: float,
    image_size: int,
) -> tuple[int, list[float], float]:
    start = time.perf_counter()
    results = model(
        image,
        imgsz=image_size,
        conf=confidence,
        classes=[0],
        verbose=False,
    )
    inference_ms = (time.perf_counter() - start) * 1000.0
    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return 0, [], inference_ms
    confidences = [float(value) for value in boxes.conf.detach().cpu().tolist()]
    return len(confidences), confidences, inference_ms


def count_metrics(gt_count: int, pred_count: int) -> tuple[int, int, int]:
    return (
        min(gt_count, pred_count),
        max(pred_count - gt_count, 0),
        max(gt_count - pred_count, 0),
    )


def evaluate_image(
    model: Any,
    image: np.ndarray,
    entry: dict[str, Any],
    run_id: str,
    confidence: float,
    image_size: int,
    quality: int | None,
) -> dict[str, Any]:
    height, width, channels = image.shape
    raw_image_bytes = width * height * channels

    if quality is None:
        inference_image = image
        encoded_size_bytes = raw_image_bytes
        encode_ms = 0.0
        decode_ms = 0.0
        representation = "RAW"
    else:
        encode_start = time.perf_counter()
        success, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        encode_ms = (time.perf_counter() - encode_start) * 1000.0
        if not success:
            raise RuntimeError(f"JPEG Q{quality} encoding failed for {entry['filename']}")
        encoded_size_bytes = int(encoded.size)

        decode_start = time.perf_counter()
        inference_image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        decode_ms = (time.perf_counter() - decode_start) * 1000.0
        if inference_image is None:
            raise RuntimeError(f"JPEG Q{quality} decoding failed for {entry['filename']}")
        representation = "JPEG"

    pred_count, confidences, inference_ms = run_detector(
        model, inference_image, confidence, image_size
    )
    tp, fp, fn = count_metrics(entry["gt_count"], pred_count)

    return {
        "run_id": run_id,
        "filename": entry["filename"],
        "representation": representation,
        "jpeg_quality": "" if quality is None else quality,
        "gt_count": entry["gt_count"],
        "pred_count": pred_count,
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "exact_count": int(pred_count == entry["gt_count"]),
        "mean_confidence": float(np.mean(confidences)) if confidences else 0.0,
        "inference_ms": inference_ms,
        "encoded_size_bytes": encoded_size_bytes,
        "raw_image_bytes": raw_image_bytes,
        "compression_ratio": raw_image_bytes / encoded_size_bytes,
        "encode_ms": encode_ms,
        "decode_ms": decode_ms,
        "_confidences": confidences,
    }


def aggregate(rows: list[dict[str, Any]], label: str) -> dict[str, Any]:
    tp = sum(row["tp"] for row in rows)
    fp = sum(row["fp"] for row in rows)
    fn = sum(row["fn"] for row in rows)
    precision = safe_divide(tp, tp + fp)
    recall = safe_divide(tp, tp + fn)
    f1 = safe_divide(2.0 * precision * recall, precision + recall)
    confidences = [value for row in rows for value in row["_confidences"]]
    payloads = [row["encoded_size_bytes"] for row in rows]
    quality = rows[0]["jpeg_quality"]

    return {
        "representation": label,
        "jpeg_quality": quality,
        "frames": len(rows),
        "total_gt": sum(row["gt_count"] for row in rows),
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "exact_count_accuracy": safe_divide(
            sum(row["exact_count"] for row in rows), len(rows)
        ),
        "mean_predicted_count": float(np.mean([row["pred_count"] for row in rows])),
        "mean_confidence": float(np.mean(confidences)) if confidences else 0.0,
        "mean_payload_bytes": float(np.mean(payloads)),
        "median_payload_bytes": float(np.median(payloads)),
        "std_payload_bytes": float(np.std(payloads)),
        "mean_compression_ratio": float(
            np.mean([row["compression_ratio"] for row in rows])
        ),
        "mean_encode_ms": float(np.mean([row["encode_ms"] for row in rows])),
        "mean_decode_ms": float(np.mean([row["decode_ms"] for row in rows])),
        "mean_inference_ms": float(np.mean([row["inference_ms"] for row in rows])),
        "median_inference_ms": float(
            np.median([row["inference_ms"] for row in rows])
        ),
    }


def formatted_row(row: dict[str, Any], fields: list[str]) -> dict[str, Any]:
    output = {field: row[field] for field in fields}
    for key, value in output.items():
        if isinstance(value, float):
            output[key] = f"{value:.6f}"
    return output


def write_csv(path: Path, fields: list[str], rows: list[dict[str, Any]]) -> None:
    with path.open("x", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(formatted_row(row, fields) for row in rows)


def git_commit_hash() -> str | None:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip() or None


def print_baseline_check(summary_by_label: dict[str, dict[str, Any]]) -> None:
    print("\nRAW AND Q5 REPRODUCIBILITY CHECK")
    print("=" * 72)
    for label in ("RAW", "Q5"):
        summary = summary_by_label.get(label)
        if summary is None:
            print(f"{label}: not evaluated")
            continue
        print(label)
        for metric, reference in HISTORICAL_REFERENCE[label].items():
            actual = float(summary[metric])
            print(
                f"  {metric:<22} new={actual:.6f}  "
                f"reference={reference:.6f}  difference={actual - reference:+.6f}"
            )
    print("=" * 72)


def main() -> int:
    args = parse_args()
    validate_options(args)
    entries = load_and_validate_dataset(args.image_dir, args.ground_truth)
    print("Dataset validation PASS: 60 unique images and 60 unique annotations.")

    try:
        import ultralytics
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "Ultralytics is required; run this script in the YOLO environment"
        ) from exc

    print(f"Loading model once: {args.model}")
    model = YOLO(str(args.model))

    warmup_path = args.image_dir / "frame_0001.png"
    warmup_image = cv2.imread(str(warmup_path), cv2.IMREAD_COLOR)
    if warmup_image is None or warmup_image.shape != (720, 1280, 3):
        raise RuntimeError(
            f"expected a decodable 1280x720 three-channel warm-up image: {warmup_path}"
        )
    print(
        "Warming up YOLO with 3 unrecorded inferences on frame_0001.png "
        f"(confidence={args.confidence}, imgsz={args.imgsz}, classes=[0])"
    )
    for _ in range(3):
        run_detector(model, warmup_image, args.confidence, args.imgsz)

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_UTC")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    details_path = args.output_dir / f"quality_sweep_details_{timestamp}.csv"
    summary_path = args.output_dir / f"quality_sweep_summary_{timestamp}.csv"
    metadata_path = args.output_dir / f"quality_sweep_metadata_{timestamp}.json"
    for path in (details_path, summary_path, metadata_path):
        if path.exists():
            raise FileExistsError(f"refusing to overwrite existing output: {path}")

    detail_rows: list[dict[str, Any]] = []
    total_evaluations = len(entries) * (1 + len(args.qualities))
    completed = 0

    for entry in entries:
        image_path = args.image_dir / entry["filename"]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None or image.shape != (720, 1280, 3):
            raise RuntimeError(
                f"expected a decodable 1280x720 three-channel image: {image_path}"
            )

        for quality in [None, *args.qualities]:
            row = evaluate_image(
                model,
                image,
                entry,
                args.run_id,
                args.confidence,
                args.imgsz,
                quality,
            )
            detail_rows.append(row)
            completed += 1
            label = "RAW" if quality is None else f"Q{quality}"
            print(
                f"[{completed:03d}/{total_evaluations:03d}] "
                f"{entry['filename']} {label}: GT={entry['gt_count']} "
                f"pred={row['pred_count']}"
            )

    summary_rows: list[dict[str, Any]] = []
    summary_by_label: dict[str, dict[str, Any]] = {}
    representation_specs = [("RAW", "")] + [
        (f"Q{quality}", quality) for quality in args.qualities
    ]
    for label, quality in representation_specs:
        selected = [row for row in detail_rows if row["jpeg_quality"] == quality]
        summary = aggregate(selected, label)
        summary_rows.append(summary)
        summary_by_label[label] = summary

    metadata = {
        "run_id": args.run_id,
        "timestamp_utc": timestamp,
        "image_directory": str(args.image_dir.resolve()),
        "ground_truth_path": str(args.ground_truth.resolve()),
        "model_path": str(args.model.expanduser().resolve()),
        "model_name": "YOLOv8n",
        "ultralytics_version": ultralytics.__version__,
        "opencv_version": cv2.__version__,
        "python_version": platform.python_version(),
        "confidence_threshold": args.confidence,
        "imgsz": args.imgsz,
        "class_filter": [0],
        "class_filter_description": "COCO person class only",
        "jpeg_qualities": args.qualities,
        "frame_count": len(entries),
        "warmup_inferences": 3,
        "warmup_frame": "frame_0001.png",
        "warmup_recorded_in_csv": False,
        "raw_timing_convention": "RAW encode_ms and decode_ms are 0.0",
        "evaluation_definition": (
            "Count-based, not bounding-box IoU: TP=min(gt_count,pred_count); "
            "FP=max(pred_count-gt_count,0); FN=max(gt_count-pred_count,0)."
        ),
        "git_commit_hash": git_commit_hash(),
        "details_file": details_path.name,
        "summary_file": summary_path.name,
    }

    write_csv(details_path, DETAIL_FIELDS, detail_rows)
    write_csv(summary_path, SUMMARY_FIELDS, summary_rows)
    with metadata_path.open("x", encoding="utf-8") as output_file:
        json.dump(metadata, output_file, indent=2)
        output_file.write("\n")

    print_baseline_check(summary_by_label)
    print(f"Details: {details_path}")
    print(f"Summary: {summary_path}")
    print(f"Metadata: {metadata_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

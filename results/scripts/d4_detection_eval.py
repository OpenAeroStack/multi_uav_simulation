#!/usr/bin/env python3
"""
D4 offline detection evaluation.

Compares human detection on:
  1. Original lossless PNG images — represents edge processing.
  2. JPEG quality 5 versions of the same images — represents viable ground mode.

Ground truth CSV format:
    filename,gt_count
    frame_0001.png,5
    frame_0002.png,4

An optional notes column is ignored.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from ultralytics import YOLO


def safe_divide(numerator: float, denominator: float) -> float:
    return numerator / denominator if denominator > 0 else 0.0


def run_detector(
    model: YOLO,
    image: np.ndarray,
    confidence: float,
    image_size: int,
) -> tuple[int, list[float], float]:
    """Run YOLO and return count, confidences and inference wall time."""
    start = time.perf_counter()

    results = model(
        image,
        imgsz=image_size,
        conf=confidence,
        classes=[0],          # COCO person class
        verbose=False,
    )

    inference_ms = (time.perf_counter() - start) * 1000.0

    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return 0, [], inference_ms

    confidences = [
        float(value)
        for value in boxes.conf.detach().cpu().tolist()
    ]

    return len(confidences), confidences, inference_ms


def count_metrics(
    ground_truth: int,
    predicted: int,
) -> tuple[int, int, int]:
    """
    Pragmatic count-based matching.

    This measures whether the predicted number of humans matches the manually
    labelled number. It does not perform bounding-box IoU matching.
    """
    true_positive = min(ground_truth, predicted)
    false_positive = max(predicted - ground_truth, 0)
    false_negative = max(ground_truth - predicted, 0)

    return true_positive, false_positive, false_negative


def load_ground_truth(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        raise FileNotFoundError(f"Ground-truth CSV not found: {path}")

    entries: list[dict[str, Any]] = []

    with path.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)

        required = {"filename", "gt_count"}
        actual = set(reader.fieldnames or [])

        missing = required - actual
        if missing:
            raise ValueError(
                f"Ground-truth CSV is missing columns: {sorted(missing)}"
            )

        for row_number, row in enumerate(reader, start=2):
            filename = str(row["filename"]).strip()

            if not filename:
                raise ValueError(
                    f"Empty filename in ground-truth row {row_number}"
                )

            try:
                ground_truth_count = int(row["gt_count"])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Invalid gt_count in row {row_number}: "
                    f"{row['gt_count']!r}"
                ) from exc

            if ground_truth_count < 0:
                raise ValueError(
                    f"Negative gt_count in row {row_number}"
                )

            entries.append(
                {
                    "filename": filename,
                    "gt_count": ground_truth_count,
                    "notes": str(row.get("notes", "")).strip(),
                }
            )

    if not entries:
        raise ValueError("Ground-truth CSV contains no labelled frames")

    return entries


def aggregate_mode(
    rows: list[dict[str, Any]],
    prefix: str,
) -> dict[str, float]:
    tp = sum(int(row[f"{prefix}_tp"]) for row in rows)
    fp = sum(int(row[f"{prefix}_fp"]) for row in rows)
    fn = sum(int(row[f"{prefix}_fn"]) for row in rows)

    precision = safe_divide(tp, tp + fp)
    recall = safe_divide(tp, tp + fn)
    f1 = safe_divide(
        2.0 * precision * recall,
        precision + recall,
    )

    all_confidences: list[float] = []
    for row in rows:
        all_confidences.extend(row[f"{prefix}_confidences"])

    exact_count_frames = sum(
        int(row[f"{prefix}_pred_count"]) == int(row["gt_count"])
        for row in rows
    )

    inference_values = [
        float(row[f"{prefix}_inference_ms"])
        for row in rows
    ]

    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "mean_confidence": (
            float(np.mean(all_confidences))
            if all_confidences else 0.0
        ),
        "exact_count_accuracy": safe_divide(
            exact_count_frames,
            len(rows),
        ),
        "mean_inference_ms": float(np.mean(inference_values)),
        "median_inference_ms": float(np.median(inference_values)),
        "p95_inference_ms": float(
            np.percentile(inference_values, 95)
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--images-dir",
        type=Path,
        default=Path(
            "results/phase_d_application/raw/d4_reference_01"
        ),
    )
    parser.add_argument(
        "--ground-truth",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(
            os.path.expanduser("~/yolo_env/yolov8n.pt")
        ),
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=5,
    )
    parser.add_argument(
        "--confidence",
        type=float,
        default=0.25,
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=960,
    )
    parser.add_argument(
        "--run-id",
        default="d4_raw_vs_q5_01",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(
            "results/phase_d_application/processed"
        ),
    )

    args = parser.parse_args()

    if not 0 <= args.jpeg_quality <= 100:
        raise ValueError("JPEG quality must be between 0 and 100")

    if not 0.0 <= args.confidence <= 1.0:
        raise ValueError("Confidence threshold must be between 0 and 1")

    ground_truth_path = (
        args.ground_truth
        if args.ground_truth is not None
        else args.images_dir / "ground_truth.csv"
    )

    entries = load_ground_truth(ground_truth_path)

    missing_images = [
        entry["filename"]
        for entry in entries
        if not (args.images_dir / entry["filename"]).exists()
    ]

    if missing_images:
        preview = "\n".join(missing_images[:10])
        raise FileNotFoundError(
            f"{len(missing_images)} labelled images are missing:\n{preview}"
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    detailed_path = (
        args.output_dir
        / f"d4_detection_details_{args.run_id}_{timestamp}.csv"
    )
    summary_path = (
        args.output_dir
        / f"d4_detection_summary_{args.run_id}_{timestamp}.csv"
    )

    print(f"Loading model: {args.model}")
    model = YOLO(str(args.model))

    # Warm up both representations using the first labelled frame.
    first_image = cv2.imread(
        str(args.images_dir / entries[0]["filename"]),
        cv2.IMREAD_COLOR,
    )

    if first_image is None:
        raise RuntimeError(
            f"Could not read first image: {entries[0]['filename']}"
        )

    success, first_encoded = cv2.imencode(
        ".jpg",
        first_image,
        [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
    )

    if not success:
        raise RuntimeError("JPEG warm-up encode failed")

    first_q5 = cv2.imdecode(
        first_encoded,
        cv2.IMREAD_COLOR,
    )

    print("Warming up YOLO...")
    run_detector(
        model,
        first_image,
        args.confidence,
        args.imgsz,
    )
    run_detector(
        model,
        first_q5,
        args.confidence,
        args.imgsz,
    )

    rows: list[dict[str, Any]] = []

    for index, entry in enumerate(entries, start=1):
        filename = entry["filename"]
        image_path = args.images_dir / filename

        image = cv2.imread(
            str(image_path),
            cv2.IMREAD_COLOR,
        )

        if image is None:
            raise RuntimeError(f"Could not decode image: {image_path}")

        encode_start = time.perf_counter()

        success, encoded = cv2.imencode(
            ".jpg",
            image,
            [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
        )

        encode_ms = (
            time.perf_counter() - encode_start
        ) * 1000.0

        if not success:
            raise RuntimeError(
                f"JPEG encoding failed for {filename}"
            )

        decode_start = time.perf_counter()

        compressed_image = cv2.imdecode(
            encoded,
            cv2.IMREAD_COLOR,
        )

        decode_ms = (
            time.perf_counter() - decode_start
        ) * 1000.0

        if compressed_image is None:
            raise RuntimeError(
                f"JPEG decoding failed for {filename}"
            )

        raw_count, raw_confidences, raw_inference_ms = run_detector(
            model,
            image,
            args.confidence,
            args.imgsz,
        )

        q5_count, q5_confidences, q5_inference_ms = run_detector(
            model,
            compressed_image,
            args.confidence,
            args.imgsz,
        )

        gt_count = int(entry["gt_count"])

        raw_tp, raw_fp, raw_fn = count_metrics(
            gt_count,
            raw_count,
        )

        q5_tp, q5_fp, q5_fn = count_metrics(
            gt_count,
            q5_count,
        )

        row = {
            "run_id": args.run_id,
            "filename": filename,
            "gt_count": gt_count,
            "notes": entry["notes"],
            "raw_pred_count": raw_count,
            "raw_tp": raw_tp,
            "raw_fp": raw_fp,
            "raw_fn": raw_fn,
            "raw_confidences": raw_confidences,
            "raw_mean_confidence": (
                float(np.mean(raw_confidences))
                if raw_confidences else 0.0
            ),
            "raw_inference_ms": raw_inference_ms,
            "q5_pred_count": q5_count,
            "q5_tp": q5_tp,
            "q5_fp": q5_fp,
            "q5_fn": q5_fn,
            "q5_confidences": q5_confidences,
            "q5_mean_confidence": (
                float(np.mean(q5_confidences))
                if q5_confidences else 0.0
            ),
            "q5_inference_ms": q5_inference_ms,
            "q5_size_bytes": int(encoded.size),
            "q5_encode_ms": encode_ms,
            "q5_decode_ms": decode_ms,
            "original_png_bytes": image_path.stat().st_size,
        }

        rows.append(row)

        print(
            f"[{index:03d}/{len(entries):03d}] {filename} | "
            f"GT={gt_count} raw={raw_count} q5={q5_count}"
        )

    raw_summary = aggregate_mode(rows, "raw")
    q5_summary = aggregate_mode(rows, "q5")

    detailed_fields = [
        "run_id",
        "filename",
        "gt_count",
        "notes",
        "raw_pred_count",
        "raw_tp",
        "raw_fp",
        "raw_fn",
        "raw_mean_confidence",
        "raw_inference_ms",
        "q5_pred_count",
        "q5_tp",
        "q5_fp",
        "q5_fn",
        "q5_mean_confidence",
        "q5_inference_ms",
        "q5_size_bytes",
        "q5_encode_ms",
        "q5_decode_ms",
        "original_png_bytes",
    ]

    with detailed_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as output_file:
        writer = csv.DictWriter(
            output_file,
            fieldnames=detailed_fields,
        )
        writer.writeheader()

        for row in rows:
            output_row = {
                field: row[field]
                for field in detailed_fields
            }

            for field in (
                "raw_mean_confidence",
                "raw_inference_ms",
                "q5_mean_confidence",
                "q5_inference_ms",
                "q5_encode_ms",
                "q5_decode_ms",
            ):
                output_row[field] = f"{float(output_row[field]):.4f}"

            writer.writerow(output_row)

    q5_sizes = [
        float(row["q5_size_bytes"])
        for row in rows
    ]

    encode_times = [
        float(row["q5_encode_ms"])
        for row in rows
    ]

    decode_times = [
        float(row["q5_decode_ms"])
        for row in rows
    ]

    summary_rows = [
        {
            "run_id": args.run_id,
            "representation": "edge_raw",
            "frames": len(rows),
            "confidence_threshold": args.confidence,
            "jpeg_quality": "n/a",
            **raw_summary,
            "mean_payload_bytes": "n/a",
            "mean_encode_ms": "n/a",
            "mean_decode_ms": "n/a",
        },
        {
            "run_id": args.run_id,
            "representation": "ground_q5",
            "frames": len(rows),
            "confidence_threshold": args.confidence,
            "jpeg_quality": args.jpeg_quality,
            **q5_summary,
            "mean_payload_bytes": float(np.mean(q5_sizes)),
            "mean_encode_ms": float(np.mean(encode_times)),
            "mean_decode_ms": float(np.mean(decode_times)),
        },
    ]

    summary_fields = [
        "run_id",
        "representation",
        "frames",
        "confidence_threshold",
        "jpeg_quality",
        "tp",
        "fp",
        "fn",
        "precision",
        "recall",
        "f1",
        "mean_confidence",
        "exact_count_accuracy",
        "mean_inference_ms",
        "median_inference_ms",
        "p95_inference_ms",
        "mean_payload_bytes",
        "mean_encode_ms",
        "mean_decode_ms",
    ]

    with summary_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as output_file:
        writer = csv.DictWriter(
            output_file,
            fieldnames=summary_fields,
        )
        writer.writeheader()

        for summary_row in summary_rows:
            formatted = dict(summary_row)

            for key, value in formatted.items():
                if isinstance(value, float):
                    formatted[key] = f"{value:.6f}"

            writer.writerow(formatted)

    print("\nD4 COUNT-BASED DETECTION RESULTS")
    print("=" * 66)
    print(
        f"{'Representation':<16}"
        f"{'Precision':>11}"
        f"{'Recall':>11}"
        f"{'F1':>11}"
        f"{'Exact count':>14}"
    )
    print("-" * 66)

    for label, summary in (
        ("Edge/raw", raw_summary),
        ("Ground/q5", q5_summary),
    ):
        print(
            f"{label:<16}"
            f"{summary['precision']:>11.3f}"
            f"{summary['recall']:>11.3f}"
            f"{summary['f1']:>11.3f}"
            f"{summary['exact_count_accuracy']:>14.3f}"
        )

    print("=" * 66)
    print(f"Detailed CSV: {detailed_path}")
    print(f"Summary CSV:  {summary_path}")
    print(
        f"Mean q5 payload: {np.mean(q5_sizes) / 1024:.1f} KiB"
    )
    print(
        f"Mean q5 encode/decode: "
        f"{np.mean(encode_times):.2f} / "
        f"{np.mean(decode_times):.2f} ms"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
#!/usr/bin/env python3
"""Validate the conference-paper quality-sweep input dataset."""

from __future__ import annotations

import csv
import re
import sys
from pathlib import Path

import cv2


REPO_ROOT = Path(__file__).resolve().parents[3]
IMAGE_DIR = (
    REPO_ROOT / "results/phase_d_application/raw/d4_reference_01"
)
GROUND_TRUTH = Path(__file__).resolve().parent / "input/ground_truth_clean.csv"
EXPECTED_FILENAMES = [f"frame_{index:04d}.png" for index in range(1, 61)]


def main() -> int:
    failures: list[str] = []

    image_paths = sorted(IMAGE_DIR.glob("*.png"))
    image_filenames = [path.name for path in image_paths]

    if len(image_paths) != 60:
        failures.append(f"expected 60 PNG images, found {len(image_paths)}")
    if image_filenames != EXPECTED_FILENAMES:
        failures.append("PNG filenames are not exactly frame_0001.png through frame_0060.png")
    if len(image_filenames) != len(set(image_filenames)):
        failures.append("PNG filenames are not unique")

    try:
        with GROUND_TRUTH.open(newline="", encoding="utf-8") as csv_file:
            reader = csv.DictReader(csv_file)
            if reader.fieldnames != ["filename", "gt_count", "notes"]:
                failures.append("ground-truth columns must be filename, gt_count, notes")
            rows = list(reader)
    except (OSError, csv.Error) as exc:
        failures.append(f"could not read cleaned ground truth: {exc}")
        rows = []

    gt_filenames = [str(row.get("filename", "")).strip() for row in rows]
    if len(rows) != 60:
        failures.append(f"expected 60 ground-truth rows, found {len(rows)}")
    if len(gt_filenames) != len(set(gt_filenames)):
        failures.append("ground-truth filenames are not unique")

    image_set = set(image_filenames)
    gt_set = set(gt_filenames)
    missing_pngs = sorted(gt_set - image_set)
    missing_annotations = sorted(image_set - gt_set)
    if missing_pngs:
        failures.append(f"ground-truth rows without PNGs: {', '.join(missing_pngs)}")
    if missing_annotations:
        failures.append(f"PNGs without ground-truth rows: {', '.join(missing_annotations)}")

    for row_number, row in enumerate(rows, start=2):
        value = str(row.get("gt_count", "")).strip()
        if re.fullmatch(r"\d+", value) is None:
            failures.append(f"row {row_number} has invalid gt_count: {value!r}")

    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            failures.append(f"could not decode {image_path.name}")
        elif image.shape != (720, 1280, 3):
            failures.append(
                f"{image_path.name} has shape {image.shape}, expected (720, 1280, 3)"
            )

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1

    print("PASS: 60 unique labelled PNG frames; filenames, dimensions, channels, and counts valid.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

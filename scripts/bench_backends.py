#!/usr/bin/env python3
"""Time each inference back-end on the same image. Run ON THE PI.

Standalone benchmark: no ROS, no network, nothing else competing. The figures
are the floor - what the model costs with the board to itself. Subtracting
this from the in-pipeline figure gives the contention.

    ~/yolo_env/bin/python bench_backends.py [image.jpg]

CORRECTNESS IS CHECKED BEFORE SPEED. Every back-end is called repeatedly and
the detection count must be identical every time. A back-end that varies is
reported as BROKEN and excluded from the ranking, however fast it is.

That check exists because of a real failure. An NCNN model exported at
384x640, called with imgsz=640, returned:

    [3, 13, 13, 13, 13, 13]

Correct on the first call, then thirteen boxes spanning most of the image at
confidence 0.88. Nothing errored. A benchmark that timed the model once would
have reported a clean 3x speed-up and recommended it.

The cause: for NCNN the input shape is FIXED at export time, and passing imgsz
overrides the shape ultralytics uses to decode the output coordinates.
imgsz=640 means 640x640 square, which is not 384x640. So NCNN entries below
pass no imgsz at all and let the export decide.
"""
import statistics as st
import sys
import time
from pathlib import Path

import cv2
from ultralytics import YOLO

IMAGE = sys.argv[1] if len(sys.argv) > 1 else "/home/anton/frame_640x384.jpg"
RUNS = 10
WARMUP = 3
CONF = 0.4
PERSON = 0

# (label, path, kwargs).  NCNN entries deliberately pass NO imgsz.
CANDIDATES = [
    ("PyTorch  640",  "~/models/yolov8n.pt",                 {"imgsz": 640}),
    ("PyTorch  480",  "~/models/yolov8n.pt",                 {"imgsz": 480}),
    ("PyTorch  320",  "~/models/yolov8n.pt",                 {"imgsz": 320}),
    ("NCNN 384x640",  "~/models/yolov8n_384x640_ncnn_model", {}),
    ("NCNN 544x960",  "~/models/yolov8n_ncnn_model",         {}),
    ("YOLO11n NCNN",  "~/models/yolo11n_384x640_ncnn_model", {}),
    ("YOLO11n torch", "~/models/yolo11n.pt",                 {"imgsz": 640}),
    ("YOLO11n OpenVINO", "~/models/yolo11n_openvino_model", {}),
    ("YOLO11n MNN",      "~/models/yolo11n.mnn",            {}),


]


def bench(label, path, kwargs, img):
    p = Path(path).expanduser()
    if not p.exists():
        return None, f"not found: {p}"
    try:
        model = YOLO(str(p))
        for _ in range(WARMUP):
            model(img, classes=[PERSON], conf=CONF, verbose=False, **kwargs)
        times, counts = [], []
        for _ in range(RUNS):
            t0 = time.time()
            r = model(img, classes=[PERSON], conf=CONF, verbose=False, **kwargs)[0]
            times.append((time.time() - t0) * 1000.0)
            counts.append(len(r.boxes))
    except Exception as e:
        return None, f"FAILED: {type(e).__name__}: {e}"

    if len(set(counts)) != 1:
        return None, f"BROKEN - count varies across calls: {counts}"
    # Median, not mean: one 7 s outlier was seen in 480 frames on this board,
    # and a mean lets a single scheduling hiccup dominate.
    return (st.median(times), min(times), max(times), counts[0]), None


def main():
    img = cv2.imread(IMAGE)
    if img is None:
        print(f"cannot read {IMAGE}")
        print("pass an image path as the first argument")
        return
    h, w = img.shape[:2]
    print(f"image: {IMAGE}  ({w}x{h})")
    print(f"{RUNS} runs each, {WARMUP} warm-up, median reported")
    print("correctness checked first: the detection count must not vary\n")
    print(f"  {'back-end':<16} {'median':>9} {'min':>8} {'max':>8} {'people':>7}")
    print("  " + "-" * 52)

    good = []
    for label, path, kwargs in CANDIDATES:
        res, err = bench(label, path, kwargs, img)
        if err:
            print(f"  {label:<16} {err}")
            continue
        med, lo, hi, n = res
        print(f"  {label:<16} {med:>7.0f}ms {lo:>7.0f}ms {hi:>7.0f}ms {n:>7}")
        good.append((label, med, n))

    if not good:
        return

    counts = {n for _, _, n in good}
    if len(counts) != 1:
        print(f"\n  WARNING: back-ends disagree on how many people are present: {counts}")
        print("  A faster back-end that finds a different number is not a speed-up.")

    base_label, base, _ = good[0]
    print(f"\n  speed-up relative to {base_label}:")
    for label, med, _ in good[1:]:
        print(f"    {label:<16} {base / med:>5.2f}x")

    best_label, best, _ = min(good, key=lambda x: x[1])
    print(f"\n  fastest correct: {best_label} at {best:.0f} ms  ->  {1000/best:.2f} fps")
    print("  compare with the in-pipeline figure from ~/rate.sh;")
    print("  the difference is contention, not model cost.")


if __name__ == "__main__":
    main()

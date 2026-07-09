"""
yolo_test.py
------------
Test YOLO detection across ALL frames in an altitude folder.
Reports per-frame results plus a summary (detection rate, avg confidence).

Usage:
    python3 yolo_test.py ~/FYP/multi_uav_sim/yolo_test_frames/10
    python3 yolo_test.py ~/FYP/multi_uav_sim/yolo_test_frames/40
"""

import sys
import os
import cv2
from ultralytics import YOLO

model = YOLO('yolov8n.pt')

folder = sys.argv[1]
frames = sorted([f for f in os.listdir(folder)
                  if f.endswith('.jpg') and not f.startswith('yolo_result')])

if not frames:
    print(f"No jpg frames found in {folder}")
    sys.exit(1)

print(f"\n{'='*60}")
print(f"Testing {len(frames)} frames in: {folder}")
print(f"{'='*60}\n")

# Make an output subfolder for annotated results
out_dir = os.path.join(folder, 'yolo_annotated')
os.makedirs(out_dir, exist_ok=True)

person_detections = []   # confidence values where "person" was found
frames_with_person = 0

for fname in frames:
    fpath = os.path.join(folder, fname)
    results = model(fpath, conf=0.15, verbose=False)
    boxes = results[0].boxes

    found_person = False
    frame_persons = []

    for box in boxes:
        cls = model.names[int(box.cls)]
        conf = float(box.conf)
        if cls == 'person':
            found_person = True
            frame_persons.append(conf)
            person_detections.append(conf)

    if found_person:
        frames_with_person += 1
        confs_str = ', '.join(f'{c:.2f}' for c in frame_persons)
        print(f"  {fname:<20} person DETECTED  (conf: {confs_str})")
    else:
        # show what else (if anything) was detected, for context
        other = [model.names[int(b.cls)] for b in boxes]
        other_str = f"  (found: {other})" if other else ""
        print(f"  {fname:<20} no person{other_str}")

    # save annotated frame
    annotated = results[0].plot()
    cv2.imwrite(os.path.join(out_dir, fname), annotated)

# ── Summary ────────────────────────────────────────────────────────────────
print(f"\n{'='*60}")
print(f"SUMMARY — {folder}")
print(f"{'='*60}")
print(f"  Frames tested:        {len(frames)}")
print(f"  Frames with person:   {frames_with_person}")
print(f"  Detection rate:       {100*frames_with_person/len(frames):.0f}%")
if person_detections:
    avg_conf = sum(person_detections) / len(person_detections)
    print(f"  Avg confidence:       {avg_conf:.2f}")
    print(f"  Min / Max confidence: {min(person_detections):.2f} / "
          f"{max(person_detections):.2f}")
else:
    print(f"  Avg confidence:       N/A (no detections)")
print(f"  Annotated frames:     {out_dir}")
print(f"{'='*60}\n")
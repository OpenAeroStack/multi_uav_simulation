"""
yolo_test.py
------------
Test YOLO detection across a folder of altitude test frames.

Usage:
    python3 yolo_test.py ~/FYP/multi_uav_sim/yolo_test_frames/10
    python3 yolo_test.py ~/FYP/multi_uav_sim/yolo_test_frames/20
    ... etc for each altitude folder
"""

import sys
import os
from ultralytics import YOLO

model = YOLO('yolov8n.pt')

folder = sys.argv[1]
frames = sorted([f for f in os.listdir(folder) if f.endswith('.jpg')])

if not frames:
    print(f"No jpg frames found in {folder}")
    sys.exit(1)

# Use the last frame (most stable, drone settled by then)
last_frame = os.path.join(folder, frames[-1])
print(f"\nTesting: {last_frame}")

results = model(last_frame, conf=0.15)  # lower conf threshold to see borderline detections
boxes = results[0].boxes

print(f"Total detections: {len(boxes)}")
if len(boxes) == 0:
    print("  → NO DETECTIONS at this altitude")
else:
    for box in boxes:
        cls = model.names[int(box.cls)]
        conf = float(box.conf)
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        w, h = x2-x1, y2-y1
        print(f"  {cls}: conf={conf:.2f}  box_size={w:.0f}x{h:.0f}px")

# Save annotated image for visual inspection
annotated = results[0].plot()
out_path = os.path.join(folder, 'yolo_result.jpg')
import cv2
cv2.imwrite(out_path, annotated)
print(f"Annotated image saved: {out_path}")
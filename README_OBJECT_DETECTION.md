# Human Detection with Drone Camera

Detects people in the drone's camera feed using YOLOv8.

## How to run

**1. Start the simulation:**
```bash
./launch/launch_single_dds.sh
```

**2. Boot SITL (new terminal):**
```bash
mavproxy.py --master=tcp:127.0.0.1:5760
```

**3. Start detection (new terminal):**
```bash
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
./venv/bin/python scripts/yolo_detect_node.py
```

A window opens showing green boxes on detected people.

## Notes

- Always run the detector with `./venv/bin/python` after sourcing ROS.
- If you get a NumPy error: `./venv/bin/pip install "numpy<2"`
- To see boxes in rqt, open `/uav1/camera/annotated` (not `image_raw`).

# Human Detection with Drone Camera

Detects people in the drone's camera feed using YOLOv8, running as a ROS2 node
on top of the ArduPilot + Gazebo DDS simulation.

## Pipeline

```
Gazebo camera                YOLOv8 node                   outputs
/uav1/camera/image_raw  ->  scripts/yolo_detect_node.py ->  /uav1/detections/humans  (JSON list)
(sensor_msgs/Image)                                         /uav1/camera/annotated   (Image w/ boxes)
                                                            "UAV1 human detection"   (OpenCV window)
```

- The camera comes from `models/gimbal_small_2d` (libgazebo_ros_camera, namespace
  `/uav1`), mounted on `iris_1`, which `worlds/single_uav.world` spawns.
- The gimbal tilt joint is locked and the sensor pose set in
  `models/gimbal_small_2d/model.sdf`. The camera pose is `0 0 0 -1.57 PITCH 0`,
  where the middle value (pitch) aims the view: `-1.57` = straight forward,
  toward `-1.0` (current, oblique down) for the aerial survey, toward `0` for
  more steeply down. Restart Gazebo after changing it.
- A static `person_standing` model is placed in front of the drone so there is a
  human to detect.
- YOLO reports only COCO class `0` ("person").

## How to run

**1. Start the simulation:**
```bash
cd ~/multi_uav_simulation
./launch/launch_single_dds.sh
```

**2. Boot SITL (new terminal):**
```bash
mavproxy.py --master=tcp:127.0.0.1:5760
```
Wait until the launch terminal prints `session established`.

**3. Start detection (new terminal):**
```bash
cd ~/multi_uav_simulation
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
./venv/bin/python scripts/yolo_detect_node.py
```
A window opens showing green boxes on detected people, and detections stream on
`/uav1/detections/humans`.

**4. (Optional) Fly the survey while detecting:**
```bash
cd ~/multi_uav_simulation
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
./venv/bin/python scripts/fly_survey_dds.py
```
This DDS-native script arms, takes off, flies forward over the people, then RTLs
while the detector runs alongside. It controls the drone entirely through
ArduPilot's AP_DDS interface (`/ap/mode_switch`, `/ap/arm_motors`,
`/ap/experimental/takeoff`, `/ap/cmd_vel`) — no MAVLink/pymavlink.

## Node parameters

`scripts/yolo_detect_node.py` exposes ROS2 parameters (override with `-p`):

| Parameter     | Default                   | Description                          |
|---------------|---------------------------|--------------------------------------|
| `image_topic` | `/uav1/camera/image_raw`  | Input camera topic                   |
| `model_path`  | `yolov8n.pt`              | YOLO weights (auto-downloads if absent) |
| `show_window` | `true`                    | Show the live OpenCV preview window   |

Detection confidence threshold is `CONF_THRESHOLD = 0.4` in the script.

Example (headless, custom topic):
```bash
./venv/bin/python scripts/yolo_detect_node.py \
  --ros-args -p show_window:=false -p image_topic:=/uav2/camera/image_raw
```

## Notes / troubleshooting

- Always run the detector with `./venv/bin/python` **after** sourcing ROS — that
  is the only combo where `rclpy`, `cv_bridge`, `ultralytics`, and `torch` all
  import together.
- NumPy error (`_ARRAY_API not found` / NumPy 2.x): `cv_bridge` is compiled
  against NumPy 1.x. Re-pin with `./venv/bin/pip install "numpy<2"`.
- To inspect results in `rqt`, open `/uav1/camera/annotated` (not `image_raw`).
- No detections? Confirm the camera is publishing: `ros2 topic hz /uav1/camera/image_raw`.

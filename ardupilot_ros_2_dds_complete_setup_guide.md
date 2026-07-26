# ArduPilot + ROS2 + DDS + Gazebo Integration Guide

## Overview

This guide explains how to integrate:

- ArduPilot SITL
- Gazebo Classic 11
- ROS2 Humble
- AP_DDS (ArduPilot DDS)
- micro-ROS Agent
- ROS2 Camera Topics

using an already working:

```text
Gazebo + ArduPilot SITL + MAVProxy
```

simulation system.

This guide is written for beginners and follows the exact workflow used in the project.

---

# Final System Architecture

```text
Gazebo World
     ↓
Gazebo UAV Model
     ↓
ArduPilot SITL
     ↓
AP_DDS
     ↓
micro_ros_agent
     ↓
ROS2 Topics
     ↓
YOLO / AI / Mission Nodes
```

---

# What Was Already Working

Before starting DDS integration, the following already worked:

- Gazebo Classic 11
- ArduPilot SITL
- MAVProxy
- Single and multi-UAV launch scripts
- Gazebo worlds
- ArduPilot Gazebo plugin

Existing workflow:

```bash
bash launch/launch_single.sh
```

This launched:

```text
Gazebo ↔ ArduPilot SITL ↔ MAVProxy
```

---

# Goal of DDS Integration

The goal is to expose:

- telemetry
- GPS
- battery
- pose
- camera feed

as ROS2 topics.

This allows:

- AI nodes
- YOLO human detection
- autonomous mission logic
- ROS2 communication

---

# Important Concept

There are TWO separate systems:

| System | Purpose |
|---|---|
| AP_DDS | Flight telemetry via ROS2 |
| Gazebo ROS Camera Plugin | Camera image topics |

DDS telemetry topics:

```text
/ap/navsat
/ap/battery
/ap/pose/filtered
```

Camera topics:

```text
/uav1/camera/image_raw
```

These are different systems.

---

# Step 1 — Install ROS2 Humble

Ubuntu version used:

```text
Ubuntu 22.04
```

Install ROS2 Humble:

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Install ROS2 repository:

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Install ROS2 desktop:

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

Add ROS2 to bashrc:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
ros2 topic list
```

---

# Step 2 — Create ROS2 Workspace

Create workspace:

```bash
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
```

Install vcs:

```bash
sudo apt install python3-vcstool
```

Clone AP_DDS repositories:

```bash
vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
```

---

# Step 3 — Install Dependencies

```bash
cd ~/ardu_ws
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

---

# Step 4 — Install Micro-XRCE-DDS-Gen

Install Java:

```bash
sudo apt install default-jre
```

Clone generator:

```bash
cd ~/ardu_ws
git clone --recurse-submodules --branch v4.7.0 https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
```

Build:

```bash
cd Micro-XRCE-DDS-Gen
./gradlew assemble
```

Add to PATH:

```bash
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
microxrceddsgen -help
```

---

# Step 5 — Build DDS-Enabled ArduPilot

IMPORTANT:

Use the ArduPilot inside:

```text
~/ardu_ws/src/ardupilot
```

NOT another older ArduPilot folder.

Go to ArduPilot:

```bash
cd ~/ardu_ws/src/ardupilot
```

Install ArduPilot dependencies:

```bash
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Configure SITL with DDS:

```bash
./waf configure --board sitl --enable-DDS
```

Verify DDS enabled:

```bash
./waf configure --board sitl --enable-DDS 2>&1 | grep -i dds
```

Expected:

```text
DDS client : enabled
```

Build Copter:

```bash
./waf copter
```

---

# Step 6 — Update setup.sh

Open:

```bash
nano ~/multi_uav_simulation/setup.sh
```

Set:

```bash
export ARDUPILOT_HOME="$HOME/ardu_ws/src/ardupilot"
```

This is VERY important.

Otherwise the old non-DDS ArduPilot build will be used.

---

# Step 7 — Create DDS Parameter File

Create params directory:

```bash
cd ~/multi_uav_simulation
mkdir -p params
```

Create:

```bash
nano params/uav1_dds.parm
```

Add:

```text
DDS_ENABLE 1
DDS_UDP_PORT 2019
DDS_DOMAIN_ID 0

ARMING_CHECK 0
```

Explanation:

| Parameter | Purpose |
|---|---|
| DDS_ENABLE | Enable DDS |
| DDS_UDP_PORT | DDS communication port |
| DDS_DOMAIN_ID | ROS2 DDS domain |
| ARMING_CHECK 0 | Disable checks for simulation |

---

# Step 8 — Install Gazebo ROS Packages

Install Gazebo ROS integration:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

This installs:

- libgazebo_ros_camera.so
- Gazebo ROS bridges
- image transport plugins

---

# Step 9 — Add ROS2 Camera Plugin

Open camera model:

```bash
nano ~/multi_uav_simulation/models/gimbal_small_2d/model.sdf
```

Find:

```xml
<sensor name="camera_sensor" type="camera">
```

Inside the sensor block, add:

```xml
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">

  <ros>
    <namespace>/uav1</namespace>
  </ros>

  <camera_name>camera</camera_name>

  <image_topic_name>image_raw</image_topic_name>

  <camera_info_topic_name>camera_info</camera_info_topic_name>

  <frame_name>camera_link</frame_name>

</plugin>
```

This creates ROS2 camera topics.

---

# Step 10 — Launch DDS Simulation

Run:

```bash
cd ~/multi_uav_simulation
bash launch/launch_single_dds.sh
```

This launches:

- Gazebo
- micro_ros_agent
- DDS-enabled ArduPilot SITL
- ROS2 bridge

---

# Step 11 — Connect MAVProxy

Open another terminal:

```bash
mavproxy.py --master=tcp:127.0.0.1:5760
```

You should see:

```text
Detected vehicle
online system
```

---

# Step 12 — Fly the Drone

In MAVProxy:

Switch to guided mode:

```text

```

Arm:

```text
arm throttle
```

Takeoff:

```text
takeoff 10
```

The drone should fly in Gazebo.

---

# Step 13 — Verify ROS2 Topics

Open another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
```

List topics:

```bash
ros2 topic list
```

Expected telemetry topics:

```text
/ap/navsat
/ap/battery
/ap/pose/filtered
```

---

# Step 14 — Verify Camera Topic

Check image topic:

```bash
ros2 topic list | grep image
```

Expected:

```text
/uav1/camera/image_raw
```

---

# Step 15 — View Camera Feed

Run:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/uav1/camera/image_raw
```

You should see live UAV camera video.

---

# Useful ROS2 Commands

List topics:

```bash
ros2 topic list
```

Inspect topic:

```bash
ros2 topic info /ap/navsat
```

View live data:

```bash
ros2 topic echo /ap/navsat
```

Check frequency:

```bash
ros2 topic hz /ap/navsat
```

---

# Final Working System

The final working architecture:

```text
Gazebo Camera
      ↓
ROS2 Image Topic
      ↓
YOLO Node
      ↓
Human Detection
```

Telemetry architecture:

```text
ArduPilot SITL
      ↓
AP_DDS
      ↓
micro_ros_agent
      ↓
ROS2 Topics
```

---

# Common Errors and Fixes

## Error: No DDS Topics

Cause:

```text
DDS not enabled
```

Fix:

```bash
./waf configure --board sitl --enable-DDS
```

---

## Error: No Camera Topic

Cause:

```text
ROS camera plugin missing
```

Fix:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

and add:

```xml
libgazebo_ros_camera.so
```

---

## Error: MAVProxy Takeoff Failed

Cause:

```text
Vehicle in STABILIZE mode
```

Fix:

```text
mode guided
arm throttle
takeoff 10
```

---

## Error: Failed to load defaults

Cause:

```text
Missing DDS parameter file
```

Fix:

```text
params/uav1_dds.parm
```

---

# Next Phase — YOLO Integration

Next step:

```text
ROS2 image subscriber
        ↓
OpenCV conversion
        ↓
YOLO inference
        ↓
Human detection
```

This becomes the ML phase of the UAV project.


# multi_uav_sim

A standalone Gazebo + ArduPilot SITL simulation for multi-UAV research.
All Gazebo models are bundled in the repo.

The stack integrates:
- **Gazebo Classic 11** — 3D physics simulation
- **ArduPilot SITL** — flight controller simulation
- **AP_DDS + micro_ros_agent** — ROS2 telemetry bridge
- **ROS2 Humble** — mission control and research layer
- **Camera feeds** — downward-facing cameras on each drone, published as ROS2 image topics

---

## System Requirements

- Ubuntu 22.04
- Gazebo Classic 11 (**not** Gazebo Garden/Harmonic/Fortress)
- ArduPilot built from source (with `--enable-DDS`)
- ArduPilot Gazebo plugin (`khancyr/ardupilot_gazebo`)
- ROS2 Humble
- MAVProxy
- Python 3

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        YOUR LAPTOP                          │
│                                                             │
│  ┌──────────┐   UDP    ┌─────────────┐   DDS    ┌───────┐  │
│  │  Gazebo  │◄────────►│  ArduPilot  │◄────────►│ ROS2  │  │
│  │  (3D)    │  FDM     │    SITL     │  MAVLink  │       │  │
│  └──────────┘          └─────────────┘◄────────►└───────┘  │
│       │                                          TCP        │
│  Camera feed                                               │
│  (libgazebo_ros_camera.so)                                  │
│  → /uavN/camera/image_raw                                  │
└─────────────────────────────────────────────────────────────┘

Communication layers:
  Gazebo ↔ ArduPilot       UDP FDM sockets (libArduPilotPlugin.so)
  ArduPilot → ROS2         AP_DDS via micro_ros_agent (telemetry)
  ROS2 → ArduPilot         MAVLink TCP via drone_bridge (commands)
  Gazebo camera → ROS2     libgazebo_ros_camera.so → /uavN/camera/image_raw
```

---

## Part 1 — Base Simulation Setup

### Step 1 — Install Gazebo 11

```bash
sudo apt update
sudo apt install gazebo11 libgazebo11-dev
```

Verify:
```bash
gazebo --version
# Should print: Gazebo multi-robot simulator, version 11.x.x
```

---

### Step 2 — Install ROS2 Humble

Follow the official guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Then install colcon and gazebo ROS packages:
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-gazebo-ros-pkgs
```

---

### Step 3 — Build ArduPilot with DDS support

> **Important:** This repo uses a specific ArduPilot workspace (`ardu_ws`) that includes
> `micro_ros_agent` and is built with `--enable-DDS`. Do NOT use a plain `~/ardupilot` clone.

```bash
# Create the workspace
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws/src

# Clone ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Build with DDS enabled (capital DDS)
./waf configure --board sitl --enable-dds
./waf copter
```

Verify DDS is compiled in:
```bash
./waf configure --board sitl --enable-DDS 2>&1 | grep -i dds
# Should show: DDS client : enabled
```

---

### Step 4 — Build micro_ros_agent

```bash
cd ~/ardu_ws/src
git clone https://github.com/microROS/micro-ROS-Agent
cd ~/ardu_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select micro_ros_agent ardupilot_msgs
```

---

### Step 5 — Install ArduPilot Gazebo Plugin

```bash
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

Verify:
```bash
ls /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/ | grep ArduPilot
# Should show: libArduPilotPlugin.so
```

---

### Step 6 — Install MAVProxy and pymavlink

```bash
pip3 install MAVProxy pymavlink
```

---

### Step 7 — Clone This Repo

```bash
git clone <your-repo-url> ~/FYP/multi_uav_sim
cd ~/FYP/multi_uav_sim
```

---

### Step 8 — Configure setup.sh

Open `setup.sh` and set `ARDUPILOT_HOME` to the ardu_ws ArduPilot:

```bash
export ARDUPILOT_HOME="$HOME/ardu_ws/src/ardupilot"
```

The following plugin paths must also be set in `setup.sh` for camera and physics plugins to load:
```bash
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH}
```

---

### Step 9 — Configure ~/.bashrc

Add these lines to the bottom of `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
export PATH=$PATH:~/.local/bin
export PATH=$PATH:~/ardu_ws/Micro-XRCE-DDS-Gen/scripts
source ~/ardu_ws/install/setup.bash
source ~/FYP/multi_uav_sim/ros2/install/setup.bash
```

Then reload:
```bash
source ~/.bashrc
```

> **Important:** Do NOT add `export COLCON_TRACE` to bashrc — this breaks ROS2 package sourcing.

---

### Step 10 — Build the ROS2 package

```bash
cd ~/FYP/multi_uav_sim
bash build_ros2.sh
```

---

## Part 2 — Running the Simulation

### Terminal 1 — Launch simulation

For single UAV with DDS/ROS2:
```bash
cd ~/FYP/multi_uav_sim
bash launch/launch_single_dds.sh
```

For 3 UAVs with DDS/ROS2 (required for `multi_mission`):
```bash
bash launch/launch_multi_dds.sh
```

This starts everything in one step: Gazebo, 3 SITL instances, 3 `micro_ros_agent` instances,
and 3 `drone_bridge` nodes.

Wait until you see:
```
[UAV1] ✓ DDS GPS flowing ...
[UAV2] ✓ DDS GPS flowing ...
[UAV3] ✓ DDS GPS flowing ...
```

---

### Terminal 2 — Run drone_bridge (single UAV)

```bash
ros2 run uav_controller drone_bridge
```

Wait until you see:
```
[UAV1] MAVLink heartbeat OK
[UAV1] Bridge ready | waiting for DDS GPS from /ap/navsat ...
[UAV1] ✓ DDS GPS flowing (-35.363262, 149.165237) — safe to call /uav1/takeoff now
```

---

### Terminal 3 — Verify topics

```bash
ros2 topic list
```

Expected topics include both `/ap/*` (DDS direct), `/uav1/*` (bridge), and camera:
```
/ap/navsat
/ap/pose/filtered
/ap/battery
/uav1/gps
/uav1/rel_alt
/uav1/mode
/uav1/armed
/uav1/battery
/uav1/camera/image_raw
/uav1/camera/camera_info
```

---

### Terminal 4 — Control via ROS2 services

```bash
ros2 service call /uav1/takeoff std_srvs/srv/Trigger {}
ros2 service call /uav1/rtl     std_srvs/srv/Trigger {}
ros2 service call /uav1/land    std_srvs/srv/Trigger {}
ros2 service call /uav1/arm     std_srvs/srv/Trigger {}
ros2 service call /uav1/disarm  std_srvs/srv/Trigger {}
```

---

## Part 3 — Camera Feeds

Each drone has a downward-facing camera attached to its gimbal. The camera feed is published
as a ROS2 image topic via `libgazebo_ros_camera.so`.

### Camera topics

| Drone | Image topic | Camera info topic |
|-------|-------------|-------------------|
| UAV1 | `/uav1/camera/image_raw` | `/uav1/camera/camera_info` |
| UAV2 | `/uav2/camera/image_raw` | `/uav2/camera/camera_info` |
| UAV3 | `/uav3/camera/image_raw` | `/uav3/camera/camera_info` |

### How it works

The camera plugin (`libgazebo_ros_camera.so`) is embedded directly in each drone's model SDF
(`models/iris_N/model.sdf`). When Gazebo launches with the `-s libgazebo_ros_init.so` and
`-s libgazebo_ros_factory.so` server plugins, the camera automatically publishes to ROS2 —
no extra nodes or scripts needed.

The launch scripts already include these flags.

### View camera feed — single drone

After launching `launch_single_dds.sh`:

```bash
# Verify camera topic is publishing
ros2 topic list | grep camera
ros2 topic hz /uav1/camera/image_raw

# View live feed
ros2 run rqt_image_view rqt_image_view
```

In `rqt_image_view`, select `/uav1/camera/image_raw` from the dropdown.

Install rqt_image_view if not present:
```bash
sudo apt install ros-humble-rqt-image-view
```

### View camera feed — 3 drones simultaneously

After launching `launch_multi_dds.sh`, open three separate `rqt_image_view` windows:

```bash
# Terminal A
ros2 run rqt_image_view rqt_image_view

# Terminal B
ros2 run rqt_image_view rqt_image_view

# Terminal C
ros2 run rqt_image_view rqt_image_view
```

Set each window to a different topic: `/uav1/camera/image_raw`, `/uav2/camera/image_raw`,
`/uav3/camera/image_raw`.

Or use a single rqt window with multiple image panels:
```bash
rqt
```
Go to **Plugins → Visualization → Image View** and add three panels.

### Save camera frames from ROS2

```bash
# Save a single frame
ros2 run image_transport republish raw --ros-args \
    -r in:=/uav1/camera/image_raw \
    -r out:=/uav1/camera/compressed

# Or use cv_bridge in Python to process frames
```

### Camera specifications

| Property | Value |
|----------|-------|
| Resolution | 640 × 480 |
| Format | RGB8 |
| Field of view | 2.0 rad (~115°) |
| Update rate | 10 Hz |
| Orientation | Downward-facing |
| Mount | gimbal_small_2d tilt_link |

---

## Part 4 — 3-Drone Coordinated Mission

`multi_mission` runs all three UAVs through a barrier-synchronised mission over ROS2.

**Mission phases:**

| Phase | Action |
|-------|--------|
| 0 | All 3 wait for GPS from their `drone_bridge` |
| 1 | All 3 takeoff to 20m — barrier sync |
| 2 | UAV1 → Left 50m, UAV2 → Right 50m, UAV3 → Forward 50m — barrier sync |
| 3 | All 3 hold 5 seconds — barrier sync |
| 4 | All 3 RTL simultaneously |

**Terminal 1:**
```bash
bash launch/launch_multi_dds.sh
```

Wait for all 3 `✓ DDS GPS flowing` messages.

**Terminal 2:**
```bash
ros2 run uav_controller multi_mission
```

---

## Part 5 — MAVProxy / Python Script Control (no ROS2)

```bash
# Single UAV
cd ~ && mavproxy.py --master=tcp:127.0.0.1:5760

# Automated missions
python3 scripts/single_drone_takeoff.py
python3 scripts/single_drone_mission.py
python3 scripts/multi_drone_mission.py
```

> Do not run MAVProxy and drone_bridge on the same TCP port simultaneously.

---

## ROS2 Package — uav_controller

Located at `ros2/uav_controller/`.

### Nodes

| Node | Command | Purpose |
|------|---------|---------|
| `drone_bridge` | `ros2 run uav_controller drone_bridge` | MAVLink+DDS ↔ ROS2 bridge |
| `takeoff_mission` | `ros2 run uav_controller takeoff_mission` | Auto arm+takeoff+RTL (single UAV) |
| `l_mission` | `ros2 run uav_controller l_mission` | L-shaped autonomous mission |
| `multi_mission` | `ros2 run uav_controller multi_mission` | Barrier-synchronised 3-drone mission |

### Topics published by drone_bridge

| Topic | Type | Source |
|-------|------|--------|
| `/uavN/gps` | `sensor_msgs/NavSatFix` | DDS `/ap/navsat` |
| `/uavN/rel_alt` | `std_msgs/Float32` | MAVLink GLOBAL_POSITION_INT |
| `/uavN/battery` | `std_msgs/Float32` | DDS `/ap/battery` |
| `/uavN/mode` | `std_msgs/String` | MAVLink heartbeat |
| `/uavN/armed` | `std_msgs/Bool` | MAVLink heartbeat |
| `/uavN/camera/image_raw` | `sensor_msgs/Image` | Gazebo camera plugin |
| `/uavN/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo camera plugin |

### Services

| Service | Type | Action |
|---------|------|--------|
| `/uavN/arm` | `std_srvs/Trigger` | Arm motors |
| `/uavN/disarm` | `std_srvs/Trigger` | Disarm motors |
| `/uavN/takeoff` | `std_srvs/Trigger` | Arm + takeoff |
| `/uavN/land` | `std_srvs/Trigger` | Switch to LAND mode |
| `/uavN/rtl` | `std_srvs/Trigger` | Return to launch |

### drone_bridge parameters

```bash
ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=1 \
    -p mavlink_port:=5760 \
    -p takeoff_altitude:=10.0
```

---

## Port Reference

| UAV | MAVLink TCP | FDM UDP in | FDM UDP out | DDS UDP |
|-----|-------------|------------|-------------|---------|
| UAV 1 | 5760 | 9002 | 9003 | 2019 |
| UAV 2 | 5770 | 9012 | 9013 | 2020 |
| UAV 3 | 5780 | 9022 | 9023 | 2021 |

---

## Project Structure

```
multi_uav_sim/
├── launch/
│   ├── launch_multi_uav.sh       # 3 UAV simulation (MAVLink only)
│   ├── launch_multi_dds.sh       # 3 UAV simulation (DDS + ROS2, all-in-one)
│   ├── launch_single.sh          # single UAV (MAVLink only)
│   └── launch_single_dds.sh      # single UAV with DDS + ROS2
├── models/
│   ├── iris_1/model.sdf          # UAV1: ports 9002/9003, camera /uav1/camera/
│   ├── iris_2/model.sdf          # UAV2: ports 9012/9013, camera /uav2/camera/
│   ├── iris_3/model.sdf          # UAV3: ports 9022/9023, camera /uav3/camera/
│   ├── iris_with_standoffs/      # base iris mesh
│   └── gimbal_small_2d/          # 2D gimbal (camera mounted on tilt_link)
├── params/
│   ├── uav1_dds.parm             # DDS_ENABLE=1, DDS_UDP_PORT=2019
│   ├── uav2_dds.parm             # DDS_ENABLE=1, DDS_UDP_PORT=2020
│   └── uav3_dds.parm             # DDS_ENABLE=1, DDS_UDP_PORT=2021
├── ros2/
│   └── uav_controller/
│       └── uav_controller/
│           ├── drone_bridge.py    # MAVLink+DDS ↔ ROS2 bridge
│           ├── takeoff_mission.py # single-drone auto mission
│           ├── l_mission.py       # L-shaped waypoint mission
│           └── multi_mission.py   # barrier-synchronised 3-drone mission
├── scripts/                      # pymavlink scripts (no ROS2 needed)
│   ├── single_drone_takeoff.py
│   ├── single_drone_mission.py
│   └── multi_drone_mission.py
├── worlds/
│   ├── multi_uav.world
│   └── single_uav.world
├── setup.sh
├── build_ros2.sh
└── README.md
```

---

## Troubleshooting

**`Package 'uav_controller' not found`**
```bash
source ~/.bashrc
cd ~/FYP/multi_uav_sim && bash build_ros2.sh
source ~/FYP/multi_uav_sim/ros2/install/setup.bash
```

**Camera topic not appearing (`/uavN/camera/image_raw` missing)**
- Make sure `launch_single_dds.sh` or `launch_multi_dds.sh` includes `-s libgazebo_ros_init.so -s libgazebo_ros_factory.so` in the Gazebo launch line
- Check `setup.sh` has both `GAZEBO_PLUGIN_PATH` and `LD_LIBRARY_PATH` set to include `/opt/ros/humble/lib`
- Verify `ros-humble-gazebo-ros-pkgs` is installed: `sudo apt install ros-humble-gazebo-ros-pkgs`

**DDS GPS not flowing**
```bash
ros2 topic list | grep /ap
# If empty, micro_ros_agent is not connected — check launch script output
```

**Arm timed out**
Wait for `✓ DDS GPS flowing` message before calling takeoff.

**Ports already in use**
```bash
pkill -f arducopter && pkill -f gzserver && pkill -f gzclient && pkill -f micro_ros_agent
sleep 5
```

**Cannot connect both MAVProxy and drone_bridge**
```bash
mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```

---

## Notes

- Requires **Gazebo Classic 11** — not compatible with Gazebo Harmonic/Garden
- `ARDUPILOT_HOME` must point to `~/ardu_ws/src/ardupilot` (DDS-enabled build)
- Do NOT set `COLCON_TRACE` in bashrc — it breaks ROS2 package sourcing
- All Gazebo models are bundled — no internet needed to run the simulation
- Camera feeds are published automatically when Gazebo launches — no extra nodes needed
- MAVProxy and drone_bridge cannot share the same TCP port simultaneously

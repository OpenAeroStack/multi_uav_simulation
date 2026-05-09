# multi_uav_sim

A standalone Gazebo + ArduPilot SITL simulation for multi-UAV research.
All Gazebo models are bundled in the repo.

The stack integrates:
- **Gazebo Classic 11** — 3D physics simulation
- **ArduPilot SITL** — flight controller simulation
- **AP_DDS + micro_ros_agent** — ROS2 telemetry bridge
- **ROS2 Humble** — mission control and research layer

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
│                                          TCP                │
└─────────────────────────────────────────────────────────────┘

Communication layers:
  Gazebo ↔ ArduPilot   UDP FDM sockets (libArduPilotPlugin.so)
  ArduPilot → ROS2     AP_DDS via micro_ros_agent (telemetry)
  ROS2 → ArduPilot     MAVLink TCP via drone_bridge (commands)
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

Then install colcon:
```bash
sudo apt install python3-colcon-common-extensions
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
./waf configure --board sitl --enable-DDS
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
git clone https://github.com/micro-ROS/micro_ros_agent.git

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

For 3 UAVs (MAVLink only, no DDS):
```bash
bash launch/launch_multi_uav.sh
```

Wait until you see:
```
Waiting for connection....
[info] running... | port: 2019     ← micro_ros_agent ready
Connection on serial port 5760     ← SITL booted
```

---

### Terminal 2 — Run drone_bridge (ROS2 control)

```bash
ros2 run uav_controller drone_bridge
```

Wait until you see:
```
[UAV1] MAVLink heartbeat OK
[UAV1] Bridge ready | waiting for DDS GPS from /ap/navsat ...
[UAV1] ✓ DDS GPS flowing (-35.363262, 149.165237) — safe to call /uav1/takeoff now
```

The third line confirms DDS is working and the drone is ready.

---

### Terminal 3 — Verify topics

```bash
# See all topics
ros2 topic list

# Verify DDS telemetry is flowing
ros2 topic echo /ap/navsat --once        # GPS from DDS
ros2 topic echo /uav1/gps --once         # GPS re-published by bridge
ros2 topic echo /uav1/rel_alt --once     # altitude above home
ros2 topic echo /uav1/mode --once        # flight mode
```

Expected topic list includes both `/ap/*` (DDS direct) and `/uav1/*` (bridge):
```
/ap/navsat
/ap/pose/filtered
/ap/battery
/uav1/gps
/uav1/rel_alt
/uav1/mode
/uav1/armed
/uav1/battery
```

---

### Terminal 4 — Control via ROS2 services

```bash
# Arm + takeoff to 10m
ros2 service call /uav1/takeoff std_srvs/srv/Trigger {}

# Return to launch
ros2 service call /uav1/rtl std_srvs/srv/Trigger {}

# Land
ros2 service call /uav1/land std_srvs/srv/Trigger {}

# Arm only
ros2 service call /uav1/arm std_srvs/srv/Trigger {}

# Disarm
ros2 service call /uav1/disarm std_srvs/srv/Trigger {}
```

Or run the automatic takeoff mission:
```bash
ros2 run uav_controller takeoff_mission
```

This auto-arms, takes off to 10m, hovers 10 seconds, then RTLs.

---

## Part 3 — MAVProxy / Python Script Control (no ROS2)

These work independently without running drone_bridge.

### Manual control via MAVProxy

```bash
# Single UAV
cd ~ && mavproxy.py --master=tcp:127.0.0.1:5760

# Multi UAV — one terminal per drone
cd ~ && mavproxy.py --master=tcp:127.0.0.1:5760 --logfile=~/mav1.tlog
cd ~ && mavproxy.py --master=tcp:127.0.0.1:5770 --logfile=~/mav2.tlog
cd ~ && mavproxy.py --master=tcp:127.0.0.1:5780 --logfile=~/mav3.tlog
```

### Automated missions via Python scripts

```bash
python3 scripts/single_drone_takeoff.py    # takeoff + hover
python3 scripts/single_drone_mission.py    # takeoff + waypoints + RTL
python3 scripts/multi_drone_mission.py     # 3 drones concurrent mission
```

> **Note:** Do not run MAVProxy and drone_bridge on the same TCP port simultaneously —
> they will compete for the connection. Use one or the other.

---

## ROS2 Package — uav_controller

Located at `ros2/uav_controller/`. Provides:

### Nodes

| Node | Command | Purpose |
|------|---------|---------|
| `drone_bridge` | `ros2 run uav_controller drone_bridge` | MAVLink+DDS ↔ ROS2 bridge |
| `takeoff_mission` | `ros2 run uav_controller takeoff_mission` | Auto arm+takeoff+RTL |

### Topics published by drone_bridge

| Topic | Type | Source |
|-------|------|--------|
| `/uav1/gps` | `sensor_msgs/NavSatFix` | DDS `/ap/navsat` |
| `/uav1/rel_alt` | `std_msgs/Float32` | DDS `/ap/pose/filtered` |
| `/uav1/battery` | `std_msgs/Float32` | DDS `/ap/battery` |
| `/uav1/mode` | `std_msgs/String` | MAVLink heartbeat |
| `/uav1/armed` | `std_msgs/Bool` | MAVLink heartbeat |

### Services

| Service | Type | Action |
|---------|------|--------|
| `/uav1/arm` | `std_srvs/Trigger` | Arm motors |
| `/uav1/disarm` | `std_srvs/Trigger` | Disarm motors |
| `/uav1/takeoff` | `std_srvs/Trigger` | Arm + takeoff to configured altitude |
| `/uav1/land` | `std_srvs/Trigger` | Switch to LAND mode |
| `/uav1/rtl` | `std_srvs/Trigger` | Return to launch |

### Parameters

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
│   ├── launch_single.sh          # single UAV (MAVLink only)
│   └── launch_single_dds.sh      # single UAV with DDS + ROS2
├── models/                       # all Gazebo models (bundled)
│   ├── iris_1/                   # UAV 1 (ports 9002/9003)
│   ├── iris_2/                   # UAV 2 (ports 9012/9013)
│   ├── iris_3/                   # UAV 3 (ports 9022/9023)
│   ├── iris_with_standoffs/      # base iris mesh
│   └── gimbal_small_2d/          # 2D gimbal with camera
├── params/
│   └── uav1_dds.parm             # DDS_ENABLE=1, DDS_UDP_PORT=2019
├── ros2/                         # ROS2 workspace
│   └── uav_controller/           # ROS2 package
│       └── uav_controller/
│           ├── drone_bridge.py   # MAVLink+DDS ↔ ROS2 bridge node
│           └── takeoff_mission.py# example mission node
├── scripts/                      # pymavlink scripts (no ROS2 needed)
│   ├── single_drone_takeoff.py
│   ├── single_drone_mission.py
│   └── multi_drone_mission.py
├── worlds/
│   ├── multi_uav.world
│   └── single_uav.world
├── setup.sh                      # environment variables
├── build_ros2.sh                 # builds the ROS2 package
└── README.md
```

---

## Troubleshooting

**`Package 'uav_controller' not found`**
```bash
# Make sure COLCON_TRACE is not set, then:
source ~/.bashrc
# If still failing, rebuild:
cd ~/FYP/multi_uav_sim && bash build_ros2.sh
source ~/FYP/multi_uav_sim/ros2/install/setup.bash
```

**DDS GPS not flowing (bridge stuck at "waiting for DDS GPS")**
```bash
# Check micro_ros_agent is running (should be started by launch script)
# Check DDS topics exist:
ros2 topic list | grep /ap
# If empty, micro_ros_agent is not connected to SITL
```

**Arm timed out**
Wait longer after `Bridge ready` before calling takeoff.
Watch for the `✓ DDS GPS flowing` message — only call takeoff after that.

**Ports already in use**
```bash
pkill -f arducopter && pkill -f gzserver && pkill -f gzclient && pkill -f micro_ros_agent
sleep 5
```

**MAVProxy permission denied**
```bash
cd ~ && mavproxy.py --master=tcp:127.0.0.1:5760 --logfile=~/mav1.tlog
```

**Cannot connect both MAVProxy and drone_bridge**
Use MAVProxy as a forwarder:
```bash
mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
# Then connect drone_bridge to UDP 14550
```

---

## Notes

- Requires **Gazebo Classic 11** — not compatible with Gazebo Harmonic/Garden
- `ARDUPILOT_HOME` must point to `~/ardu_ws/src/ardupilot` (DDS-enabled build)
- Do NOT set `COLCON_TRACE` in bashrc — it breaks ROS2 package sourcing
- All Gazebo models are bundled — no internet needed to run the simulation
- MAVProxy and drone_bridge cannot share the same TCP port simultaneously
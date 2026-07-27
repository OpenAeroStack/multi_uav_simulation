# Edge Processing — RPi5 Hardware-in-the-Loop

Branch: `ground-vs-edge-processing-RPi`

This document explains **edge processing**: running the vision detection **onboard
the drone** (on a real Raspberry Pi 5) so that only tiny detection results — not
video — travel over the wireless link to the ground station. For the base simulation
setup (Gazebo, SITL, ROS 2), see the main [README.md](README.md).

---

## 1. What edge processing means here

A surveillance drone sees people with its camera. In **edge processing** the drone
does the work itself:

1. The camera frame is processed **on the drone's onboard computer** (the RPi5).
2. YOLO runs there and produces a small detection result (~76 bytes: class,
   confidence, bounding box).
3. **Only that result** is sent to the ground station over the wireless link.

The heavy video never crosses the radio link — it stays on the drone. This saves
bandwidth and keeps working even when the wireless channel is poor.

---

## 2. Setup — the RPi5 as the onboard edge computer

The RPi5 is **UAV2's onboard computer**. Its autopilot (SITL) still runs on the
host; only its **vision** runs on the Pi. Vision only for now — the Pi does not
command the drone yet.

```
        HOST PC                                              RPi5  (UAV2)
┌────────────────────────┐                          ┌──────────────────────────┐
│ Gazebo (UAV2 camera)   │  /uav2/camera/image_raw   │  camera_relay (throttle) │
│ SITL   ·  ns-3  ·  GCS  │═════════════════════════▶│  detector (YOLO onboard) │
│                        │  ETHERNET — sensor link    │        │                 │
│                        │  (172.31.2.x, UNIMPAIRED)  │        │ /detections/uav2 │
│  ns-3  ◀───────────────────────────────────────────────────┘  (~76 bytes)     │
│  (simulated 802.11a)   │  ETHERNET — wireless link                             │
│  GCS  ◀── detections ──│  (10.42.0.x, THROUGH ns-3, impaired)                  │
└────────────────────────┘                          └──────────────────────────┘
```

---

## 3. Why the camera stream goes to the Pi over Ethernet

The camera is **simulated in Gazebo on the host**, so its frames have to physically
reach the Pi — over the **USB-Gigabit-Ethernet cable**.

On a *real* drone the camera is bolted to the Pi and connected by a **CSI ribbon
cable**; the frame is captured onboard and never touches a network. In this HITL rig
the Ethernet cable **stands in for that CSI cable**. This camera transport is a
**simulation artifact** and is *not* part of what is measured — it just feeds
simulated pixels into the real edge processor. Raw 640×480 RGB is ~73 Mbps, which
Gigabit Ethernet handles easily (and `camera_relay`'s `frame_rate_hz` throttle trims
it further).

### Two logical links (both ride the Ethernet cable(s))

| Link | Subnet | Carries | Impaired by ns-3? |
|---|---|---|---|
| Sensor / management | `172.31.2.x` | Gazebo camera → Pi (the drone's input bus) | No |
| Wireless | `10.42.0.x` | Pi → GCS: **76-byte detections only** | **Yes** |

The GCS lives **only** on the wireless subnet, so the OS automatically routes the
detection traffic through ns-3 while the camera takes the plain sensor link. (Two
USB-Ethernet adapters, or one adapter with two VLANs, keep the two links physically
separate so real Wi-Fi never contaminates the simulated channel.)

---

## 4. The edge nodes (`ros2/uav_vision/`)

All run on the drone side (the RPi5, or a namespace standing in for it), except the
receiver/logger which run at the GCS:

| Node | Where | Role |
|---|---|---|
| `camera_relay` | drone (Pi) | subscribe raw camera, throttle to `frame_rate_hz`, republish **locally** on `/cluster/cam/uavN` |
| `detector` | drone (Pi) | run YOLO on the local frame, publish `/detections/uavN` (~76 B) — this is what crosses the wireless link |
| `gcs_receiver` | GCS | receive the detections |
| `metrics_logger` | GCS | per-frame CSV: detection size, latency, `navsat_age` (telemetry health) |

The nodes deliberately avoid `cv_bridge` (NumPy 2.x incompatibility) and do their own
Image↔numpy conversion. `camera_relay` keeps the raw frame **local** to the drone —
nothing but the detection result leaves the onboard computer.

---

## 5. Host-only dry run (namespace stands in for the Pi)

Before the Pi is wired in, the whole thing runs on the host, with a **Linux network
namespace playing the RPi5's role** (`uav1ns` in the examples below; the real Pi will
be **`uav2ns`**). ns-3 bridges the namespaces' TAP devices to form the simulated
Wi-Fi.

### 5.1 One-time prerequisites

- ns-3 3.38 built with the scenario (see `ns3/CMakeLists.txt` header; scratch copy at
  `~/ns-allinone-3.38/ns-3.38/scratch/multi_uav_simulation`).
- `colcon build --packages-select multi_uav_gazebo_plugins uav_vision`.
- YOLO weights at `~/yolo_env/yolov8n.pt` (or pass `-p model_path:=...`); the detector
  needs `ultralytics` (kept in the repo `venv`).

### 5.2 Bring-up order

```bash
# 1. Namespaces + TAP devices (root)
sudo bash scripts/netns/wireless_up.sh       # gcsns + uavNns, taps, bridges (10.42.0.x)
sudo bash scripts/netns/management_up.sh     # sensor links (172.31.N.x)

# 2. Gazebo (root ns) with the single-UAV city world
gzserver --verbose worlds/small_city_single_uav.world     # + correct GAZEBO_PLUGIN_PATH

# 3. SITL inside the drone namespace
sudo ip netns exec uav1ns <ardupilot>/build/sitl/bin/arducopter \
     --model gazebo-iris --sysid 1 --sim-address=172.31.1.1 -I0

# 4. ns-3 wireless channel (taps must exist; NS3_EXECUTABLE_PATH must point at the
#    tap-bridge build dir so TapBridge can find its tap-creator helper)
export NS3_EXECUTABLE_PATH=~/ns-allinone-3.38/ns-3.38/build/src/tap-bridge:...
<ns3>/build/scratch/multi_uav_simulation/ns3.38-three_uav_tapbridge_integrated-default --simTime=0

# 5. Position feed (root ns) so ns-3 tracks real geometry + obstacle loss
python3 scripts/world_pos_publisher.py
```

### 5.3 The UDP-only DDS profile (REQUIRED on a single host)

On one machine, `/dev/shm` is shared across namespaces, so FastDDS would deliver
detections through **shared memory and bypass ns-3** — making any latency number
meaningless. Force UDP with the provided profile:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/config/fastdds_udp_only.xml
```

> On the **real Pi** (a separate machine with no shared `/dev/shm`) this profile is
> unnecessary but harmless — FastDDS already uses UDP there.

### 5.4 Launch the edge nodes

Each node runs inside a namespace and exports the DDS profile. The detector uses the
repo `venv` python (for `ultralytics`) with ROS on the path.

```bash
# uav1ns — camera_relay (throttle + local republish)
sudo ip netns exec uav1ns bash -c 'cd '"$PWD"' && source /opt/ros/humble/setup.bash && source install/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE='"$PWD"'/config/fastdds_udp_only.xml && ros2 run uav_vision camera_relay --ros-args -p uav_id:=1 -p processing_mode:=edge -p frame_rate_hz:=2.0'

# uav1ns — detector (YOLO onboard)
sudo ip netns exec uav1ns bash -c 'cd '"$PWD"' && source /opt/ros/humble/setup.bash && source install/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE='"$PWD"'/config/fastdds_udp_only.xml && ./venv/bin/python install/uav_vision/lib/uav_vision/detector --ros-args -p uav_id:=1 -p processing_mode:=edge -p model_path:='"$PWD"'/yolov8n.pt'

# gcsns — gcs_receiver
sudo ip netns exec gcsns bash -c 'cd '"$PWD"' && source /opt/ros/humble/setup.bash && source install/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE='"$PWD"'/config/fastdds_udp_only.xml && ros2 run uav_vision gcs_receiver --ros-args -p uav_id:=1 -p processing_mode:=edge'
```

**Success:** `gcs_receiver` logs `[GCS] detection #... size=76B` — those bytes crossed
the ns-3 simulated Wi-Fi. Fly a mission (e.g. `scripts/single_drone_takeoff.py`) so
the camera passes over people and the person count goes non-zero.

---

## 6. Moving from namespace to the real Pi

The namespace `uav2ns` and the RPi5 are interchangeable by design:

1. Replace the `uav2ns` veth in bridge `br-uav2` with the **physical USB-Ethernet
   NIC** facing the Pi.
2. Give the Pi `10.42.0.12` (wireless, routed through ns-3) and `172.31.2.2` (sensor).
3. Run the same `camera_relay` + `detector` on the Pi. No DDS profile needed.
4. Sync clocks (chrony/PTP) so cross-machine latency numbers are valid.

See `docs/HITL_INTEGRATION_PLAN.md` for the full phased plan and open items.

---

## 7. Key files

| Path | What |
|---|---|
| `ros2/uav_vision/` | the edge nodes (`camera_relay`, `detector`, `gcs_receiver`, `metrics_logger`) |
| `scripts/netns/wireless_up.sh` · `management_up.sh` | create namespaces, TAPs, the two links |
| `scripts/netns/netns_down.sh` | tear everything down |
| `config/fastdds_udp_only.xml` | force UDP (disable SHM) for host-only runs |
| `ns3/` | the wireless-channel model (obstacle shadowing + Nakagami fading) |
| `worlds/small_city_single_uav.world` | single-UAV city world (netns, camera, obstacle plugin) |
| `docs/HITL_INTEGRATION_PLAN.md` | the phased HITL plan |
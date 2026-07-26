# HITL Integration Plan — Raspberry Pi 5 edge node ↔ SITL host

**Author:** anton · **Status:** in progress · **Branch:** `ground-vs-edge-processing-RPi`
(sub-branch of `ground-vs-edge-processing`)

## Goal

Move UAV1's **onboard vision compute** off the host and onto a **real Raspberry Pi 5**
(companion-computer / hardware-in-the-loop), while the autopilot (ArduPilot SITL),
Gazebo, ns-3 and the ground station stay on the host PC. This turns the pure-software
`edge vs ground` experiment into a HITL one where the "edge" node is real silicon.

### Decisions (locked)
- **Pi 5** as the edge compute node.
- **USB-Gigabit-Ethernet adapter(s)** for the Pi↔host link (host has WiFi only).
- **Vision only first** — the Pi runs `camera_relay` + `detector`; it does NOT command
  the drone yet. SITL/autopilot stays on the host.
- **Topology: 2 drones total.** UAV1 = host/"cluster" drone (SITL + vision on host).
  UAV2 = the RPi5 hardware edge node (vision only; UAV2's SITL still runs on the host).
  So **the Pi replaces the `uav2ns` namespace** (10.42.0.12 wireless / 172.31.2.2 sensor),
  which `wireless_up.sh` + `management_up.sh` already create. GCS on host.

## The core mapping

The Pi replaces the `uav1ns` namespace. Everything randilsk built for that namespace
already fits — including its **two separate links**:

```
PURE SIM (now)                          HITL (target)
root ns:  Gazebo + SITL + ns-3          HOST PC:  Gazebo + SITL + ns-3 + gcsns
uav1ns :  camera_relay + detector  ──▶  PI 5:     camera_relay + detector (edge)
gcsns  :  gcs_receiver + metrics        HOST PC:  gcsns (unchanged)

uav1ns had TWO links; the Pi needs the SAME two:
  1. sensor/mgmt link (172.31.1.x)  UNIMPAIRED → Gazebo camera frames reach the Pi
  2. wireless link    (10.42.0.x)   via ns-3   → Pi→GCS detections/video get loss+fading
```
Because the GCS is reachable only on the 10.42.0.x subnet (behind ns-3), DDS
automatically routes Pi→GCS traffic through the impaired channel — the same trick the
namespaces use. See `scripts/netns/wireless_up.sh` and `management_up.sh`.

## Phases

### Phase 1 — Install ns-3 3.38 on the host  ✅ DONE (2026-07-26)
- All build deps already present; no sudo needed.
- Installed at `~/ns-allinone-3.38/ns-3.38`; scenario staged in `scratch/multi_uav_simulation`.
- `./ns3 configure` (ROS 2 sourced) → Tap Bridge ON, rclcpp found. `./ns3 build three_uav_tapbridge_integrated` OK.
- Binary verified: `build/scratch/multi_uav_simulation/ns3.38-three_uav_tapbridge_integrated-default`,
  linked against `librclcpp.so` + `std_msgs`, runs `--PrintHelp`.
- **Remember:** the scratch copy is a COPY — after editing `ns3/*.cc` in the repo, re-copy into
  `scratch/multi_uav_simulation/` and rebuild.

### Phase 2 — Baseline the stack on the host alone
**Phase 2a (flat, no netns/ns-3) ✅ DONE (2026-07-26):** validated the whole vision pipeline
end-to-end with zero sudo:
`Gazebo(single_uav.world) + SITL(flat,127.0.0.1) → camera_relay(edge) → detector(YOLO) → gcs_receiver`.
`/uav1/camera/image_raw` @ ~6 Hz; detector ~44 ms/frame on host x86; edge detection msg = **76 bytes**.
0 persons (empty world) — plumbing proven; real detections need a world with people.

Key facts learned:
- **Everything sim (gzserver/SITL/ns-3/DDS) must run with the command sandbox DISABLED**
  (`dangerouslyDisableSandbox`) — the sandbox kills gzserver (exit 144) and blocks DDS.
- **Run strategy for vision nodes:** system `python3` has rclpy+cv2+numpy → run
  `camera_relay`/`gcs_receiver`/`metrics_logger` via `ros2 run`. `detector` needs `ultralytics`
  (only in repo `venv`) → run with `venv/bin/python <install>/lib/uav_vision/detector` and
  `PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:<install>/uav_vision/lib/python3.10/site-packages`.
  Same pattern the Pi will use. There is no `~/yolo_env` on the host; used repo `yolov8n.pt`.
- Env for launch: `scratchpad/flat_env.sh` (sets ARDUPILOT_HOME=~/ardu_ws/src/ardupilot, GAZEBO paths
  incl. ardupilot_gazebo/build + install/multi_uav_gazebo_plugins/lib).

**Phase 2b (netns + ns-3, NEEDS sudo — user drives):** add the impaired link per `SETUP.md` §6–8.
Blocked in this agent because sudo needs a password (no TTY). Every `ip netns exec` needs root.
(Note: `micro_ros_agent` not installed — needed only for the `navsat_age` metric.)

### Phase 3 — Prepare the Pi 5
- Ubuntu 22.04 (64-bit) + ROS 2 Humble (must match host distro/DDS exactly). [VERIFY current state]
- Recreate `~/yolo_env` with `ultralytics` + `yolov8n.pt` (ARM64 wheels).
- Clone repo; `colcon build --packages-select uav_vision`.
- Sanity: run `detector.py` on a static image to confirm YOLO works on ARM.

### Phase 4 — Bring the Pi in the EASY way (no ns-3 impairment yet)
Prove the HITL compute path first: one plain wired link, same `ROS_DOMAIN_ID`, flat DDS.
Pi runs `camera_relay`+`detector` (edge) on the host's `/uav1/camera/image_raw`; host runs
`gcs_receiver`. Success = Pi detects people in the SITL feed, detections appear on host.

### Phase 5 — Insert the ns-3 impaired link (full HITL)
Replace the `uav1ns` veth in `br-uav1` with the physical path to the Pi. Pi gets
`10.42.0.11` (wireless, via host `br-uav1`→`tap-uav1`→ns-3) and `172.31.1.2` (sensor, direct).

### Phase 6 — Run the edge-vs-ground HITL comparison
Same experiment, real edge node. Collect CSVs for both modes.

## Hard problems to plan around
1. **Host has WiFi only** → use USB-Gig-E adapter(s). Raw 640×480 RGB ≈ 73 Mbps; the
   sensor link should be wired. Ideally two paths (2 adapters or VLANs) so sensor and
   wireless links are physically separate and WiFi loss doesn't contaminate measurements.
2. **Clock sync** — `metrics_logger` latency = `receipt_wall − send_wall`. Across two
   machines clocks differ → run chrony/PTP between Pi and host or latency numbers are junk.
3. **DDS discovery over a lossy link** — default multicast is fragile through bridges +
   loss. Use unicast discovery peers (CycloneDDS `Peers` / FastDDS static discovery).
4. **YOLO speed on Pi 5** — yolov8n ~a few fps on CPU; that's the point of edge-vs-ground.
   Optional accelerator (Coral/Hailo) later.

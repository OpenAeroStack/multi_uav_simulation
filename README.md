# Multi-UAV city simulation with ns-3

This repository integrates Gazebo Classic, three ArduPilot SITL vehicles, ROS 2
Humble, AP_DDS, MAVLink, and an ns-3 Wi-Fi channel. Linux network namespaces
ensure that command and telemetry traffic cannot silently bypass ns-3 through
the host loopback interface.

The primary entry point is:

```bash
./scripts/launch_city_dds.sh
```

The launcher creates the topology, builds and starts ns-3, starts Gazebo and
the three SITL instances, validates AP_DDS telemetry, starts the bridge nodes,
and finally starts `city_mission`.

> The complete runtime still requires local sudo access for namespace and TAP
> creation. Run `sudo -v` before launching so setup and cleanup do not stop at
> an interactive password prompt.

## Architecture

```text
ROOT NETWORK NAMESPACE
────────────────────────────────────────────────────────────────────────────
  scripts/launch_city_dds.sh
  Gazebo Classic 11
  ns-3 real-time simulation

                  ns-3 802.11a ad-hoc Wi-Fi
          LogDistance + Nakagami + optional random loss

 tap-gcs         tap-uav1         tap-uav2         tap-uav3
    │                │                │                │
 br-gcs           br-uav1          br-uav2          br-uav3
    │                │                │                │
 veth-gcs-host    veth-uav1-host   veth-uav2-host   veth-uav3-host
    │                │                │                │
    ▼                ▼                ▼                ▼

  gcsns             uav1             uav2             uav3
  ─────             ────             ────             ────
  wifi0 .10         wifi0 .11        wifi0 .12        wifi0 .13
  city_mission      SITL I0          SITL I1          SITL I2
  3 drone_bridge    SYSID 1          SYSID 2          SYSID 3
  3 DDS agents      TCP 5760         TCP 5770         TCP 5780
                    DDS→.10:2019     DDS→.10:2020     DDS→.10:2021

GAZEBO/SITL MANAGEMENT LINKS — do not pass through ns-3
────────────────────────────────────────────────────────────────────────────
  sim-uav1-host 172.31.1.1/30 ─── sim0 172.31.1.2/30 in uav1
  sim-uav2-host 172.31.2.1/30 ─── sim0 172.31.2.2/30 in uav2
  sim-uav3-host 172.31.3.1/30 ─── sim0 172.31.3.2/30 in uav3
```

### Traffic paths

Mission commands are forced through ns-3:

```text
city_mission
  → ROS 2 service/topic inside gcsns
  → drone_bridge
  → MAVLink TCP over gcsns/wifi0
  → ns-3
  → uavN/wifi0
  → ArduPilot SITL
```

AP_DDS telemetry is also forced through ns-3:

```text
ArduPilot AP_DDS in uavN
  → XRCE-DDS UDP to 10.42.0.10:2019/2020/2021
  → uavN/wifi0
  → ns-3
  → gcsns/wifi0
  → micro_ros_agent
  → /ap/vN/* inside gcsns
  → drone_bridge
  → /uavN/*
  → city_mission
```

Gazebo physics deliberately bypasses ns-3 and uses isolated management veths:

```text
Gazebo FDM state → SITL sim0
SITL servo output → Gazebo ArduPilot plugin
```

Root-side TAPs, bridges, and wireless veths have no IP addresses. There is no
wireless default route, fake gateway, `/32` route, or MASQUERADE rule.

## Address and port reference

### Wireless interfaces

| Endpoint | Namespace/interface | Address | MAC |
|---|---|---|---|
| GCS | `gcsns/wifi0` | `10.42.0.10/24` | `02:00:00:00:00:00` |
| UAV1 | `uav1/wifi0` | `10.42.0.11/24` | `02:00:00:00:00:01` |
| UAV2 | `uav2/wifi0` | `10.42.0.12/24` | `02:00:00:00:00:02` |
| UAV3 | `uav3/wifi0` | `10.42.0.13/24` | `02:00:00:00:00:03` |

### Vehicle ports

| UAV | MAVLink TCP | AP_DDS UDP destination | Gazebo servo input | SITL FDM input |
|---|---|---|---|---|
| UAV1 | `10.42.0.11:5760` | `10.42.0.10:2019` | `172.31.1.1:9002` | `172.31.1.2:9003` |
| UAV2 | `10.42.0.12:5770` | `10.42.0.10:2020` | `172.31.2.1:9012` | `172.31.2.2:9013` |
| UAV3 | `10.42.0.13:5780` | `10.42.0.10:2021` | `172.31.3.1:9022` | `172.31.3.2:9023` |

The Gazebo/SITL ports use UDP. No TCP or `socat` relay is used for physics.

## Requirements

- Ubuntu 22.04
- Bash
- Gazebo Classic 11
- ROS 2 Humble
- `ros-humble-gazebo-ros-pkgs`
- ArduPilot built for SITL with DDS enabled
- `ardupilot_msgs` and `micro_ros_agent` installed in `~/ardu_ws`
- ArduPilot Gazebo plugin providing `libArduPilotPlugin.so`
- ns-3 3.38 or a compatible CMake-based ns-3 release
- `rmw_cyclonedds_cpp` if CycloneDDS is selected for other experiments
- Linux tools: `iproute2`, `bridge`, `ethtool`, `ping`, `sudo`
- Python packages required by the ROS package, including `pymavlink`

Install the common Ubuntu packages:

```bash
sudo apt update
sudo apt install \
  gazebo11 libgazebo11-dev \
  ros-humble-gazebo-ros-pkgs \
  python3-colcon-common-extensions \
  iproute2 ethtool iputils-ping
```

## ArduPilot and ROS workspace

The default configuration expects:

```text
~/ardu_ws/src/ardupilot
~/ardu_ws/install/setup.bash
```

Build ArduPilot with DDS support:

```bash
cd ~/ardu_ws/src/ardupilot
git submodule update --init --recursive
./waf configure --board sitl --enable-dds
./waf copter
```

Verify the binary:

```bash
test -x ~/ardu_ws/src/ardupilot/build/sitl/bin/arducopter
```

The three parameter files configure unique clients:

```text
params/uav1_dds.parm: SYSID_THISMAV=1, DDS_UDP_PORT=2019
params/uav2_dds.parm: SYSID_THISMAV=2, DDS_UDP_PORT=2020
params/uav3_dds.parm: SYSID_THISMAV=3, DDS_UDP_PORT=2021
```

All three enable `DDS_USE_NS=1`, producing `/ap/v1`, `/ap/v2`, and `/ap/v3`.

## Build the ROS 2 package

```bash
cd /path/to/multi_uav_simulation/ros2
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
colcon build --packages-select uav_controller
source install/setup.bash
```

Verify the important executables:

```bash
ros2 pkg executables uav_controller | grep -E \
  'drone_bridge|city_mission|gazebo_ns3_position_sender'
```

## Install the ns-3 target

The repository keeps its ns-3 source under `ns3/`. Place it in the ns-3 scratch
tree before the first launch:

```bash
export NS3_ROOT="$HOME/ns-allinone-3.38/ns-3.38"
mkdir -p "$NS3_ROOT/scratch/three-uav"
cp ns3/three_uav_tapbridge_rt.cc \
  "$NS3_ROOT/scratch/three-uav/three_uav_tapbridge_rt.cc"
cp ns3/CMakeLists.txt "$NS3_ROOT/scratch/three-uav/CMakeLists.txt"

cd "$NS3_ROOT"
./ns3 build three-uav
```

The launcher resolves `$NS3_ROOT`, then known locations such as
`~/ns-allinone-3.38/ns-3.38` and `~/ns-3-dev`. It runs an incremental
`./ns3 build three-uav` before startup.

## Run the complete simulation

From the repository root:

```bash
sudo -v
./scripts/launch_city_dds.sh
```

For a non-default ns-3 checkout:

```bash
sudo -v
NS3_ROOT=/path/to/ns-3 ./scripts/launch_city_dds.sh
```

The mission starts automatically only after all readiness checks pass.

### Startup order

1. Remove stale topology.
2. Create namespaces, TAPs, bridges, wireless veths, and management veths.
3. Verify addresses, bridge membership, STP, routes, and management isolation.
4. Resolve and build the ns-3 `three-uav` target.
5. Start ns-3 in real-time mode and log to `/tmp/ns3_stdout.log`.
6. Wait for carrier/`LOWER_UP` on all four TAPs.
7. Start Gazebo with `worlds/city_3uav.world`.
8. Start three namespaced SITL processes.
9. Start three `micro_ros_agent` processes inside `gcsns`.
10. Wait for UDP ports 2019, 2020, and 2021.
11. Require active publishers and valid GPS messages on all `/ap/vN/navsat` topics.
12. Require MAVLink TCP reachability from `gcsns`.
13. Start the three `drone_bridge` nodes inside `gcsns`.
14. Recheck every prerequisite process.
15. Start `city_mission` inside `gcsns`.

Press Ctrl+C to stop. Cleanup uses saved PIDs and removes all topology objects.

## Network-only verification

Verify wireless connectivity without starting Gazebo, SITL, or ROS nodes:

```bash
sudo -v
./scripts/launch_city_dds.sh --verify-network-only
```

Prove that connectivity depends on ns-3:

```bash
sudo -v
./scripts/launch_city_dds.sh \
  --verify-network-only \
  --prove-ns3-path
```

The second mode first verifies these paths:

```text
gcsns → 10.42.0.11
gcsns → 10.42.0.12
gcsns → 10.42.0.13
```

It then stops ns-3 and requires all three pings to fail. It also prints TAP
counters, namespace routes and addresses, and bridge membership.

Validate cleanup without changing the system:

```bash
./scripts/launch_city_dds.sh --cleanup-dry-run
```

## Inspect the running system

### Namespaces and interfaces

```bash
sudo ip netns list
sudo ip netns exec gcsns ip -brief address
sudo ip netns exec uav1 ip -brief address
sudo ip netns exec uav2 ip -brief address
sudo ip netns exec uav3 ip -brief address

bridge link show master br-gcs
bridge link show master br-uav1
bridge link show master br-uav2
bridge link show master br-uav3
```

### TAP attachment

```bash
ip -brief link show tap-gcs
ip -brief link show tap-uav1
ip -brief link show tap-uav2
ip -brief link show tap-uav3

cat /sys/class/net/tap-gcs/carrier
```

Attached TAPs should show carrier `1` or the `LOWER_UP` flag.

### MAVLink listeners

```bash
sudo ip netns exec uav1 ss -ltn 'sport = :5760'
sudo ip netns exec uav2 ss -ltn 'sport = :5770'
sudo ip netns exec uav3 ss -ltn 'sport = :5780'
```

### AP_DDS agent sockets

```bash
sudo ip netns exec gcsns ss -lunp \
  '( sport = :2019 or sport = :2020 or sport = :2021 )'
```

## Inspect ROS 2 and DDS

ROS commands must run inside `gcsns` using domain 0. A convenient shell is:

```bash
sudo ip netns exec gcsns sudo -H -u "$USER" bash
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
source /path/to/multi_uav_simulation/ros2/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS2CLI_NO_DAEMON=1
```

Then inspect the namespaced AP_DDS topics:

```bash
ros2 topic info /ap/v1/navsat --verbose
ros2 topic info /ap/v2/navsat --verbose
ros2 topic info /ap/v3/navsat --verbose

ros2 topic echo /ap/v1/navsat sensor_msgs/msg/NavSatFix --once
ros2 topic echo /ap/v2/navsat sensor_msgs/msg/NavSatFix --once
ros2 topic echo /ap/v3/navsat sensor_msgs/msg/NavSatFix --once
```

Bridge application topics:

```text
/uavN/gps
/uavN/rel_alt
/uavN/battery
/uavN/mode
/uavN/armed
/uavN/goto
```

Bridge services:

```text
/uavN/arm
/uavN/disarm
/uavN/takeoff
/uavN/land
/uavN/rtl
```

## Logs

| Component | Log |
|---|---|
| ns-3 | `/tmp/ns3_stdout.log` |
| Gazebo | `/tmp/gazebo_city.log` |
| UAV1 SITL | `/tmp/multi_uav_sitl/uav1/arducopter.log` |
| UAV2 SITL | `/tmp/multi_uav_sitl/uav2/arducopter.log` |
| UAV3 SITL | `/tmp/multi_uav_sitl/uav3/arducopter.log` |
| UAV1 agent | `/tmp/micro_ros_agent_uav1.log` |
| UAV2 agent | `/tmp/micro_ros_agent_uav2.log` |
| UAV3 agent | `/tmp/micro_ros_agent_uav3.log` |
| UAV1 bridge | `/tmp/drone_bridge_uav1.log` |
| UAV2 bridge | `/tmp/drone_bridge_uav2.log` |
| UAV3 bridge | `/tmp/drone_bridge_uav3.log` |
| Mission | `/tmp/city_mission.log` |

Logs are preserved during cleanup.

## ns-3 metrics

Wi-Fi MAC/PHY traces are authoritative for TapBridge traffic. FlowMonitor is
retained only as supplementary simulated-IPv4 output because it may not observe
all externally injected frames.

### RSSI and SNR

```text
/tmp/snr_log.csv
time_s,rx_node,node_label,rssi_dbm,noise_dbm,snr_db
```

### Frame events

```text
/tmp/wifi_frame_metrics.csv
time_s,node_id,node_label,event,frame_bytes,rssi_dbm,snr_db
```

Events include `PHY_TX_FRAME`, `PHY_RX_SIGNAL`, `MAC_TX_OFFERED`, `MAC_RX_OK`,
and available MAC/PHY drop events.

### Throughput

```text
/tmp/wifi_throughput.csv
```

The file contains interval and cumulative frame counts, byte counts, drop
counts, and TX/RX throughput in Mbit/s for GCS and UAV1–UAV3.

Inspect live output:

```bash
tail -f /tmp/snr_log.csv
tail -f /tmp/wifi_frame_metrics.csv
tail -f /tmp/wifi_throughput.csv
```

## Gazebo-to-ns-3 position synchronization

The repository contains:

- `gazebo_ns3_position_sender`: reads `/gazebo/model_states` and sends ENU
  positions to `127.0.0.1:5555` at 10 Hz.
- An ns-3 host UDP receiver that schedules position updates on the simulator
  event loop.

Receiver options include:

```text
--enableExternalMobilitySync=true
--positionSyncAddress=127.0.0.1
--positionSyncPort=5555
--positionSyncPollMs=100
--positionSyncStaleMs=1000
```

Current limitation: the unified launcher does not yet start the sender or pass
`--enableExternalMobilitySync=true`. Normal integrated runs therefore use the
fixed ns-3 fallback positions:

```text
GCS  (0, 0, 0)
UAV1 (0, 0, 60)
UAV2 (50, 0, 40)
UAV3 (-50, 0, 50)
```

Standalone sender test mode:

```bash
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
ros2 run uav_controller gazebo_ns3_position_sender \
  --test --test-count 3 --test-rate-hz 10
```

## Cleanup

Cleanup runs on `EXIT`, `INT`, and `TERM`. It stops saved process trees in this
order:

1. `city_mission`
2. All bridge nodes
3. All agents
4. All SITL instances
5. Gazebo and its children
6. ns-3
7. Any launcher-owned `socat` relays

It then removes namespaces, bridges, TAPs, wireless veths, and management
veths. Missing objects are ignored, so repeated cleanup is safe.

## Troubleshooting

### Launcher stops at a sudo password prompt

```bash
sudo -v
./scripts/launch_city_dds.sh
```

For long runs, the sudo timestamp may expire before cleanup. Refresh it from a
second terminal with `sudo -v` before stopping the simulation.

### ns-3 target is missing

Copy `ns3/three_uav_tapbridge_rt.cc` and `ns3/CMakeLists.txt` into
`$NS3_ROOT/scratch/three-uav/`, then run:

```bash
cd "$NS3_ROOT"
./ns3 build three-uav
```

### TAP readiness times out

```bash
cat /tmp/ns3_stdout.log
ip -details link show tap-gcs
ip -details link show tap-uav1
```

Confirm TAP ownership matches the normal user running ns-3.

### Gazebo exits during startup

```bash
cat /tmp/gazebo_city.log
ldconfig -p | grep ArduPilotPlugin
```

Verify `GAZEBO_PLUGIN_PATH`, `GAZEBO_MODEL_PATH`, and the Gazebo ROS packages.

### AP_DDS GPS readiness fails

The launcher automatically prints the relevant SITL log, agent log, UDP socket
state, namespace routes, and namespace interfaces. Also inspect:

```bash
cat /tmp/micro_ros_agent_uav1.log
cat /tmp/multi_uav_sitl/uav1/arducopter.log
sudo ip netns exec gcsns ss -lunp
```

### MAVLink bridge cannot connect

```bash
sudo ip netns exec gcsns ping -c 2 10.42.0.11
sudo ip netns exec uav1 ss -ltn 'sport = :5760'
cat /tmp/drone_bridge_uav1.log
```

Do not connect MAVProxy to the same SITL TCP listener while `drone_bridge` owns
the connection.

## Known limitations

- A complete live integration run has not yet been completed in this execution
  environment because sudo authentication is interactive.
- Live Gazebo-to-ns-3 mobility synchronization exists in code but is not wired
  into the unified launcher.
- The optional random loss term is temporally independent, not spatially
  correlated building shadowing.
- The channel does not yet use city geometry, terrain blockage, antenna
  orientation, or UAV attitude.
- Camera ROS traffic is not routed through the simulated wireless link.
- FlowMonitor is not authoritative for TapBridge traffic; use the Wi-Fi CSVs.
- Cleanup needs a valid sudo credential to remove namespaces and interfaces.
- Paths assume ROS Humble and the default `~/ardu_ws` layout unless edited.

## Important files

| File | Responsibility |
|---|---|
| `scripts/launch_city_dds.sh` | Unified startup, readiness, mission, cleanup, verification |
| `scripts/setup_ns3_wireless_topology.sh` | Namespaces, TAPs, bridges, wireless and management veths |
| `ns3/three_uav_tapbridge_rt.cc` | Wi-Fi, channel, TapBridge, mobility, metrics |
| `worlds/city_3uav.world` | City world, WGS84 origin, vehicle inclusion |
| `models/iris_N/model.sdf` | Gazebo plugin, FDM ports, cameras |
| `params/uavN_dds.parm` | AP_DDS namespace, port, and vehicle ID |
| `ros2/uav_controller/uav_controller/drone_bridge.py` | Namespaced AP_DDS and MAVLink bridge |
| `ros2/uav_controller/uav_controller/city_mission.py` | Three-UAV city mission |
| `ros2/uav_controller/uav_controller/gazebo_ns3_position_sender.py` | Optional Gazebo ENU sender |

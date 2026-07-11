# Setup Guide — Multi-UAV Simulation with Obstacle-Loss RF Channel

This guide takes a fresh machine to a working setup of the full framework:
**Gazebo** (small-city world) + **ArduPilot SITL** drones + the **obstacle
raycast plugin** + the **NS-3** simulated Wi-Fi channel that applies
distance-, obstacle-, and fading-based signal loss between the drones.

Follow it top to bottom. Every command is copy-pasteable. If something fails,
jump to [Troubleshooting](#10-troubleshooting) — it lists the exact errors this
setup can produce and the one-line fix for each.

> For *how the system works* (data flow, equations, threads), see
> [`architecture.md`](architecture.md). This file is only about **installing and
> running** it.

---

## 0. What you will end up with

```
Gazebo (small_city_base.world)                         NS-3 process
  ├─ obstacle raycast plugin  ──/link_obstacle_loss──▶  applies per-link dB loss
  └─ gazebo_ros_state plugin  ──/gazebo/model_states─┐   + distance path loss
                                                     │   + Nakagami fading
  world_pos_publisher.py  ◀──────────────────────────┘   ──▶ /ns3_link_rssi
        └──/uav_world_positions──▶ NS-3 (moves its UAV nodes)
ArduPilot SITL ×3  ◀── MAVLink ── multi_drone_mission_new.py (flies the drones)
```

Tested on **Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 + NS-3 3.38**.

---

## 1. Prerequisites

Install these first. The **bold** ones are large external projects with their
own install docs — links given; don't skip them.

| Component | Why | Install |
|---|---|---|
| Ubuntu 22.04 | base OS | — |
| **ROS 2 Humble** | messaging + gazebo_ros plugins | https://docs.ros.org/en/humble/Installation.html |
| **Gazebo Classic 11** | physics/world simulator | `sudo apt install gazebo libgazebo-dev` |
| **NS-3 3.38** | network simulator | https://www.nsnam.org/releases/ (build from source) |
| **ArduPilot SITL** (ArduCopter) | drone autopilot firmware | https://ardupilot.org/dev/docs/building-setup-linux.html |
| **ardupilot_gazebo** (Classic) | provides `libArduPilotPlugin.so` used by the iris models | https://github.com/ArduPilot/ardupilot_gazebo (gazebo-classic branch) |

Then the apt/pip packages this framework needs directly:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \   # gazebo_ros, gazebo_dev, libgazebo_ros_state.so, gazebo_msgs
  python3-colcon-common-extensions \
  libxml2-utils \                # xmllint (world file validation)
  iperf3 iproute2 bridge-utils \ # network test + TAP/netns tooling
  linux-tools-common linux-tools-generic   # cpupower (optional real-time tuning)
pip3 install pymavlink            # used by the mission script
```

**Sanity check** before continuing:

```bash
source /opt/ros/humble/setup.bash
gazebo --version | head -1                       # Gazebo 11.x
ros2 pkg prefix gazebo_ros >/dev/null && echo "gazebo_ros OK"
ls /opt/ros/humble/lib/libgazebo_ros_state.so    # must exist
ls <NS3>/build/                                  # your built NS-3 tree
ls ~/ardupilot/build/sitl/bin/arducopter         # your built SITL binary
```
Replace `<NS3>` with your NS-3 root (e.g. `~/ns-allinone-3.38/ns-3.38`).

---

## 2. Clone the two repositories

This framework is **two** repos: the simulation package, and the city world.

```bash
# 1) the simulation package — put it in a colcon workspace's src/
mkdir -p ~/multi-uav-workspace/src
cd ~/multi-uav-workspace/src
git clone https://github.com/OpenAeroStack/multi_uav_simulation.git

# 2) the small-city world — clone into your HOME (the launch defaults to $HOME)
cd ~
git clone https://github.com/OpenAeroStack/small_city_gazebo_world.git
```

> The launch script defaults `SMALL_CITY_DIR="$HOME/small_city_gazebo_world"`.
> If you clone it elsewhere, edit that one line in
> `launch/launch_multi_uav_new.sh`.

---

## 3. Build the Gazebo obstacle plugin

The plugin is the ament package `multi_uav_gazebo_plugins` (source dir
`gazebo_plugins/`). Build it from the workspace root:

```bash
cd ~/multi-uav-workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select multi_uav_gazebo_plugins
```

Success produces:
```
install/multi_uav_gazebo_plugins/lib/libgazebo_obstacle_plugin.so
```

Verify the material logic is compiled in:
```bash
strings install/multi_uav_gazebo_plugins/lib/libgazebo_obstacle_plugin.so \
  | grep -E '^(noloss|foliage|vehicle|concrete)$'
```

> The `ros2/uav_controller` package is **optional** — it depends on
> `ardupilot_msgs` and is only needed for the DDS mission variants. The core
> obstacle-loss flow does not require it (the position relay is a standalone
> script and the mission uses pymavlink).

---

## 4. Install and build the NS-3 scenario

The NS-3 sources live in `ns3/`. Copy that folder into your NS-3 tree as a
**scratch subdirectory**, then build it. NS-3 must be **configured with a ROS 2
environment sourced**, because the scenario links against `rclcpp`.

```bash
# copy the scenario into NS-3's scratch/ (as its own subfolder)
cp -r ~/multi-uav-workspace/src/multi_uav_simulation/ns3 \
      <NS3>/scratch/multi_uav_simulation

# configure + build (ROS must be sourced so find_package(rclcpp) works)
cd <NS3>
source /opt/ros/humble/setup.bash
./ns3 configure
./ns3 build three_uav_tapbridge_obstacle_loss
```

Success produces:
```
<NS3>/build/scratch/multi_uav_simulation/ns3.38-three_uav_tapbridge_obstacle_loss-default
```

> The bundled `ns3/CMakeLists.txt` builds exactly this target (`build_exec`
> with `three_uav_tapbridge_obstacle_loss.cc` + `dynamic_obstacle_loss_model.cc`).
> No hand-editing needed.

---

## 5. Configure the environment

Edit **one** value in `setup.sh` — your ArduPilot path:

```bash
# ~/multi-uav-workspace/src/multi_uav_simulation/setup.sh
export ARDUPILOT_HOME="$HOME/ardupilot"     # <-- set to your ardupilot tree
```

`setup.sh` (sourced automatically by the launch script) sets
`GAZEBO_MODEL_PATH` (for the iris drone models) and `GAZEBO_PLUGIN_PATH`. The
launch script additionally adds the small-city model path and the built plugin.

You also need `libArduPilotPlugin.so` (from **ardupilot_gazebo**) on the Gazebo
plugin path. Either `sudo make install` it, or add its build dir:
```bash
export GAZEBO_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GAZEBO_PLUGIN_PATH
```

---

## 6. Network namespaces + TAP devices (for NS-3)

NS-3 bridges **real** Linux traffic into the simulated Wi-Fi via TAP devices.
These must exist **before** NS-3 starts. Create them (needs sudo; run once per
boot):

```bash
sudo bash ~/multi-uav-workspace/src/multi_uav_simulation/scripts/setup_netns_tap.sh
```

This creates namespaces `uav1ns/2ns/3ns` (+ `gcsns`) with IPs
`10.42.0.11/12/13` and user-owned taps `tap-uav1/2/3`. Verify:
```bash
ip link show tap-uav1     # must exist; NS-3 aborts with "Operation not permitted" if missing
```

To tear them down later: `sudo bash scripts/cleanup_netns_tap.sh`.

---

## 7. Run the full stack

Use four terminals. Dependency order is strict:
**TAP (§6) → Gazebo+SITL+relay → NS-3 → mission.**

```bash
# ── Terminal 1 — Gazebo (small-city world) + 3× SITL + position relay ──
cd ~/multi-uav-workspace/src/multi_uav_simulation
bash launch/launch_multi_uav_new.sh
#   wait for "All 3 SITL instances running"
#   check /tmp/world_pos_publisher.log has NO "No /model_states" warning

# ── Terminal 2 — NS-3 RF channel (taps must exist; Gazebo/ROS must be up) ──
source /opt/ros/humble/setup.bash
<NS3>/build/scratch/multi_uav_simulation/ns3.38-three_uav_tapbridge_obstacle_loss-default \
  --tapBase=tap-uav
#   taps go LOWER_UP; it starts publishing /ns3_link_rssi

# ── Terminal 3 — fly the mission ──
source /opt/ros/humble/setup.bash
python3 ~/multi-uav-workspace/src/multi_uav_simulation/scripts/multi_drone_mission_new.py

# ── Terminal 4 (optional) — prove the data plane through the radio ──
bash ~/multi-uav-workspace/src/multi_uav_simulation/scripts/iperf3_channel_test.sh

# ── Optional real-time tuning (pins CPUs, sets performance governor) ──
bash ~/multi-uav-workspace/src/multi_uav_simulation/scripts/pin_realtime.sh
```

---

## 8. Verification

With the stack up (Gazebo + NS-3), in a ROS-sourced terminal:

```bash
# drones present in Gazebo ground truth:
ros2 topic echo --once /gazebo/model_states | grep -c iris_          # expect 3

# NS-3 is reporting link RSSI (expect ~ -60 dBm per link at the start formation):
ros2 topic echo --once /ns3_link_rssi

# obstacle losses reported per drone pair (values differ by material, not all 15):
ros2 topic echo --once /link_obstacle_loss
```

**What "correct" looks like:** at the initial ~50 m spacing each link RSSI is
around **−60 dBm** (well above the −82 dBm sensitivity). Fly a drone behind a
building and its link RSSI drops sharply; a link crossing only trees/`noloss_`
props barely changes.

---

## 9. Obstacle material tags (reference)

The plugin picks per-obstacle loss from a **keyword in the model's `<name>`** in
the world file, so materials are author-controlled. Current map
(`gazebo_obstacle_plugin.cc :: ComputeObstacleLoss`):

| Keyword in `<name>` | Entry loss `L_e` | Meaning |
|---|---|---|
| `noloss` | **0 dB** (ray steps past it) | thin street furniture: poles, signs, hydrants, dumpster, postbox |
| `glass` | 4 dB | windows |
| `foliage` | 5 dB | trees |
| `wood` | 8 dB | wooden poles, pier |
| `vehicle` | 12 dB | cars (hollow metal shell) |
| `concrete` | 15 dB (**default**) | buildings |
| `metal` | 20 dB | solid steel structures |

Total loss for a blocked link = `L_e + 0.5 dB/m × material_thickness`. To retag
an object, just rename it in the world (e.g. `concrete_house_1_5`) — no code
change needed. To add a new material, add one `if (name.find("x")) L_e = …`
line and **rebuild the plugin** (§3).

`small_city_base.world` ships already tagged (78 `noloss_`, 61 `foliage_`,
21 `concrete_`, 17 `vehicle_`, 3 `metal_`, 1 `wood_`).

---

## 10. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| NS-3: `CreateTap(): Could not allocate tap device — Operation not permitted` | TAP devices don't exist (gone after reboot) | Re-run §6 `setup_netns_tap.sh`. Check first: `ip link show tap-uav1`. |
| Gazebo loads an **empty** world / `Falling back on worlds/empty.world` | XML parse error in the `.world` | `xmllint --noout <world>` prints the exact bad line. (Add `xmllint --noout "$WORLD_PATH" \|\| exit 1` to the launch to catch it early.) |
| `Unable to resolve uri[model://…]` / missing buildings | Gazebo can't find the city models | Ensure `~/small_city_gazebo_world/models` is on `GAZEBO_MODEL_PATH` (the launch adds it; if you moved the repo, fix `SMALL_CITY_DIR`). |
| Plugin build: `fatal error: gazebo_plugins/…hh: No such file` | wrong include path | Include must be relative (`#include "gazebo_plugins/gazebo_obstacle_plugin.hh"`); rebuild with `colcon build`. |
| Drones spawn but mission **hangs at "waiting for EKF"** / unstable takeoff | world physics too coarse / not lockstep-friendly | World must use `real_time_update_rate=-1` and `max_step_size=0.001` (already set in `small_city_base.world`). |
| `Ctrl-C` doesn't stop NS-3 | SIGTERM doesn't interrupt `Simulator::Run()` | Stop it with `kill -9 <pid>` (find with `pgrep -f ns3.38-three`). |
| `ros2 topic list` shows nothing (not even `/rosout`) | stale ROS 2 CLI daemon | `ros2 daemon stop && ros2 daemon start`, or add `--no-daemon`. |
| iperf3 shows ~0 throughput / ~100 % loss even with a clear link | NS-3 real-time engine can't keep up above ~1.4 Mbit/s on this host | Keep offered load low (`iperf3 -b 500K`); run `pin_realtime.sh`; see `architecture.md` §10–11. |
| All obstacles report the same ~15 dB | model names lack a material keyword | Retag names in the world (§9) and rebuild is **not** needed for the world — just relaunch Gazebo. |
| No `libArduPilotPlugin.so` / SITL never connects to Gazebo | ardupilot_gazebo not on plugin path | Install `ardupilot_gazebo` and add its build dir to `GAZEBO_PLUGIN_PATH` (§5). |

---

## Repository layout (what you cloned)

```
multi_uav_simulation/
├── gazebo_plugins/        # obstacle raycast plugin (build with colcon) → §3
│   ├── src/gazebo_obstacle_plugin.cc
│   ├── include/gazebo_plugins/gazebo_obstacle_plugin.hh
│   ├── CMakeLists.txt  package.xml
├── ns3/                   # NS-3 scenario (copy into scratch/) → §4
│   ├── three_uav_tapbridge_obstacle_loss.cc
│   ├── dynamic_obstacle_loss_model.{cc,hh}
│   └── CMakeLists.txt
├── models/                # iris_1/2/3 drone models (Gazebo)
├── worlds/                # multi_uav_plugin.world (simple test world)
├── scripts/
│   ├── setup_netns_tap.sh / cleanup_netns_tap.sh   # §6
│   ├── world_pos_publisher.py                      # Gazebo→NS-3 position relay
│   ├── multi_drone_mission_new.py                  # the mission
│   ├── iperf3_channel_test.sh                       # data-plane test
│   └── pin_realtime.sh                             # CPU pinning / governor
├── launch/launch_multi_uav_new.sh                  # Gazebo + SITL + relay → §7
├── setup.sh               # env (edit ARDUPILOT_HOME) → §5
├── architecture.md        # how it all works
└── SETUP.md               # this file
```

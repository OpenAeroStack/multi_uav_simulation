# Setup Guide — Multi-UAV Simulation with Obstacle-Loss RF Channel

This guide takes a fresh machine to a working setup of the full framework:
**Gazebo** (`small_city_base.world`) + **ArduPilot SITL** drones + the
**obstacle raycast plugin** + the **NS-3** simulated Wi-Fi channel
(`three_uav_tapbridge_integrated`) that applies distance-, obstacle- and
fading-based signal loss across the **four** nodes — a ground control station
and three UAVs.

Follow it top to bottom. Every command is copy-pasteable. If something fails,
jump to [Troubleshooting](#11-troubleshooting) — it lists the exact errors this
setup can produce and the one-line fix for each.

> For *how the system works* (data flow, equations, threads), see
> [`architecture.md`](architecture.md). This file is only about **installing and
> running** it.

---

## 0. What you will end up with

```
Gazebo (small_city_base.world)                        NS-3 (integrated scenario)
  ├─ obstacle raycast plugin ──/link_obstacle_loss──▶  per-link obstacle dB
  │    (6 links: 3 GCS↔UAV + 3 UAV↔UAV)                + log-distance path loss
  └─ gazebo_ros_state plugin ──/gazebo/model_states─┐  + Nakagami fading (m
                                                    │    switches on LoS state)
  world_pos_publisher.py  ◀─────────────────────────┘  ──▶ /ns3_link_rssi
        └──/uav_world_positions──▶ NS-3 (moves nodes)  ──▶ /ns3_link_snr
ArduPilot SITL ×3  ◀── MAVLink ── multi_drone_mission_new.py (flies the drones)
```

Node numbering is **one convention everywhere** — Gazebo, ROS topics and NS-3
node ids are identical, with no offset applied anywhere:

| id | Node | TAP | netns | IP |
|---|---|---|---|---|
| 0 | GCS (`<model name="gcs">`, static) | `tap-gcs` | `gcsns` | 10.42.0.10 |
| 1 | UAV1 (`iris_1`) | `tap-uav1` | `uav1ns` | 10.42.0.11 |
| 2 | UAV2 (`iris_2`) | `tap-uav2` | `uav2ns` | 10.42.0.12 |
| 3 | UAV3 (`iris_3`) | `tap-uav3` | `uav3ns` | 10.42.0.13 |

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
Replace `<NS3>` with your NS-3 root (on this machine: `~/ns-3.3`, which is a
3.38 tree — check with `cat <NS3>/VERSION`).

---

## 2. Clone the repository

**One** repo now. The small-city assets (terrain, buildings, trees, vehicles,
street furniture) and the iris drone models are vendored under `models/`, so
the external `small_city_gazebo_world` repo is no longer needed.

```bash
mkdir -p ~/multi-uav-workspace/src
cd ~/multi-uav-workspace/src
git clone https://github.com/OpenAeroStack/multi_uav_simulation.git
```

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
install/multi_uav_gazebo_plugins/lib/libobstacle_raycast_plugin.so
```

Verify the material logic is compiled in:
```bash
strings install/multi_uav_gazebo_plugins/lib/libobstacle_raycast_plugin.so \
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
./ns3 build three_uav_tapbridge_integrated
```

Success produces:
```
<NS3>/build/scratch/multi_uav_simulation/ns3.38-three_uav_tapbridge_integrated-default
```

> **The scratch copy is a copy, not a link.** Editing `ns3/*.cc` in the
> workspace has no effect until you re-copy it into `<NS3>/scratch/` and
> rebuild. Keep the two in sync — the workspace copy is the one under version
> control.

> `ns3/CMakeLists.txt` also builds `three_uav_tapbridge_obstacle_loss`, the
> 3-node predecessor without a GCS node. Everything in this guide uses the
> integrated target; the older one is kept only because earlier notes and
> validation runs refer to it.

Useful flags of the integrated binary (`--PrintHelp` lists all of them):

| Flag | Default | Meaning |
|---|---|---|
| `--tap0 … --tap3` | `tap-gcs`, `tap-uav1..3` | TAP device per node |
| `--enableTap` | `true` | install TapBridge (needs the TAPs from §6) |
| `--standalone` | `false` | run without ROS, Gauss-Markov mobility (model debugging) |
| `--gcsX --gcsY --gcsZ` | `-24, 0, 0` | initial GCS position — already matches `small_city_base.world` |
| `--simTime` | `0` | 0 = run until killed |
| `--mLos --mNlos` | `3`, `1` | Nakagami shape, clear vs blocked |
| `--emaAlpha` | `0.3` | smoothing on incoming obstacle loss |
| `--blockThreshDb --clearThreshDb` | `3`, `1` | LoS/NLoS hysteresis window |
| `--csvPath` | off | per-link validation CSV |
| `--snrLogFile` | off | per-packet SNR CSV (`MonitorSnifferRx`) |
| `--enableNetAnim --animFile` | off | NetAnim XML |
| `--rngRun` | `1` | RNG run number; change it or repeated runs draw identical fading |

---

## 5. Configure the environment

Edit **one** value in `setup.sh` — your ArduPilot path:

```bash
# ~/multi-uav-workspace/src/multi_uav_simulation/setup.sh
export ARDUPILOT_HOME="$HOME/ardupilot"     # <-- set to your ardupilot tree
```

`setup.sh` (sourced automatically by the launch script) sets
`GAZEBO_MODEL_PATH` to this package's `models/` dir and `GAZEBO_PLUGIN_PATH`.
The launch script additionally adds the built obstacle plugin from the colcon
install tree.

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

This creates four namespaces — `gcsns`, `uav1ns`, `uav2ns`, `uav3ns` — with IPs
`10.42.0.10/.11/.12/.13`, each on its **own** bridge (`br-gcs`, `br-uav1…`)
with a user-owned tap (`tap-gcs`, `tap-uav1…`). One bridge per node is what
makes NS-3 the only layer-2 path between namespaces; §9 verifies that.

```bash
ip link show tap-gcs      # must exist; NS-3 aborts with "Operation not permitted" if missing
```

To tear it all down later:
```bash
bash ~/multi-uav-workspace/src/multi_uav_simulation/scripts/cleanup_nestns_tap.sh
```
(Yes, the filename is misspelled `nestns`.)

---

## 7. Run the full stack

Use four terminals. Dependency order is strict:
**TAP (§6) → Gazebo+SITL+relay → NS-3 → mission.**

```bash
# ── Terminal 1 — Gazebo (small_city_base.world) + 3× SITL + position relay ──
cd ~/multi-uav-workspace/src/multi_uav_simulation
bash launch/launch_multi_uav_new.sh
#   wait for "All 3 SITL instances running"
#   check /tmp/world_pos_publisher.log: expect "GCS enabled: model 'gcs' -> id 0"
#   and NO "No /model_states" warning

# ── Terminal 2 — NS-3 RF channel (taps must exist; Gazebo/ROS must be up) ──
source /opt/ros/humble/setup.bash
~/ns-3.3/build/scratch/multi_uav_simulation/ns3.38-three_uav_tapbridge_integrated-default
#   default tap names already match §6, and the default GCS position already
#   matches small_city_base.world -- no flags needed
#   taps go LOWER_UP; it starts publishing /ns3_link_rssi and /ns3_link_snr
#   at t=10 s it prints the integration check -- read it (§8)

# ── Terminal 3 — fly the mission ──
source /opt/ros/humble/setup.bash
python3 ~/multi-uav-workspace/src/multi_uav_simulation/scripts/multi_drone_mission_new.py

# ── Terminal 4 (optional) — record the live channel, passively ──
source /opt/ros/humble/setup.bash
python3 ~/multi-uav-workspace/src/multi_uav_simulation/scripts/test_scripts/record_live_links.py \
  --tag mission1

# ── Optional real-time tuning (pins CPUs, sets performance governor) ──
bash ~/multi-uav-workspace/src/multi_uav_simulation/scripts/test_scripts/pin_realtime.sh
```

---

## 8. Verification

With the stack up (Gazebo + NS-3), in a ROS-sourced terminal:

```bash
# drones + ground station present in Gazebo ground truth:
ros2 topic echo --once /gazebo/model_states | grep -cE 'iris_|gcs'   # expect 4

# positions being relayed (payload starts with 0.0 = the GCS):
ros2 topic echo --once /uav_world_positions

# NS-3 is reporting link RSSI and SNR for all 6 links (18 floats each):
ros2 topic echo --once /ns3_link_rssi
ros2 topic echo --once /ns3_link_snr

# obstacle losses reported per pair (6 links):
ros2 topic echo --once /link_obstacle_loss
```

**Read the NS-3 integration check.** At t≈10 s the scenario prints either

```
[integration check] OK: positions for all 4 nodes and obstacle reports for all
6 links have been received.
```

or an `***** INCOMPLETE FEED *****` block naming exactly which nodes or links
were never fed. This check exists because every way this co-simulation breaks
looks the same from outside — a missing GCS feed leaves node 0 parked at its
CLI default with permanently-clear links, which is indistinguishable from a
correctly-working unobstructed run.

**What "correct" looks like:** before the drones move, the three UAVs sit in a
50 m equilateral triangle at 20 m altitude and the GCS is at (−24, 0, 2.9), so
with 20 dBm Tx and `Lref = 46.73 dB @ 1 m` the UAV↔UAV links read about
**−61 dBm** (SNR ≈ 33 dB against the −94 dBm noise floor) and the GCS links
span 31–77 m, i.e. roughly **−57 to −64 dBm**. Every value carries a fading
draw on top, so expect a few dB of jitter. Fly a drone behind a tagged obstacle
and its link RSSI drops by the obstacle's loss while its fading regime switches
from m=3 to m=1.

> **In `small_city_base.world` today, obstacle loss is always 0 dB.** See §10 —
> the world has no material-tagged models yet, so the links exercise path loss
> and LoS fading only. For a working obstacle demo use
> `worlds/multi_uav_plugin.world`, which has a 4 dB `glass_wall` between
> `iris_1` and `iris_2`. That world puts the GCS at (0, 6, 0), so add
> `--gcsX=0 --gcsY=6` to the NS-3 command.

---

## 9. Test and validation scripts (`scripts/test_scripts/`)

All of these write to `test_logs/`, which is **git-ignored** — it is
regenerable evidence, not source. `test_logs/README.md` documents the output
formats and the known gaps in the current evidence.

| Script | Needs Gazebo? | What it does |
|---|---|---|
| `verify_datapath.sh` | **must be stopped** | Tier 0: proves packets between namespaces actually cross the NS-3 channel |
| `run_channel_validation.py` | **must be stopped** | drives NS-3 through 4 scenarios, checks the channel maths, writes a pass/fail summary |
| `record_live_links.py` | yes (passive) | subscribes only; records a live flight to CSV |
| `iperf3_channel_test.sh` | no (NS-3 running) | pushes real UDP traffic and shows it degrade when an obstacle is injected |
| `sweep_test.py` | yes | sweeps UAV2 past a wall and prints the reported obstacle loss |
| `pin_realtime.sh` | after launch | CPU pinning + performance governor |

**Two of these are drivers, not observers.** `verify_datapath.sh` and
`run_channel_validation.py` launch their own NS-3 and publish their own
positions and obstacle reports. Run either while Gazebo is up and every node
position will alternate between the real drone and the script's fixed
coordinates at 10 Hz, silently corrupting both. Only `record_live_links.py` is
safe to start and stop mid-flight.

```bash
# Tier 0 — does traffic really traverse the channel? (needs sudo; ~2 min)
bash scripts/test_scripts/verify_datapath.sh
#   the load-bearing step is PHASE 2, the negative control: with NS-3 stopped
#   the ping MUST fail completely. If it succeeds a bypass path exists and no
#   later measurement means anything, so the script aborts there.

# Channel-model validation — ~6 min, or --quick for ~3
python3 scripts/test_scripts/run_channel_validation.py
python3 scripts/test_scripts/run_channel_validation.py --quick

# Re-check an existing CSV without launching or publishing anything
# (safe alongside a live run)
python3 scripts/test_scripts/run_channel_validation.py --analyse-only test_logs/live_flight.csv
```

To have NS-3 log its own per-link data during a live flight, pass
`--csvPath=test_logs/live_flight.csv --snrLogFile=test_logs/live_snr.csv` in
Terminal 2 and analyse it afterwards with `--analyse-only`.

---

## 10. Obstacle material tags (reference)

The plugin picks per-obstacle loss from a **keyword in the model's `<name>`** in
the world file, so materials are author-controlled. Current map
(`obstacle_raycast_plugin.cc :: ComputeObstacleLoss`):

| Keyword in `<name>` | Entry loss `L_e` | Meaning |
|---|---|---|
| `glass` | 4 dB | windows |
| `foliage` | 5 dB | trees |
| `wood` | 8 dB | wooden poles, pier |
| `vehicle` | 12 dB | cars (hollow metal shell) |
| `concrete` | 15 dB | buildings |
| `metal` | 20 dB | solid steel structures |
| `noloss` | 0 dB — ray steps past it | thin street furniture: poles, signs, hydrants, postbox |
| *(no keyword)* | **0 dB** | untagged models are treated as RF-transparent |

Total loss for a blocked link = `L_e + 0.5 dB/m × material_thickness`, where
thickness is the true entry-face-to-exit-face depth found by a second, backward
ray (clamped at 20 m). To retag an object, just rename it in the world (e.g.
`concrete_house_1_5`) — no code change needed. To add a new material, add one
`if (name.find("x")) { L_e = …; material_known = true; }` line and **rebuild the
plugin** (§3).

Also never a source of loss, regardless of name: the ground plane, any model
matching `uav_prefix` (a drone body), and the `gcs` structure — a link must not
be blocked by its own antenna mast.

> ⚠️ **The default is 0 dB, not concrete.** An untagged model attenuates
> nothing. This is a deliberate change from the earlier behaviour, where an
> unrecognised obstacle defaulted to 15 dB — but it means a mistyped keyword
> fails silently as a perfectly clear link.
>
> ⚠️ **`small_city_base.world` is currently untagged.** It holds 44 models — 38
> sidewalks, a gas station, an asphalt plane, the ground plane, the GCS and the
> 3 drones — and **not one carries a material keyword**, so every link in that
> world reports exactly 0 dB obstacle loss, always. The city assets that would
> block links (buildings, trees, vehicles) are vendored under `models/` but not
> yet instantiated in the world. Add them with a keyword in the `<model name=…>`
> to make the obstacle path do anything.

---

## 11. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| NS-3: `CreateTap(): Could not allocate tap device — Operation not permitted` | TAP devices don't exist (gone after reboot) | Re-run §6 `setup_netns_tap.sh`. Check first: `ip link show tap-gcs`. |
| NS-3 prints `***** INCOMPLETE FEED *****` naming `GCS(0)` | the GCS is not being fed | Check `<model name="gcs">` is in the world, `<gcs_enabled>true</gcs_enabled>` in the plugin block, and that `world_pos_publisher.py` logged "GCS enabled". |
| NS-3 prints `INCOMPLETE FEED` naming links `(0,1) (0,2) (0,3)` | ray-caster is only covering UAV↔UAV pairs | Same `<gcs_enabled>` block — without it the three GCS links are modelled as permanently clear, the most optimistic case for the most easily blocked link. |
| Every link reports 0 dB obstacle loss forever | expected in `small_city_base.world` — nothing is material-tagged | §10. Use `worlds/multi_uav_plugin.world` for an obstacle demo, or tag models in the city world. |
| Positions look frozen but the topic is live | Gazebo exited; the relay used to rebroadcast its last frame forever | Already guarded: the relay goes silent after `stale_after` (2 s) and logs an error. If you see that error, restart Gazebo. |
| Gazebo loads an **empty** world / `Falling back on worlds/empty.world` | XML parse error in the `.world` | `xmllint --noout <world>` prints the exact bad line. |
| `Unable to resolve uri[model://…]` | Gazebo can't find the vendored models | `GAZEBO_MODEL_PATH` must include this package's `models/` dir — `setup.sh` adds it, so source it (the launch script does). |
| Plugin build: `fatal error: gazebo_plugins/…hh: No such file` | wrong include path | Include must be relative (`#include "gazebo_plugins/obstacle_raycast_plugin.hh"`); rebuild with `colcon build`. |
| Edited `ns3/*.cc` but nothing changed | you edited the workspace copy, not the scratch copy | Re-copy `ns3/` into `<NS3>/scratch/multi_uav_simulation` and rebuild (§4). |
| Drones spawn but mission **hangs at "waiting for EKF"** / unstable takeoff | world physics too coarse / not lockstep-friendly | World must use `real_time_update_rate=-1` and `max_step_size=0.001` (already set in `small_city_base.world`). |
| `Ctrl-C` doesn't stop NS-3 | SIGTERM doesn't interrupt `Simulator::Run()` | `kill -9 $(pgrep -f ns3.38-three_uav)`. |
| `ros2 topic list` shows nothing (not even `/rosout`) | stale ROS 2 CLI daemon | `ros2 daemon stop && ros2 daemon start`, or add `--no-daemon`. |
| iperf3 shows ~0 throughput / ~100 % loss even with a clear link | NS-3 real-time engine can't keep up above ~1.4 Mbit/s on this host | Keep offered load low (`iperf3 -b 500K`); run `pin_realtime.sh`; see `architecture.md` §10–11. |
| A validation run reports all-green but NS-3 died mid-run | partial data can satisfy every per-link check | `run_channel_validation.py` checks process exit status first, per scenario — read that row before the rest. |
| Positions alternate between real and fixed coordinates | a driver script was run against a live stack | Stop Gazebo before `verify_datapath.sh` / `run_channel_validation.py` (§9). |
| No `libArduPilotPlugin.so` / SITL never connects to Gazebo | ardupilot_gazebo not on plugin path | Install `ardupilot_gazebo` and add its build dir to `GAZEBO_PLUGIN_PATH` (§5). |

---

## Repository layout (what you cloned)

```
multi_uav_simulation/
├── gazebo_plugins/        # obstacle raycast plugin (build with colcon) → §3
│   ├── src/obstacle_raycast_plugin.cc
│   ├── include/gazebo_plugins/obstacle_raycast_plugin.hh
│   ├── CMakeLists.txt  package.xml
├── ns3/                   # NS-3 scenarios (copy into scratch/) → §4
│   ├── three_uav_tapbridge_integrated.cc     # ← the current one (GCS + 3 UAVs)
│   ├── three_uav_tapbridge_obstacle_loss.cc  # 3-node predecessor
│   ├── dynamic_obstacle_loss_model.{cc,hh}
│   ├── plot_ns3_validation.py
│   └── CMakeLists.txt     # builds both targets
├── models/                # iris drones + all vendored small-city assets
├── worlds/
│   ├── small_city_base.world      # the world this guide uses → §7
│   └── multi_uav_plugin.world     # single glass wall, for obstacle demos
├── scripts/
│   ├── setup_netns_tap.sh / cleanup_nestns_tap.sh   # §6
│   ├── world_pos_publisher.py                       # Gazebo→NS-3 position relay
│   ├── multi_drone_mission_new.py                   # the mission
│   └── test_scripts/                                # → §9
│       ├── verify_datapath.sh          # Tier 0: traffic really crosses NS-3
│       ├── run_channel_validation.py   # channel-model validation harness
│       ├── record_live_links.py        # passive live-flight recorder
│       ├── iperf3_channel_test.sh      # data-plane test
│       ├── sweep_test.py               # obstacle-loss sweep past a wall
│       └── pin_realtime.sh             # CPU pinning / governor
├── test_logs/             # generated evidence (git-ignored); see its README
├── launch/launch_multi_uav_new.sh                   # Gazebo + SITL + relay → §7
├── setup.sh               # env (edit ARDUPILOT_HOME) → §5
├── architecture.md        # how it all works
└── SETUP.md               # this file
```

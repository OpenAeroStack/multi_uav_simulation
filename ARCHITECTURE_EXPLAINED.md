# Architecture Explained — `multi_uav_sim`, branch `integrated-ns3`

This document was produced by reading the actual files in this repository
(no external memory or assumptions) as of the `integrated-ns3` branch,
latest commit `dfc5558` ("Updated SETUP", author dice_apps). Every claim
below is backed by a specific file and line; where something could not be
confirmed by reading the code, that is stated explicitly instead of guessed.

---

## SECTION 1 — Big picture

**In plain language:** this project is a **co-simulation** of a small
drone fleet. Three separate programs run at the same time and talk to each
other:

1. **Gazebo** — a 3-D physics simulator. It renders a city world
   (`worlds/small_city_base.world`), simulates the physical drone bodies
   (the `iris_1`/`iris_2`/`iris_3` models) and a ground station model
   (`gcs`), and runs two custom plugins (state publisher + obstacle
   ray-caster, see Section 7).
2. **ArduPilot SITL** — the actual autopilot firmware ("Software In The
   Loop"), one process per drone, doing real flight-control math
   (stabilization, GPS hold, navigation) against Gazebo's physics instead
   of a real flight controller board.
3. **ns-3** — a discrete-event Wi-Fi network simulator. It does **not**
   simulate physics; it takes the drones' real (x,y,z) positions as they
   come from Gazebo and uses them to compute a realistic wireless link
   (path loss + obstacle shadowing + Nakagami fading) between four nodes:
   GCS + 3 UAVs.

These three talk over **ROS 2** topics (a publish/subscribe message bus)
and, for actual packet traffic, over Linux **TAP virtual network devices**
that ns-3's `TapBridge` module attaches to.

**One complete run, end to end** (this is the `integrated-ns3` branch's
intended pipeline — see Section 2 for why this specific one and not the
other launch scripts):

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              GAZEBO PROCESS                                 │
│  ┌───────────────┐   physics    ┌──────────────────────────────────────┐  │
│  │ iris_1/2/3     │◄────────────┤  gazebo_ros_state plugin (.so)        │  │
│  │ + gcs models   │   (ODE)     │  → publishes /gazebo/model_states     │  │
│  └───────┬────────┘             │    (ground-truth poses, 20 Hz)        │  │
│          │ ArduPilot            └──────────────┬─────────────────────────┘  │
│          │ <-> Gazebo lockstep                  │                           │
│          │ (UDP, physics FDM)                   │                           │
│          │                      ┌──────────────▼─────────────────────────┐  │
│          │                      │  obstacle_raycast_plugin (.so)         │  │
│          │                      │  embeds its own rclcpp::Node           │  │
│          │                      │  sub /uav_world_positions              │  │
│          │                      │  pub /link_obstacle_loss (per-pair dB) │  │
│          │                      └──────────────┬─────────────────────────┘  │
└──────────┼──────────────────────────────────────┼───────────────────────────┘
           │                                       │
  ┌────────▼─────────┐                             │
  │ ArduPilot SITL ×3│                              │
  │ (arducopter bin) │                              │
  └────────┬─────────┘                              │
           │ MAVLink (TCP 5760/5770/5780)            │
           │                                          │
  ┌────────▼─────────────────────────┐                │
  │  world_pos_publisher.py (ROS2)   │                │
  │  sub /gazebo/model_states        │                │
  │  pub /uav_world_positions ───────┼────────────────┘
  │  (10 Hz, [id,x,y,z,...])          │
  └───────────────────────────────────┘
           │
           │ /uav_world_positions, /link_obstacle_loss
           ▼
┌──────────────────────────────────────────────────────────────────────┐
│                    NS-3 PROCESS  (three_uav_tapbridge_integrated)      │
│  ┌─────────────────────────────┐   ┌────────────────────────────────┐│
│  │ THREAD B: rclcpp executor    │   │ THREAD A: Simulator::Run()     ││
│  │  Ns3RosNode "ns3_bridge_node"│   │  ApplyFeed(): drains buffer →  ││
│  │  sub /uav_world_positions    │──▶│  ConstantVelocityMobilityModel ││
│  │  sub /link_obstacle_loss     │   │  DynamicObstacleLossModel      ││
│  │  pub /ns3_link_rssi          │◄──│  Wi-Fi 802.11a PHY + fading    ││
│  │  pub /ns3_link_snr           │   │  TapBridge ⇄ tap-gcs/uav1/2/3  ││
│  └───────────────────────────────┘   └────────────┬───────────────────┘│
└────────────────────────────────────────────────────┼─────────────────┘
                                                       │ Layer-2 Ethernet
                                                       │ frames over TAP
                                                       ▼
                                        (whatever real app sends packets
                                         over the tap-uavN / tap-gcs devices)
```

No network namespaces are used by this particular pipeline
(`launch_multi_uav_new.sh`) — everything runs flat on the host. Network
namespaces (`ip netns`) **are** used by a different, older launch path
(`launch_multi_uav_netns.sh`, `launch_multi_dds.sh` — see Section 2), which
is a separate design that isolates each drone's MAVLink traffic into its
own namespace so it's forced through ns-3's TAP bridges. That second design
uses the **old** `three_uav_tapbridge_rt` ns-3 target, not
`three_uav_tapbridge_integrated`, and has no live position feed at all
(Section 4).

---

## SECTION 2 — Which launch script is "the" one

I read every script under `launch/` in full. Here's what each one
actually starts, in order:

| Script | World | Starts (in order) | Namespaces? | NS-3 feed? |
|---|---|---|---|---|
| `launch_multi_uav_new.sh` | `small_city_base.world` | kill old procs → build ArduCopter → launch Gazebo → sleep 20s → source ROS → **start `world_pos_publisher.py`** → SITL×3 (own tmp dirs) | No (flat host) | **Yes** — starts the publisher; ns-3 binary itself is left for a second terminal |
| `launch_multi_uav.sh` | `multi_uav_plugin.world` (predecessor world) | kill old procs → build ArduCopter → Gazebo → SITL×3 (shared cwd `~/`) | No | No — never calls `world_pos_publisher.py` |
| `launch_multi_uav_netns.sh` | `multi_uav.world` (plain, no plugins) | kill old procs → `gzserver` (headless) → SITL×3 each inside `ip netns exec uavNns` | **Yes** | No feed script started; prints instructions to manually run **`three_uav_tapbridge_rt`** (the old, no-ROS target) in a separate terminal |
| `launch_multi_dds.sh` | `multi_uav.world` | netns+TAP setup script → Gazebo → 3× `micro_ros_agent` inside `gcsns` → SITL×3 (DDS env vars pointed at GCS namespace IP) → 3× `drone_bridge` | **Yes** | No feed script; prints instructions for **`three_uav_tapbridge_rt`** |
| `launch_faculty_dds.sh` | `faculty_uav.world` | Gazebo → 1× `micro_ros_agent` → 1× SITL (UAV1 only, DDS) | No | Not referenced at all |
| `launch_airport_dds.sh` | `airport_3uav.world` (per its comment header) | same DDS pattern as faculty | No | Not referenced at all |
| `launch_single_dds.sh` | `single_uav.world` | Gazebo → SITL single instance, DDS | No | Not referenced |
| `launch_single.sh` | `single_uav.world` | Gazebo → 1 SITL instance | No | Not referenced |

**Reasoning for which is "current":**

- `launch_multi_uav_new.sh` is the only script that (a) uses the newest,
  richest world (`small_city_base.world`, which is the only world besides
  `multi_uav_plugin.world` that contains the `gcs` model, the
  `obstacle_raycast` plugin, and the `gazebo_ros_state` plugin — see
  Section 5), and (b) actually starts `world_pos_publisher.py`
  (`launch/launch_multi_uav_new.sh:88-90`), whose own docstring says it
  exists specifically to feed `three_uav_tapbridge_integrated`
  (`scripts/world_pos_publisher.py:17`). This matches `SETUP.md`'s
  documented 4-terminal run procedure exactly (`SETUP.md:238-256`, quoted
  in Section 8).
- `launch_multi_uav.sh` looks like the **immediate predecessor** of
  `launch_multi_uav_new.sh` — same structure, same comments about "ADDED"
  the plugin path, but missing the position-feed block and using the
  older `multi_uav_plugin.world`. The file name itself ("new" vs. no
  suffix) and the fact it's missing exactly the one block that makes the
  live NS-3 feed work supports this being the earlier version kept around
  rather than deleted.
- `launch_multi_uav_netns.sh` and `launch_multi_dds.sh` are a **different,
  older architecture entirely**: they both point at `three_uav_tapbridge_rt`
  (confirmed dead-end for live positions, see Section 4), both use
  `multi_uav.world` (the plainest world, no obstacle plugin, no GCS
  model — Section 5), and `launch_multi_dds.sh` additionally has a
  structural oddity that strongly suggests it's stale (see Section 9:
  it's literally a self-writing heredoc script targeting a path that
  isn't even this checkout's location).
- `launch_faculty_dds.sh`, `launch_airport_dds.sh`, `launch_single_dds.sh`,
  `launch_single.sh` are **mission-specific launchers with no ns-3
  involvement at all** — they're a separate concern (flying a specific
  scripted mission over a specific world) and don't compete with the
  ns-3 pipeline question.

**Conclusion:** `launch_multi_uav_new.sh`, paired with the
`three_uav_tapbridge_integrated` ns-3 binary run by hand in a second
terminal, is the current intended way to run the integrated Gazebo↔ROS↔ns-3
stack. `launch_multi_uav_netns.sh` / `launch_multi_dds.sh` are an older,
namespace-based design that this branch has moved away from but not deleted.

---

## SECTION 3 — The ROS 2 side

### Nodes in `ros2/uav_controller/uav_controller/`

| File | Role (one sentence) | Publishes | Subscribes |
|---|---|---|---|
| `drone_bridge.py` | Per-UAV MAVLink↔DDS↔ROS2 bridge; the only node that actually talks to a SITL instance. | `{ns}/gps`, `{ns}/rel_alt`, `{ns}/mode`, `{ns}/armed`, `{ns}/battery` (`drone_bridge.py:100-104`) | `/ap/navsat`, `/ap/battery` (DDS, `:92-97`), `{ns}/goto` (`:108-110`) |
| `multi_mission.py` | Barrier-synchronised 3-drone mission (generic L/R/forward pattern). | `{ns}/goto` per UAV, `/cluster/status` (`:154-157`) | `{ns}/gps`,`{ns}/rel_alt`,`{ns}/mode`,`{ns}/armed` per UAV (`:134-143`) |
| `faculty_mission.py` | 3-drone building-sweep mission over a university campus world. | `{ns}/goto` per UAV, `/cluster/status` (`:153,157`) | same 4 telemetry topics per UAV (`:136-145`) |
| `airport_mission.py` | 3-drone clustering surveillance mission at KSQL Airport. | `{ns}/goto` per UAV, `/cluster/status` (`:143,146`) | same 4 telemetry topics per UAV (`:130-136`) |
| `l_mission.py` | Single scripted L-shaped flight path for UAV1 only. | `{ns}/goto` (`:89`) | `{ns}/gps`,`{ns}/rel_alt`,`{ns}/mode`,`{ns}/armed` (`:79-82`) |
| `takeoff_mission.py` | Calls `drone_bridge`'s arm/takeoff services for one UAV; no topics of its own beyond telemetry. | none | `{ns}/rel_alt`,`{ns}/mode`,`{ns}/armed` (`:32-34`) |
| `position_accuracy_test.py` | Validation script: commands UAV1 to known GPS points and measures arrival error. | `/uav1/goto` (`:94`) | `/uav1/gps`,`/uav1/rel_alt`,`/uav1/mode` (`:85-89`) |
| `ns3_ros2_bridge.py` | Unix-socket JSON relay meant to feed an ns-3 process — **not wired into anything built on this branch** (see Section 4/9). | `/ns3_link_metrics` (`:35`) | `/uav_world_positions`, `/link_obstacle_loss` (`:28-32`) |

### Scripts under `scripts/` that are ROS 2 nodes or pub/sub participants

| File | Role | Publishes | Subscribes |
|---|---|---|---|
| `world_pos_publisher.py` | **The** Gazebo→ns-3 position feed; single source of truth for world positions. | `/uav_world_positions` (`:90`) | `/gazebo/model_states` (default topic, `:89`) |
| `viz_dashboard.py` | Web dashboard (Dash/Plotly) — **not a real rclpy node**; it shells out to `ros2 topic echo` via `subprocess` (`:38-58`) to poll positions, so it doesn't appear in the rclpy graph at all. Notably it polls `/uav{N}/mavros/local_position/pose` (`:42`), a topic **nothing in this repo publishes** (see Section 9). | — | (indirect, via subprocess) |
| `single_drone_mission.py`, `single_drone_takeoff.py`, `multi_drone_misison.py` (sic), `multi_drone_mission_new.py` | Talk to SITL **directly via pymavlink** (`mavutil.mavlink_connection(...)`), bypassing ROS 2 entirely. Not part of the ROS graph. | — | — |
| `test_scripts/record_live_links.py` | Passive CSV recorder for a live run — explicitly publishes nothing (`:17`, `"IT PUBLISHES NOTHING AND LAUNCHES NOTHING"`). | none | `/uav_world_positions`, `/link_obstacle_loss`, `/ns3_link_rssi`, `/ns3_link_snr` (`:122-128`) |
| `test_scripts/run_channel_validation.py` | Synthetic validation harness — stands in for Gazebo, drives ns-3 directly with scripted positions/loss. | `/uav_world_positions`, `/link_obstacle_loss` (`:99-101`) | (reads ns-3's own log/CSV output, not ROS topics) |
| `test_scripts/sweep_test.py` | Manual sweep test tool for the obstacle-loss model. | `/uav_world_positions` (`:28`) | `/link_obstacle_loss` (`:29-30`) |

### Topic graph (edges): publisher → topic → subscriber(s)

```
gazebo_ros_state plugin        → /gazebo/model_states     → world_pos_publisher.py
world_pos_publisher.py         → /uav_world_positions      → obstacle_raycast_plugin (Gazebo, C++)
                                                            → three_uav_tapbridge_integrated (Ns3RosNode, C++)
                                                            → ns3_ros2_bridge.py            [orphaned receiver, Section 4]
                                                            → record_live_links.py (passive)
obstacle_raycast_plugin        → /link_obstacle_loss       → three_uav_tapbridge_integrated (Ns3RosNode, C++)
                                                            → ns3_ros2_bridge.py            [orphaned]
                                                            → record_live_links.py (passive)
                                                            → sweep_test.py
three_uav_tapbridge_integrated → /ns3_link_rssi            → record_live_links.py (passive)
three_uav_tapbridge_integrated → /ns3_link_snr              → record_live_links.py (passive)

--- MAVLink/telemetry side, independent sub-graph ---
ArduPilot SITL (via AP_DDS)     → /ap/navsat, /ap/battery  → drone_bridge.py (one instance per UAV)
drone_bridge.py                 → {ns}/gps,/rel_alt,/mode,/armed,/battery
                                                            → multi_mission.py | faculty_mission.py |
                                                              airport_mission.py | l_mission.py |
                                                              takeoff_mission.py | position_accuracy_test.py
multi_mission.py / faculty_mission.py / airport_mission.py / l_mission.py / position_accuracy_test.py
                                 → {ns}/goto                → drone_bridge.py (same UAV's instance)
multi_mission.py / faculty_mission.py / airport_mission.py
                                 → /cluster/status           → (no subscriber found in this branch's tree)

--- Unix-socket side channel (parallel to the ROS graph, but not wired to any built ns-3 target — Section 4) ---
ns3_ros2_bridge.py  ⇄ (JSON over /tmp/ns3_uav_bridge.sock) ⇄  ns3_ros2_bridge.cc (never compiled)
```

I found **no subscriber anywhere in this branch's tree** for `/cluster/status`
or for the camera topic (`/uav{N}/camera/image_raw`, see Section 6) — both
appear to be intended for a consumer that either doesn't exist on this
branch or hasn't been written yet (see Section 9 re: `city_mission`).

---

## SECTION 4 — The ns-3 side

Five `.cc` files exist under `ns3/`:

| File | Built by CMake as | Live ROS input? | Mobility model |
|---|---|---|---|
| `three_uav_tapbridge_rt.cc` | **Not built** by the current `ns3/CMakeLists.txt` at all (no `build_exec` references it) | No — self-contained | Gauss-Markov random walk or hardcoded waypoint back-and-forth |
| `three_uav_tapbridge_obstacle_loss.cc` | `three_uav_tapbridge_obstacle_loss` target (`ns3/CMakeLists.txt:41-59`) | **Yes** — embeds an `rclcpp::Node`, subscribes to `/uav_world_positions` and `/link_obstacle_loss` (`three_uav_tapbridge_obstacle_loss.cc:67,71`) | `ConstantVelocityMobilityModel`, driven by the ROS feed |
| `three_uav_tapbridge_integrated.cc` | `three_uav_tapbridge_integrated` target (`ns3/CMakeLists.txt:18-37`) — **the current scenario** per its own comment (`:6`) | **Yes**, same mechanism, plus a GCS node | `ConstantVelocityMobilityModel` for all 4 nodes by default; `--standalone` flag switches to synthetic Gauss-Markov with no ROS at all |
| `dynamic_obstacle_loss_model.cc` | Compiled into **both** targets above as a helper propagation-loss model, not an executable itself | — | — |
| `ns3_ros2_bridge.cc` | **Not built** — does not appear in either `build_exec` block's `SOURCE_FILES` | Designed to be, via a Unix-domain-socket client, but nothing built links it | N/A (never runs) |

`three_uav_tapbridge_rt.cc`'s own header explains why it takes no live
input — it predates the ROS integration entirely:
```c
/*
 * three_uav_tapbridge_rt.cc
 *
 * Real-time NS-3 scenario for multi_uav_sim.
 * Bridges three network namespaces (uav1ns/uav2ns/uav3ns) into a simulated
 * 802.11n ad-hoc channel with Nakagami fading + log-distance path loss.
 */
```
(`ns3/three_uav_tapbridge_rt.cc:1-8`)

`three_uav_tapbridge_obstacle_loss.cc`'s header records the design change:
```c
/*
 * Updated from 'three_uav_tapbridge_rt.cc'
 * ...
 * Communication with Gazebo/ROS2 is now done via a native rclcpp::Node
 * embedded directly in this NS-3 process -- no ZMQ, no separate Python
 * bridge process. Subscribes to /uav_world_positions and
 * /link_obstacle_loss, publishes /ns3_link_rssi.
 */
```
(`ns3/three_uav_tapbridge_obstacle_loss.cc:1-11`)

`three_uav_tapbridge_integrated.cc` explicitly names its two ancestors and
integrates them, adding the GCS as a real node instead of a hard-coded
coordinate:
```c
//  Integration of:
//    - three_uav_tapbridge_obstacle_loss.cc  (ROS 2 / Gazebo co-simulation)
//    - three_uav_tapbridge_rt_new.cc         (small_city_world-wimukthi branch)
//
//  Topology: 4 nodes. All four positions come from Gazebo over ROS -- the GCS
//  is a real model in the world, not a hard-coded coordinate in this file.
```
(`ns3/three_uav_tapbridge_integrated.cc:2-9`)

Mobility model, quoted directly:
```cpp
gcsMob.SetMobilityModel(standalone ? "ns3::ConstantPositionMobilityModel"
                                   : "ns3::ConstantVelocityMobilityModel");
...
uavMob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
```
(`ns3/three_uav_tapbridge_integrated.cc:755-756,792`)

Positions arrive via a native ROS 2 subscription (no socket/JSON layer):
```cpp
m_posSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
  "/uav_world_positions", 10, std::bind(&Ns3RosNode::OnPositions, this, ...));
```
(`ns3/three_uav_tapbridge_integrated.cc:254-256`)

...buffered into a mutex-guarded map on the ROS thread, then applied to the
mobility model on the ns-3 simulation thread by a recurring scheduled event:
```cpp
static void ApplyFeed(NodeContainer nodes, Ptr<DynamicObstacleLossModel> obstacleLoss,
                      double periodSec)
{
  std::map<uint32_t, Vector> pos;
  ...
  for (const auto & kv : pos) {
    auto mob = nodes.Get(id)->GetObject<ConstantVelocityMobilityModel>();
    if (mob) { mob->SetPosition(kv.second); ... }
  }
  Simulator::Schedule(Seconds(periodSec), &ApplyFeed, nodes, obstacleLoss, periodSec);
}
```
(`ns3/three_uav_tapbridge_integrated.cc:361-401`, abridged)

**Which one does the current launch script pair with?**
`launch_multi_uav_new.sh` starts `world_pos_publisher.py`, whose own
docstring says its consumer is `three_uav_tapbridge_integrated`
(`scripts/world_pos_publisher.py:17`), and `SETUP.md` confirms the same
target name in its run instructions (Section 8). The ns-3 binary itself is
**not** started by any launch script — it's a documented manual step in a
second terminal (Section 8).

---

## SECTION 5 — World files

| World file | Used by | GCS model? | `obstacle_raycast` plugin? | iris drones? |
|---|---|---|---|---|
| `small_city_base.world` | `launch_multi_uav_new.sh` (current) | Yes — `<model name="gcs">` (`:4534`) | Yes (`:4559`) | `iris_1/2/3` (`:4494-4513`) |
| `multi_uav_plugin.world` | `launch_multi_uav.sh` (older) | Yes — `<model name="gcs">` (`:145`) | Yes (`:170`) | `iris_1/2/3` (`:105-124`) |
| `multi_uav.world` | `launch_multi_uav_netns.sh`, `launch_multi_dds.sh` | No | No | `iris_1/2/3` (`:79-98`) |
| `faculty_uav.world` | `launch_faculty_dds.sh` | No | No | `iris_1/2/3` (`:384-403`) |
| `airport_3uav.world` | `launch_airport_dds.sh` | No | No | `iris_1/2/3` (`:60-77`) |
| `single_uav.world` | `launch_single.sh`, `launch_single_dds.sh` | No | No | `iris_1` only (`:78-81`) |
| `ksql_airport.world` | **No launch script references it** | No | No | None (0 `<model name=` matches) |

**Flagged as unused:** `ksql_airport.world` is not referenced by any
`launch/*.sh` script (confirmed by grepping every script for
`worlds/`) and contains no models at all in the search performed — it
looks like a leftover stub, possibly superseded by `airport_3uav.world`
(same subject — KSQL airport — but that one actually has the 3 iris
models and is the one `launch_airport_dds.sh` uses).

Only the two worlds paired with the "ROS-integrated" launch scripts
(`small_city_base.world`, `multi_uav_plugin.world`) carry the GCS model and
the obstacle-raycast plugin — every other world is a plain flight-only
scene with no wireless-channel-relevant plugins at all.

---

## SECTION 6 — Sensors actually present on the UAVs

`models/iris_1/model.sdf` (and `iris_2`, `iris_3`, which are identical
except for name suffix, `fdm_port_in/out`, IMU link name, camera sensor
name, and ROS namespace — confirmed by `diff`) includes two other models:
```xml
<include>
  <uri>model://iris_with_standoffs</uri>
</include>
<include>
  <uri>model://gimbal_small_2d</uri>
</include>
```
(`models/iris_1/model.sdf:4-9`)

**IMU sensor** — defined inside `iris_with_standoffs/model.sdf`:
```xml
<link name='iris/imu_link'>
  ...
  <sensor name="imu_sensor" type="imu">
    <pose>0 0 0 3.141593 0 0</pose>
    <always_on>1</always_on>
    <update_rate>1000.0</update_rate>
  </sensor>
</link>
```
(`models/iris_with_standoffs/model.sdf:155-172`)

**Camera sensor** — defined directly in each `iris_N/model.sdf` (not in
`gimbal_small_2d`, which only supplies the mechanical gimbal mount; the
camera lives on a `camera_link` attached via a fixed joint):
```xml
<!-- Camera sensor on gimbal tilt link → publishes to ROS2 -->
<link name="camera_link">
  <pose>0 -0.01 0.070 0 0 0</pose>
  ...
  <sensor name="iris1_camera" type="camera">
    <pose>0 0 0 0 1.5707963 0</pose>
    <camera>
      <horizontal_fov>2.0</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>15000</far>
      </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>10</update_rate>
    <visualize>1</visualize>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/uav1</namespace>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_optical_frame</frame_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>
  </sensor>
</link>
```
(`models/iris_1/model.sdf:592-630`; `iris_2`/`iris_3` are identical with
`/uav2`/`/uav3` namespaces and `iris2_camera`/`iris3_camera` sensor names)

**No `<sensor type="gps">` or lidar block exists anywhere** in
`iris_1/model.sdf` or `iris_with_standoffs/model.sdf` (I grepped both
files for "gps" case-insensitively and found no matches). The GPS data
seen on ROS topics (`{ns}/gps` published by `drone_bridge.py`) comes from
ArduPilot SITL's own internal simulated GPS/EKF, not a Gazebo sensor
plugin — worth confirming with whoever set up SITL if you need to know
exactly how that's synthesized, since it's outside this model chain.

**Plain answer to "does a working camera exist":** **Yes.** `libgazebo_ros_camera.so`
is a real, standard Gazebo ROS 2 plugin, correctly attached to a real
`<sensor type="camera">` block, namespaced per-drone. I independently
found runtime evidence this actually works: a leftover log at
`results/first_full_success_20260723_1013/city_mission.log` contains the
line `Camera relay active: /uav{1,2,3}/camera/image_raw → /cluster/cam/uav{1,2,3}`,
confirming the topic name and that it has published real frames at least
once. However, the `city_mission.py` script that log came from **does not
exist anywhere in this branch's source tree** (see Section 9) — so on
`integrated-ns3` today, the camera topic has no consumer.

---

## SECTION 7 — Custom Gazebo plugins

One custom compiled plugin package exists: `gazebo_plugins/`
(ament/colcon package name **`multi_uav_gazebo_plugins`**, per
`gazebo_plugins/CMakeLists.txt:5`), building a single shared library,
`libobstacle_raycast_plugin.so` (`gazebo_plugins/CMakeLists.txt:14-15`).

**What it does:** embeds its own `rclcpp::Node`, subscribes to
`/uav_world_positions`, and on every Gazebo world-update tick (throttled to
10 Hz) casts a ray between every pair of the `n_nodes_` (GCS + UAVs) to
compute obstacle-shadowing loss, publishing the result:

```cpp
pos_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
  "/uav_world_positions", 10,
  [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    this->UpdatePositions(msg->data);
  });

loss_pub_ = ros_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
  "/link_obstacle_loss", 10);
```
(`gazebo_plugins/src/obstacle_raycast_plugin.cc:34-41`)

```cpp
for (int i = 0; i < n_nodes_; i++) {
  for (int j = i + 1; j < n_nodes_; j++) {
    ...
    double extra_loss = CastRay(pos_i, pos_j, i, j);
    loss_msg.data.push_back(...);
    PublishRayMarker(i, j, pos_i, pos_j, extra_loss > 0.0);
  }
}
if (!loss_msg.data.empty()) loss_pub_->publish(loss_msg);
```
(`gazebo_plugins/src/obstacle_raycast_plugin.cc:85-99`)

It also does one purely-visual thing that stays inside Gazebo and never
reaches ROS: `PublishRayMarker()` sends a colored line marker (red =
blocked, blue = GCS link, green = clear) to Gazebo's native Ignition
`/marker` service for on-screen debugging (`gazebo_plugins/src/obstacle_raycast_plugin.cc:223-241`).

**Referenced by world files:** yes — `small_city_base.world:4559` and
`multi_uav_plugin.world:170` both load it:
```xml
<plugin name="obstacle_raycast" filename="libobstacle_raycast_plugin.so">
    <n_uavs>3</n_uavs>
    <uav_prefix>iris_</uav_prefix>
    <gcs_enabled>true</gcs_enabled>
    <gcs_model>gcs</gcs_model>
    <gcs_antenna_height>2.9</gcs_antenna_height>
</plugin>
```
(`worlds/small_city_base.world:4559-4568`) — note the `2.9` here matches
`world_pos_publisher.py`'s own default of `2.9` for the same purpose
(`scripts/world_pos_publisher.py`, `gcs_antenna_height` parameter), so
these two independently-configured values are at least currently
consistent with each other.

The other plugin referenced by these same worlds,
`libgazebo_ros_state.so` (`worlds/small_city_base.world:4574`), is a
**stock** Gazebo-ROS plugin (part of `gazebo_ros`, not custom to this
repo) — it's what actually produces `/gazebo/model_states`, which
`world_pos_publisher.py` consumes.

---

## SECTION 8 — If I ran this today, step by step

Walking through `launch/launch_multi_uav_new.sh` top to bottom:

1. **`source "$PROJECT_DIR/setup.sh"`** (`:8`) — sets `ARDUPILOT_HOME`
   (hard-coded in `setup.sh:14` to `$HOME/ardupilot`), `GAZEBO_MODEL_PATH`,
   `GAZEBO_RESOURCE_PATH`, `GAZEBO_PLUGIN_PATH`, `LD_LIBRARY_PATH`. It
   **hard-fails immediately** if `$HOME/ardupilot` doesn't exist
   (`setup.sh:28-32`) — this would need to be true on your machine, or you
   edit `setup.sh`.

2. **`export GAZEBO_PLUGIN_PATH="$WS_INSTALL/multi_uav_gazebo_plugins/lib:..."`**
   (`launch/launch_multi_uav_new.sh:12-13`), where
   `WS_INSTALL="$(cd "$PROJECT_DIR/../.." && pwd)/install"`. **This is a
   real risk I can't confirm without running it**: on this machine, with
   the repo checked out at `~/FYP/multi_uav_sim`, `$PROJECT_DIR/../..`
   resolves to `~` (home directory), so `WS_INSTALL` becomes `~/install` —
   and **I confirmed no such directory exists** (`ls ~/install` → not
   found). `SETUP.md` assumes the repo is cloned to
   `~/multi-uav-workspace/src/multi_uav_simulation` (`SETUP.md:95-97`),
   two levels under a colcon workspace root, which is where this path
   arithmetic would actually resolve correctly. As checked out here, this
   line would set `GAZEBO_PLUGIN_PATH` to a nonexistent directory, and
   Gazebo would very likely fail (or silently skip) loading
   `libobstacle_raycast_plugin.so` when it parses
   `small_city_base.world`'s plugin block. **I did not run Gazebo to
   confirm the exact failure mode** — but the missing directory is a
   directly-verified fact, not a guess.

3. **Cleanup** (`:22-26`): kills any previous `arducopter`, `gzserver`,
   `gzclient`, `world_pos_publisher` processes, sleeps 5s.

4. **ARDUPILOT_HOME check** (`:29-33`) — already validated by `setup.sh`,
   redundant but harmless.

5. **`WORLD_PATH="$PROJECT_DIR/worlds/small_city_base.world"`** (`:46`) —
   exists in this repo, no issue.

6. **Build ArduCopter** (`:60-63`):
   `python3 modules/waf/waf-light build --target bin/arducopter` inside
   `$ARDUPILOT_HOME`. Requires a working ArduPilot checkout with `waf`
   build tooling already configured (Python deps, cross-compile toolchain
   if any). **I can't confirm this succeeds without running it** — it
   depends entirely on the state of `~/ardupilot` on your machine, which
   is outside this repo.

7. **Binary existence check** (`:65-68`) — fails loudly if the build
   didn't produce `$ARDUPILOT_HOME/build/sitl/bin/arducopter`.

8. **Launch Gazebo** (`:70-72`): `gazebo --verbose "$WORLD_PATH" &`, then
   **hard-coded `sleep 20`** (`:75`) to let it finish loading — not an
   actual readiness check, so on a slower machine this could race.

9. **Start `world_pos_publisher.py`** (`:83-90`): re-sources
   `/opt/ros/humble/setup.bash` (requires ROS 2 Humble installed at that
   path), then `setsid python3 .../world_pos_publisher.py` backgrounded,
   logging to `/tmp/world_pos_publisher.log`. This node will immediately
   try to subscribe to `/gazebo/model_states` — if step 2's plugin-path
   issue prevented `libgazebo_ros_state.so` from working (unlikely, since
   that's a stock plugin found via the *system* Gazebo plugin path, not
   the custom one), or if Gazebo isn't fully up yet, this node will just
   sit idle logging warnings rather than crash.

10. **Three ArduCopter SITL instances** (`:98-116`), each in its own
    `/tmp/sitl_uavN` working directory (to avoid the `eeprom.bin`
    contention the comment at `:92-96` describes fixing), sysids 1-3,
    listening on MAVLink TCP ports 5760/5770/5780 (per the echoed
    instructions at `:120-122`).

11. **`wait $GAZEBO_PID`** (`:124`) — script blocks here until Gazebo
    exits; this is a foreground-holding pattern, not a background daemon.

**What must be true on your machine for this to succeed, summarized:**
- `$HOME/ardupilot` exists and is a working ArduPilot checkout buildable
  with `waf`.
- `gazebo` (Gazebo Classic 11, per `SETUP.md`) is installed and on `PATH`.
- ROS 2 Humble is installed at `/opt/ros/humble`.
- `gazebo_msgs`/`gazebo_ros` are available (for `ModelStates` and the
  stock state plugin).
- **The custom plugin path resolves correctly** — which, as this repo is
  currently checked out at `~/FYP/multi_uav_sim` rather than nested two
  levels under a colcon workspace root, it currently does **not**
  (verified above). This would need either moving the checkout to match
  `SETUP.md`'s assumed layout, or the `GAZEBO_PLUGIN_PATH` line being
  pointed somewhere that actually has `libobstacle_raycast_plugin.so`.
- Separately, and not started by this script at all: the ns-3
  `three_uav_tapbridge_integrated` binary must be built (`./ns3 build
  three_uav_tapbridge_integrated`) and run by hand in a second terminal,
  per `SETUP.md:246-251`. I found no evidence this has been built on this
  machine — no `scratch/multi_uav_simulation` directory exists under
  `~/ns-allinone-3.38/ns-3.38/scratch/`, only a stale `scratch/three-uav/`
  directory holding the unrelated `three_uav_tapbridge_rt.cc`.

---

## SECTION 9 — Loose ends, inconsistencies, and unverified claims

- **`ns3/ns3_ros2_bridge.cc` + `ros2/.../ns3_ros2_bridge.py` are a complete
  but orphaned subsystem.** Neither is referenced by anything that
  actually builds: `ns3_ros2_bridge.cc` doesn't appear in either
  `build_exec` block of `ns3/CMakeLists.txt`, and `ns3_ros2_bridge.py` is
  not registered in `ros2/uav_controller/setup.py`'s `entry_points`
  (7 scripts listed, none named `ns3_ros2_bridge`) and has no
  `if __name__ == '__main__':` guard (`ros2/uav_controller/uav_controller/ns3_ros2_bridge.py:99-101`,
  file ends at the `main()` definition). Both files were added together
  in a single commit and never touched again (`git log --follow` on both
  shows only commit `04b801d`, "Version 1 of Obstacle_loss_plugin added").

- **`launch/launch_multi_dds.sh` is a self-writing heredoc script, not a
  normal launcher.** Its entire body is `cat > ~/finaly_year_project/multi_uav_simulation/launch/launch_multi_dds.sh << 'SCRIPT' ... SCRIPT` (`launch/launch_multi_dds.sh:1,206`),
  followed by a `chmod +x` on that same hard-coded, misspelled path
  (`finaly_year_project`, not this repo's actual location). Running this
  file as checked out does not launch a simulation — it writes a copy of
  itself to a directory structure that isn't this checkout.

- **`ns3/CMakeLists.txt`'s comment names a target, `three-uav`, that
  doesn't exist** — the file only defines `three_uav_tapbridge_integrated`
  and `three_uav_tapbridge_obstacle_loss` (`ns3/CMakeLists.txt:6-7,18,41`).
  The old `three-uav` target name only survives in a stale copy under
  `~/ns-allinone-3.38/ns-3.38/scratch/three-uav/`, which is a leftover
  from before this branch's restructuring, not something this branch's
  CMakeLists produces.

- **`scripts/viz_dashboard.py` polls a topic nothing in this repo
  publishes**: `f"/uav{uav_id}/mavros/local_position/pose"`
  (`scripts/viz_dashboard.py:39,42,50`) is a MAVROS-style topic name, but
  this project's telemetry bridge (`drone_bridge.py`) uses AP_DDS
  directly and publishes `{ns}/gps` (`NavSatFix`), not
  `mavros/local_position/pose`. This dashboard would silently show no
  position data when run against this stack's actual topics.

- **`results/first_full_success_20260723_1013/city_mission.log` references
  a script that doesn't exist on this branch.** The log's own content
  (`Camera relay active: /uav{1,2,3}/camera/image_raw → /cluster/cam/uav{1,2,3}`,
  a full "City Surveillance Mission" banner, cluster-head/member role
  assignment) shows a working `city_mission` ROS node once ran
  successfully — but `city_mission.py` is not present anywhere in the
  `integrated-ns3` working tree (`grep -rln "city_mission"` across the
  repo finds nothing outside `results/`). `git log --all -S
  "city_mission"` shows the string exists in other branches/commits
  (`f5a12e1 "small city on fire"`, `90bff41 "mission has to be fixed"`),
  so this looks like a leftover artifact from a different branch or an
  in-progress feature not yet present here, not something broken on this
  branch specifically.

- **The commit that introduced the current ns-3 scenario admits it
  wasn't verified.** `git log --oneline -- ns3/three_uav_tapbridge_integrated.cc`
  shows the file was added in commit `7ae35de`, whose message is:
  *"Integrated ns3 version with removing double penalty issues. Modified
  versions of scripts in obstacle_loss_feature and small_city_world-wimukthi.
  Testing for verfication not done."* (author dice_apps, 2026-07-22 — one
  day before the latest commit on this branch).

- **No built artifact for `three_uav_tapbridge_integrated` was found
  anywhere on this machine.** `~/ns-allinone-3.38/ns-3.38/scratch/`
  contains no `multi_uav_simulation` directory (the name
  `ns3/CMakeLists.txt:3-4`'s own instructions say to create), and a
  home-directory-wide search for a built binary found only a VSCode
  IntelliSense cache file
  (`~/.cache/vscode-cpptools/ipch/.../three_uav_tapbridge_integrated.ipch`),
  not an actual executable. Combined with the previous point, whether
  this scenario has ever successfully run end-to-end on this machine is
  unconfirmed by anything on disk.

- **The `GAZEBO_PLUGIN_PATH` arithmetic in `launch_multi_uav_new.sh` (and
  identically in `launch_multi_uav.sh`) assumes a workspace nesting depth
  that doesn't match where this repo is actually checked out.** Detailed
  in Section 8, point 2 — `SETUP.md` assumes
  `~/multi-uav-workspace/src/multi_uav_simulation`; the actual checkout is
  `~/FYP/multi_uav_sim`, one level shallower, so `$PROJECT_DIR/../../install`
  resolves to a nonexistent `~/install` rather than a real colcon install
  space.

- **`architecture.md` and `SETUP.md` disagree on the current NS-3 target
  name.** `architecture.md`'s own diagram box labels the NS-3 process
  `three_uav_tapbridge_obstacle_loss` (`architecture.md:40-52` — the
  3-node predecessor), while `SETUP.md` (which has the most recent commit
  touching it, "Updated SETUP") names `three_uav_tapbridge_integrated`
  as current (`SETUP.md:20-30,147-148`). The code-level evidence (CMake
  comment, commit history, launch-script pairing) supports `SETUP.md`
  being correct and `architecture.md` being the stale one on this point.

- **No GPS `<sensor>` block exists anywhere in the iris model chain**
  (Section 6) — flagged as ambiguous rather than a bug, since ArduPilot
  SITL commonly synthesizes GPS internally rather than via a Gazebo
  sensor plugin, but I could not confirm the exact mechanism from files
  in this repo alone.

- **No explicit `TODO`/`FIXME` markers found** in any `.py`/`.cc`/`.hh`/`.sh`
  file repo-wide (a direct grep for those tokens plus phrases like "not
  tested" / "untested" / "doesn't work" returned nothing outside the one
  commit message above) — the project's known rough edges are recorded in
  commit messages and prose comments rather than code-level TODO tags.

# Multi-UAV Simulation — Practical Guide

How to run the simulation, resize the fleet, write your own mission, add a
world, and test your own algorithm.

> This supersedes `README_ADDING_DRONES.md`, which describes the old manual
> process (hand-copying model folders, editing launch scripts per drone). None
> of that is needed any more — fleet size is one number in one file.

---

## 1. What you are running

Five processes cooperate. Understanding which one owns what saves a lot of
debugging time:

| Piece | Where it runs | Owns |
|---|---|---|
| **Gazebo** | root namespace | Physics, the world, UAV models, obstacle ray-casting |
| **ArduPilot SITL** (one per UAV) | netns `uavN` | Flight control, AP_DDS telemetry |
| **ns-3** | root namespace | The radio channel — path loss, fading, building blockage |
| **micro_ros_agent** (one per UAV) | netns `gcsns` | Bridges AP_DDS telemetry into ROS 2 |
| **ROS 2 nodes** | `gcsns` / root | `drone_bridge`, `city_mission`, `dynamic_cluster_manager` |

Each UAV lives in its own Linux network namespace, wired to an ns-3 Wi-Fi node
through a TAP device. Traffic between UAVs and the GCS therefore experiences a
*simulated* radio channel — that is the point of the whole setup.

Two consequences worth internalising:

- **ns-3 bridges at layer 2** (`TapBridge` in `UseLocal` mode), so ns-3 never
  sees IP. Multi-hop routing is done with Linux `/32` routes, not an ns-3
  routing protocol. See `ros2/uav_controller/uav_controller/cluster_relay_routes.py`.
- **Gazebo runs outside the namespaces**, so each `models/iris_N/model.sdf`
  must point its ArduPilot plugin at `172.31.N.1` / `172.31.N.2`, never
  `127.0.0.1`.

---

## 2. One-time setup

```bash
# ROS 2 package
bash build_ros2.sh

# Every new terminal (or add to ~/.bashrc)
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
source ~/FYP/multi_uav_simulation/ros2/install/setup.bash
```

ns-3 is found automatically at `~/ns-allinone-3.38/ns-3.38` or `~/ns-3-dev`.
Anywhere else, set `NS3_ROOT`.

> **ns-3 source is kept in sync manually.** The build copy is
> `$NS3_ROOT/scratch/multi_uav_simulation/three_uav_tapbridge_integrated.cc`,
> a copy of `ns3/three_uav_tapbridge_integrated.cc` in this repo. Edit both,
> or your change will not take effect.

---

## 3. Running

```bash
./scripts/launch_city_dynamic_clustering.sh
```

It needs `sudo` (network namespaces) and holds the timestamp alive for the
whole run. It starts everything in order, waits for each readiness check,
runs the mission, then tears the whole thing down.

Useful environment knobs:

| Variable | Default | Use |
|---|---|---|
| `ENABLE_GAZEBO_GUI` | `1` | `0` for headless / faster runs |
| `MISSION_TIMEOUT_SEC` | `900` | Mission watchdog |
| `DDS_SETTLE_SEC` | `3` | Pause between SITL starts; raise if DDS bring-up is flaky |
| `NS3_ROOT` | auto | ns-3 checkout location |
| `ROS_DOMAIN_ID` | `0` | ROS 2 domain |

### Where to look when it runs

```
/tmp/city_mission.log               mission progress, waypoints, link loss
/tmp/dynamic_cluster_manager.log    elections, relay decisions
/tmp/micro_ros_agent_uavN.log       DDS session health per UAV
/tmp/multi_uav_sitl/uavN/arducopter.log   SITL console
/tmp/ns3_stdout.log                 ns-3, incl. per-link obstacle annotations
/tmp/ns3_link_validation.csv        per-link SNR / distance / blockage over time
/tmp/ns3_phy_snr.csv                every received packet's SNR
```

The two CSVs are the ground truth for anything RF-related. When a UAV "loses
comms", read the CSV before theorising — it tells you the real SNR.

---

## 4. Changing the number of UAVs

**Edit one number:**

```yaml
# config/fleet.yaml
fleet:
  num_uavs: 5      # <- the only edit required
```

Then run the launcher. Everything else is derived: IPs, ports, namespaces,
TAPs, DDS parameter files, Gazebo models, the world file, the ns-3 node count,
and the mission roles.

**Check before you launch** (fast, no Gazebo/SITL/ns-3 needed):

```bash
python3 scripts/validate_fleet_scaling.py --sizes 5
python3 scripts/validate_fleet_scaling.py            # sweeps 1..6
```

This verifies address/port uniqueness, DDS params matching the port map,
`iris_N` models using netns-reachable FDM addresses, the world spawning
exactly N models, and every UAV receiving a mission role.

### What each UAV gets

Derived by formula from `config/fleet.yaml`, for UAV `N`:

| Item | Formula | UAV1 | UAV5 |
|---|---|---|---|
| SITL instance | `-I(N-1)` | `-I0` | `-I4` |
| MAVLink TCP | `5760 + 10(N-1)` | 5760 | 5800 |
| DDS UDP | `2019 + (N-1)` | 2019 | 2023 |
| Gazebo FDM in/out | `9002/9003 + 10(N-1)` | 9002/9003 | 9042/9043 |
| Wireless IP | `10.42.0.10 + N` | 10.42.0.11 | 10.42.0.15 |
| Namespace / TAP | `uavN` / `tap-uavN` | uav1 | uav5 |

### Mission roles as the fleet grows

`city_mission` gives UAV1–UAV3 the three named roles and treats everything
beyond as a follower, alternating between the two survey areas with each pair
taking the next lane out:

```
UAV1  city centre circle, then relay hover   @ 60 m
UAV2  pond sweep                             @ 40 m
UAV3  mountain sweep                         @ 50 m
UAV4  pond follower,     lane 1 (+10 m east) @ 40 m
UAV5  mountain follower, lane 1 (+10 m east) @ 50 m
UAV6  pond follower,     lane 2 (+20 m east) @ 40 m
UAV7  mountain follower, lane 2 (+20 m east) @ 50 m
```

Fewer than 3 UAVs is valid too — the roles are simply included as far as the
fleet reaches.

### Per-UAV overrides (optional)

Spawn points and altitudes are auto-generated. To pin one:

```yaml
uav_overrides:
  "5":
    spawn: [-70.0, -19.5, 0.0]
    takeoff_altitude_m: 50.0
    mission_profile: "mountain_follower"
```

UAVs without an override use `defaults:` and are auto-placed along Y at
`spawn_spacing_m` from `spawn_origin`.

---

## 5. Writing your own mission

A mission is an ordinary ROS 2 node. It does not talk to ArduPilot — it talks
to `drone_bridge`, which exposes a clean per-UAV API:

| Topic / Service | Type | Direction |
|---|---|---|
| `/uavN/gps` | `sensor_msgs/NavSatFix` | ← position |
| `/uavN/rel_alt` | `std_msgs/Float32` | ← altitude above home |
| `/uavN/battery` | `std_msgs/Float32` | ← percent |
| `/uavN/mode` | `std_msgs/String` | ← flight mode |
| `/uavN/armed` | `std_msgs/Bool` | ← armed state |
| `/uavN/goto` | `geographic_msgs/GeoPoint` | → waypoint |
| `/uavN/arm` `/uavN/disarm` | `std_srvs/Trigger` | → |
| `/uavN/takeoff` `/uavN/land` `/uavN/rtl` | `std_srvs/Trigger` | → |

`/uavN/gps` is published from AP_DDS when healthy and automatically falls back
to MAVLink `GLOBAL_POSITION_INT` when the DDS fix goes stale, so it stays live
through a DDS session collapse.

### Steps

**1. Write the node** in `ros2/uav_controller/uav_controller/my_mission.py`.
Start by copying `city_mission.py` — it already solves the hard parts:

- `self.num_uavs` from the `num_uavs` ROS parameter
- `self.states[uid]` — latest telemetry plus staleness timestamps
- `self._fly_to(uid, lat, lon, alt_m, label, timeout=180)` — commands the
  waypoint and waits for arrival, with link-loss handling built in
- `self._call_service(client, uid, label, timeout=90)`
- `self._wait_for_land(uid)` — waits for actual touchdown, not just the RTL ack
- `self.barriers['takeoff' | 'positions' | 'sweep' | 'hold']` — phase
  synchronisation across all UAVs
- one thread per UAV, errors collected in the shared `errors` list

**2. Register it** in `ros2/uav_controller/setup.py`:

```python
'console_scripts': [
    ...
    'my_mission = uav_controller.my_mission:main',
],
```

**3. Build and run:**

```bash
bash build_ros2.sh
ros2 run uav_controller my_mission --ros-args -p num_uavs:=5
```

**4. To have the launcher run it**, change the node name in
`start_city_mission()` in `scripts/launch_city_dynamic_clustering.sh`.

### Writing a mission that survives link loss

This matters more than it sounds. The mission node lives in `gcsns` and hears
a UAV only through the simulated radio. When the link drops, `/uavN/gps` simply
stops updating — the last value sits there looking valid.

Always check telemetry age, never just the value:

```python
if time.monotonic() - s.last_gps_t > STALE_TELEM_SEC:
    ...   # link lost; do not trust s.lat / s.lon
```

`city_mission._fly_to` shows the full pattern: treat the waypoint as reached if
the UAV got within `LINK_LOST_ARRIVAL_M` before the link died, otherwise keep
commanding it for `LINK_LOST_GRACE_SEC` and resume if the link returns.

---

## 6. Adding a new world

The launcher generates its world from a template, stripping any `iris_<n>`
models and re-inserting exactly N of them at the configured spawn points.

**1. Build a template** at `worlds/my_world.world`. Copy `worlds/city_3uav.world`
and change the scenery. The template **must keep**:

- the `<model name="gcs">` block — the ground station's physical position
- the `obstacle_raycast` plugin (`libobstacle_raycast_plugin.so`) — feeds
  building blockage to ns-3
- the `gazebo_ros_state` plugin — publishes model positions to ROS

`iris_*` models in the template are optional; they get replaced regardless.

**2. Point the launcher at it** — in `scripts/launch_city_dynamic_clustering.sh`:

```bash
python3 "$PROJECT_DIR/scripts/generate_gazebo_fleet.py" \
    --template "$PROJECT_DIR/worlds/my_world.world" \
    ...
```

**3. Set spawn points** for the new world in `config/fleet.yaml`
(`gazebo.spawn_origin`, `gazebo.spawn_spacing_m`, or per-UAV overrides).

**4. Update mission waypoints** — `city_mission.py` has hardcoded GPS targets
(`CITY_LAT/LON`, `POND_LAT/LON`, `MTN_LAT/LON`) that belong to the city world.
`scripts/waypoint_finder.py` helps convert Gazebo XYZ to GPS.

Generate and inspect without launching:

```bash
python3 scripts/generate_gazebo_fleet.py \
    --config config/fleet.yaml \
    --template worlds/my_world.world \
    --output /tmp/test.world \
    --models-dir models --count 5
```

---

## 7. Testing your own algorithm

### Clustering / cluster-head election

`ros2/uav_controller/uav_controller/dynamic_cluster_manager.py` runs an
election every `election_period_sec` (default 2 s) and publishes:

| Topic | Contents |
|---|---|
| `/cluster/assignment` | current membership |
| `/cluster/primary_ch` `/cluster/backup_ch` | elected heads |
| `/cluster/scores` | per-UAV score breakdown |
| `/cluster/event` | election changes |
| `/cluster/relay` | which members are relaying, and why |

Its inputs come from ns-3 and Gazebo:

| Topic | Contents |
|---|---|
| `/ns3_link_snr` | per-link SNR (dB) |
| `/ns3_link_rssi` | per-link RSSI (dBm) |
| `/link_obstacle_loss` | per-link building blockage (dB) |
| `/uav_world_positions` | Gazebo ground-truth positions |

To test your own election, replace `calculate_score()` (line 463) or
`election_callback()` (line 755), or write a new node subscribing to the same
inputs. Everything is a plain
`Float32MultiArray` of flattened link triples, so you do not need the existing
node at all.

### Multi-hop relay

Relay policy lives in `cluster_relay_routes.py`. A member relays through the
cluster head when its **smoothed** direct SNR is bad and its hop to the head is
good, held for `relay_consecutive` ticks.

```bash
ros2 run uav_controller dynamic_cluster_manager --ros-args \
  -p num_uavs:=5 \
  -p relay_enter_snr_db:=15.0 \
  -p relay_exit_snr_db:=22.0 \
  -p relay_min_hop_snr_db:=15.0 \
  -p relay_consecutive:=2 \
  -p relay_snr_ema_alpha:=0.3
```

**Threshold on smoothed SNR, not raw samples.** Raw ns-3 SNR is a per-sample
Rayleigh draw: on a measured 5-UAV run a link whose telemetry was completely
dead still reported instantaneous SNR swinging −20.6 to +14.6 dB (mean 5.3,
σ 5.1). Links actually carrying telemetry averaged 27–31 dB. The defaults above
sit in that gap.

### Tuning the radio

RF parameters live in `config/fleet.yaml` under `simulation:` (tx power, rx
sensitivity, noise floor, path-loss exponent, fading, blockage thresholds).

Changing the PHY rate or propagation model means editing
`ns3/three_uav_tapbridge_integrated.cc` **and** the scratch copy — and it
invalidates comparisons against earlier results, so record what you changed.

### Offline analysis

`/tmp/ns3_link_validation.csv` has one row per link per sample:

```python
import csv
rows = list(csv.DictReader(open('/tmp/ns3_link_validation.csv')))
# t_sim, node_a, node_b, distance_m, obstacle_loss_db, snr_db, blocked, ...
```

Node 0 is the GCS; nodes 1..N are UAV1..UAVN. This is how you replay a policy
against a recorded run instead of re-flying the mission.

---

## 8. Troubleshooting

**Launcher hangs at `Waiting for ... /ap/vN/navsat`**

The UAV's AP_DDS never published. Check `/tmp/micro_ros_agent_uavN.log`:

- Nothing at all → SITL never reached the agent. Check
  `models/iris_N/model.sdf` uses `<listen_addr>172.31.N.1</listen_addr>`, not
  `127.0.0.1`. A wrong address makes SITL block forever waiting for its first
  FDM frame.
- `session established` + `participant created` then silence → the XRCE
  handshake was lost to channel contention. Raise `DDS_SETTLE_SEC`, or reduce
  steady-state DDS load (`AP_DDS_config.h` topic rates).

**A UAV flies but reports no GPS mid-mission**

`session re-established` in the agent log means the DDS session collapsed
(3 missed 500 ms pings). `drone_bridge` falls back to MAVLink automatically;
you will see one warning and one recovery line.

**Mission "completes" with every waypoint skipped**

`best inf` in `/tmp/city_mission.log` means not one fresh GPS sample arrived.
Check the SNR CSV to distinguish a genuine RF outage from a DDS failure.

**`git status` always dirty after a build**

Build outputs are gitignored; if they show up they were committed by accident.
`git rm -r --cached ros2/install ros2/log` fixes it.

---

## 9. File map

```
config/fleet.yaml          THE config — fleet size, addressing, RF, spawns
generated/                 Derived; regenerated every launch, do not edit
  fleet.json               Resolved fleet (used by the ns-3 topology script)
  params/                  Per-UAV DDS .parm files
  worlds/                  Generated world for the current fleet size

scripts/
  launch_city_dynamic_clustering.sh   Main launcher
  validate_fleet_scaling.py           Pre-flight check across fleet sizes
  generate_fleet.py                   fleet.yaml -> fleet.json
  generate_dds_params.py              -> generated/params/
  generate_gazebo_fleet.py            template + fleet -> world, clones iris_N
  setup_ns3_wireless_topology.sh      Namespaces, veths, TAPs
  waypoint_finder.py                  Gazebo XYZ -> GPS

ros2/uav_controller/uav_controller/
  drone_bridge.py            Per-UAV ROS API (MAVLink + AP_DDS)
  city_mission.py            The mission
  dynamic_cluster_manager.py Election + relay policy
  cluster_relay_routes.py    Linux /32 relay routing

ns3/three_uav_tapbridge_integrated.cc   Channel model (sync to $NS3_ROOT/scratch/)
models/iris_N/             Per-UAV Gazebo model (auto-cloned as needed)
worlds/                    World templates
```

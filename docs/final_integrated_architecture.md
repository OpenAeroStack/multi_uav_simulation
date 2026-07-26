# Final integrated architecture

This is the authoritative architecture for the active launcher:

```text
scripts/launch_city_dds.sh
```

Other launchers and predecessor NS-3 scenarios are retained for history and
are not part of this architecture.

## Component diagram

```text
ROOT NETWORK NAMESPACE
  Gazebo Classic 11
    city_3uav.world
    libgazebo_ros_state.so ───────────────▶ /gazebo/model_states
    libobstacle_raycast_plugin.so ────────▶ /link_obstacle_loss
                  ▲                                  │
                  │ /uav_world_positions             ▼
  world_pos_publisher.py ────────────────▶ integrated NS-3
            │                              three_uav_tapbridge_integrated
            └────────▶ obstacle plugin       ├─ /ns3_link_rssi
                                               └─ /ns3_link_snr

  Gazebo ◀════ isolated 172.31.N.0/30 management veths ════▶ SITL

  NS-3 node 0 ◀─ tap-gcs  ─ br-gcs  ─ veth ─▶ gcsns/wifi0
  NS-3 node 1 ◀─ tap-uav1 ─ br-uav1 ─ veth ─▶ uav1/wifi0
  NS-3 node 2 ◀─ tap-uav2 ─ br-uav2 ─ veth ─▶ uav2/wifi0
  NS-3 node 3 ◀─ tap-uav3 ─ br-uav3 ─ veth ─▶ uav3/wifi0

GCSNS
  micro-ROS agents ×3
  drone_bridge ×3
  city_mission

UAV NAMESPACES
  uav1: ArduCopter SITL instance 0
  uav2: ArduCopter SITL instance 1
  uav3: ArduCopter SITL instance 2
```

## Namespace map

| Namespace | Processes | Wireless interface | Management interface |
|---|---|---|---|
| root | Gazebo, integrated NS-3, world-position publisher | No wireless IP | `sim-uav1-host`, `sim-uav2-host`, `sim-uav3-host` |
| `gcsns` | 3 micro-ROS Agents, 3 drone bridges, city mission | `wifi0` | None |
| `uav1` | SITL instance 0 | `wifi0` | `sim0` |
| `uav2` | SITL instance 1 | `wifi0` | `sim0` |
| `uav3` | SITL instance 2 | `wifi0` | `sim0` |

The legacy namespace names `uav1ns`, `uav2ns`, and `uav3ns` are not used by
the active launcher.

## IP map

| Endpoint | Wireless IP | Management IP |
|---|---|---|
| GCS | `10.42.0.10/24` | — |
| UAV1 | `10.42.0.11/24` | `172.31.1.2/30` |
| UAV2 | `10.42.0.12/24` | `172.31.2.2/30` |
| UAV3 | `10.42.0.13/24` | `172.31.3.2/30` |
| Gazebo side for UAV1 | — | `172.31.1.1/30` |
| Gazebo side for UAV2 | — | `172.31.2.1/30` |
| Gazebo side for UAV3 | — | `172.31.3.1/30` |

Root intentionally has no `10.42.0.0/24` address and no route for that subnet.
The wireless endpoints therefore have no root-kernel routing bypass around
NS-3.

## MAC map

| Endpoint | `wifi0` MAC | TAP MAC |
|---|---|---|
| GCS | `02:00:00:00:00:00` | `02:aa:00:00:00:10` |
| UAV1 | `02:00:00:00:00:01` | `02:aa:00:00:00:11` |
| UAV2 | `02:00:00:00:00:02` | `02:aa:00:00:00:12` |
| UAV3 | `02:00:00:00:00:03` | `02:aa:00:00:00:13` |

## TAP, veth and bridge map

| Namespace endpoint | Root veth | Bridge | TAP | NS-3 node |
|---|---|---|---|---:|
| `gcsns/wifi0` | `veth-gcs-host` | `br-gcs` | `tap-gcs` | 0 |
| `uav1/wifi0` | `veth-uav1-host` | `br-uav1` | `tap-uav1` | 1 |
| `uav2/wifi0` | `veth-uav2-host` | `br-uav2` | `tap-uav2` | 2 |
| `uav3/wifi0` | `veth-uav3-host` | `br-uav3` | `tap-uav3` | 3 |

Each bridge contains exactly its veth and TAP, has STP disabled, and carries
no root IP address. Management pairs are separate:

```text
sim-uav1-host (172.31.1.1) ◀────▶ uav1/sim0 (172.31.1.2)
sim-uav2-host (172.31.2.1) ◀────▶ uav2/sim0 (172.31.2.2)
sim-uav3-host (172.31.3.1) ◀────▶ uav3/sim0 (172.31.3.2)
```

## Complete port map

| Function | Listener | Protocol | Destination/source direction |
|---|---|---|---|
| UAV1 DDS | `gcsns` `10.42.0.10:2019` | UDP | UAV1 AP_DDS → Agent |
| UAV2 DDS | `gcsns` `10.42.0.10:2020` | UDP | UAV2 AP_DDS → Agent |
| UAV3 DDS | `gcsns` `10.42.0.10:2021` | UDP | UAV3 AP_DDS → Agent |
| UAV1 MAVLink | `uav1` `0.0.0.0:5760` | TCP | GCS bridge → `10.42.0.11:5760` |
| UAV2 MAVLink | `uav2` `0.0.0.0:5770` | TCP | GCS bridge → `10.42.0.12:5770` |
| UAV3 MAVLink | `uav3` `0.0.0.0:5780` | TCP | GCS bridge → `10.42.0.13:5780` |
| UAV1 servo input | Gazebo `172.31.1.1:9002` | UDP | SITL → Gazebo |
| UAV1 FDM input | SITL `172.31.1.2:9003` | UDP | Gazebo → SITL |
| UAV2 servo input | Gazebo `172.31.2.1:9012` | UDP | SITL → Gazebo |
| UAV2 FDM input | SITL `172.31.2.2:9013` | UDP | Gazebo → SITL |
| UAV3 servo input | Gazebo `172.31.3.1:9022` | UDP | SITL → Gazebo |
| UAV3 FDM input | SITL `172.31.3.2:9023` | UDP | Gazebo → SITL |
| Gazebo master | root `:11345` | TCP | Gazebo clients → master |

Additional source evidence and live verification commands are in
`docs/current_network_and_port_mapping.md`.

## ROS 2 topic map

All integration topics use `ROS_DOMAIN_ID=0`.

| Topic | Publisher | Subscribers | Payload |
|---|---|---|---|
| `/gazebo/model_states` | Gazebo ROS state plugin | world-position publisher | Gazebo model poses |
| `/uav_world_positions` | world-position publisher | obstacle plugin, NS-3 | `[id,x,y,z,...]`, IDs 0–3 |
| `/link_obstacle_loss` | obstacle plugin | NS-3 | `[a,b,loss_dB,...]`, six links |
| `/ns3_link_rssi` | integrated NS-3 | validation/monitoring | `[a,b,rssi_dBm,...]` |
| `/ns3_link_snr` | integrated NS-3 | validation/monitoring | `[a,b,snr_dB,...]` |
| `/ap/vN/navsat` and AP DDS topics | SITL/AP_DDS through Agents | mission/bridges | namespaced vehicle telemetry |

## Gazebo data flow

1. Gazebo loads `worlds/city_3uav.world`, the three ArduPilot UAV models, the
   real `gcs` model, ROS state plugin, and obstacle ray-cast plugin.
2. Gazebo state publishes model poses.
3. The root world-position publisher maps `gcs`, `iris_1`, `iris_2`, and
   `iris_3` to node IDs 0, 1, 2, and 3.
4. Gazebo physics exchanges UDP servo/FDM packets with each SITL over its
   dedicated management /30. This traffic does not enter NS-3.

## DDS data flow

```text
SITL AP_DDS client in uavN
  -> uavN/wifi0
  -> veth/bridge/tap
  -> integrated NS-3 Wi-Fi
  -> tap-gcs/bridge/veth
  -> gcsns Agent UDP 2019/2020/2021
```

DDS crosses NS-3. No management link or root route carries DDS.

## MAVLink data flow

```text
drone_bridge in gcsns
  -> gcsns/wifi0
  -> tap-gcs
  -> integrated NS-3 Wi-Fi
  -> tap-uavN
  -> uavN/wifi0
  -> SITL TCP 5760/5770/5780
```

MAVLink crosses NS-3. The bridges do not use the management IPs.

## Obstacle-to-packet-effect data flow

```text
tagged city collision geometry
  -> Gazebo ray intersection for all six links
  -> material-dependent /link_obstacle_loss
  -> NS-3 DynamicObstacleLossModel
       EMA + block/clear hysteresis
       LoS m=3.0 / NLoS m=1.0
  -> Wi-Fi PHY receive power and decode outcome
  -> real TAP-carried DDS/MAVLink/ping packets
  -> /ns3_link_rssi, /ns3_link_snr and validation CSV
```

FlowMonitor is intentionally not used in the active TapBridge path because
UseLocal traffic bypasses the NS-3 IPv4 probes. PHY traces and real namespace
traffic are authoritative.

## Startup sequence

1. Validate DDS parameter files and acquire cleanup authority.
2. Remove previous launcher-owned processes/topology.
3. Create four isolated wireless endpoints and three management links.
4. Build and start `three_uav_tapbridge_integrated` in root.
5. Wait for all four TAPs and run wireless positive checks.
6. Start Gazebo in root with `city_3uav.world`.
7. Wait for `/gazebo/model_states`.
8. Start `world_pos_publisher.py` in root.
9. Validate the complete obstacle pipeline and NS-3 integration check.
10. Start three micro-ROS Agents in `gcsns`.
11. Start SITL instances in `uav1`, `uav2`, and `uav3`.
12. Validate DDS GPS, then start three drone bridges in `gcsns`.
13. Revalidate the obstacle pipeline.
14. Start `city_mission` in `gcsns`.

## Shutdown sequence

1. Trap launcher exit/signals and gather saved process roots.
2. Stop mission, bridges, Agents, SITL, world-position publisher, Gazebo, and
   NS-3 by tracked PID trees.
3. Finalize or recover NetAnim output.
4. Delete namespaces.
5. Delete wireless bridges, TAPs, wireless veths, and management veths.
6. Preserve `/tmp` component logs and timestamped validation results.

The obstacle plugin separately disconnects its Gazebo update callback, cancels
its owned ROS executor, joins its thread, and never calls global
`rclcpp::shutdown()`.

## Legacy files

These are retained but inactive:

| Path/reference | Status |
|---|---|
| `ns3/three_uav_tapbridge_rt.cc` | Original three-node predecessor |
| `ns3/three_uav_tapbridge_obstacle_loss.cc` | Obstacle-aware predecessor |
| CMake targets `three-uav`, `three_uav_tapbridge_obstacle_loss` | Explicitly labelled legacy; not launched |
| `ns3/ns3_ros2_bridge.*`, `/tmp/ns3_uav_bridge.sock` | Superseded Unix-socket bridge |
| `ros2/.../ns3_ros2_bridge.py` | Superseded bridge node |
| `launch/launch_city_dds.sh` | Older launcher; active launcher is under `scripts/` |
| `launch/launch_multi_dds.sh`, `launch/launch_multi_uav_netns.sh` | Older launch workflows |
| `scripts/setup_netns_tap.sh`, `scripts/test_scripts/verify_datapath.sh`, `iperf3_channel_test.sh` | Older `uavNns` topology tools |
| `scripts/launch_city_dds.sh.save` | Saved historical launcher |
| `architecture.md`, `cluster_architecture.md`, older sections of `README.md` and `SETUP.md` | Historical design narrative |

The active launcher contains legacy process-name patterns only to clean up
stale processes from older runs. Those patterns are not executable target
selection.

## Known limitations

- Full runtime evidence requires root privileges, installed ArduPilot/Gazebo
  dependencies, and a live unified run.
- The final audit environment could not grant non-interactive `sudo`; therefore
  namespace creation, TAP attachment, live port inspection, and topic payload
  readiness remain runtime-unconfirmed. Configuration-only evidence is not
  treated as a live pass.
- Obstacle-effect validation requires physically moving a UAV behind a tagged
  collision object; it correctly fails when no transition is observed.
- The integrated source treats `simTime=0` as a live run but schedules a
  24-hour stop internally.
- RSSI/SNR include stochastic fading; validation compares state medians rather
  than expecting individual samples to decrease monotonically.
- Model-level RF tags apply one material class to a collision entity; mixed
  facade materials are classified conservatively.
- Python setup artifacts under `ros2/install` can become stale after moving or
  partially rebuilding the workspace and may require a clean package rebuild.
- Retained legacy files `ns3/three_uav_tapbridge_rt.cc` and
  `scripts/setup_netns_tap.sh` contain historical merge-conflict markers. They
  are excluded from the active build and launcher. The active `setup.sh`
  conflict was resolved during the final audit.

## Final audit evidence (2026-07-25)

| Audit command/check | Result | Evidence |
|---|---|---|
| Active launcher/setup/topology shell syntax | PASS | `bash -n setup.sh scripts/launch_city_dds.sh scripts/setup_ns3_wireless_topology.sh` exited 0 |
| Active Python syntax | PASS | `python3 -m py_compile` exited 0 for the world publisher, obstacle validator, and controller modules |
| Gazebo plugin package build | PASS | `colcon build --packages-select multi_uav_gazebo_plugins --cmake-clean-cache`; one package finished |
| Controller package build | PASS | `colcon build --packages-select uav_controller`; one package finished |
| Plugin artifact | PASS | `install/multi_uav_gazebo_plugins/lib/libobstacle_raycast_plugin.so` exists |
| Integrated NS-3 target build | PASS | `./ns3 build three_uav_tapbridge_integrated`; Ninja reported no work and exited 0 |
| Network-only positive and negative-control validation | BLOCKED | Launcher configuration checks passed, then `sudo -v` could not obtain credentials in the non-interactive audit environment |
| Obstacle-pipeline live validation | FAIL | Validator exited 1 and wrote `test_logs/obstacle_effect_20260725T104242_2/summary_report.md`; no live launcher and no usable sudo authorization |

The network-only attempt also discovered unresolved conflict markers in the
active `setup.sh`. The conflict was corrected to retain the portable
`ARDUPILOT_HOME` autodetection, and `bash -n` subsequently passed. No topology
was created before either privileged validation attempt failed.

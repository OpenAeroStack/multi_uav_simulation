# Mech-workshop experiment logger

`experiment_logger.py` is a passive, standalone logger for the single-UAV
validation workflow. It does not publish commands, call services, or open a
second MAVLink connection. No ROS package build is required.

## Run sequence

Terminal 1 — launch the complete simulation and leave it running:

```bash
cd /home/randilsk/FYP/multi_uav_sim
bash scripts/ground_truth_test/launch_mech_workshop_single_uav.sh
```

Terminal 2 — start the logger **inside `gcsns`** after the launcher is ready:

```bash
sudo ip netns exec gcsns sudo -H -u randilsk bash -lc '
source /opt/ros/humble/setup.bash
source /home/randilsk/ardu_ws/install/setup.bash
source /home/randilsk/FYP/multi_uav_sim/ros2/install/setup.bash

exec python3 /home/randilsk/FYP/multi_uav_sim/scripts/ground_truth_test/experiment_logger.py
'
```

Terminal 3 — run the mission using the same `gcsns` ROS environment:

```bash
sudo ip netns exec gcsns sudo -H -u randilsk bash -lc '
source /opt/ros/humble/setup.bash
source /home/randilsk/ardu_ws/install/setup.bash
source /home/randilsk/FYP/multi_uav_sim/ros2/install/setup.bash

exec python3 /home/randilsk/FYP/multi_uav_sim/scripts/ground_truth_test/test_cruise.py
'
```

After landing, press Ctrl+C in Terminal 2. The logger closes every CSV,
finalizes `metadata.json`, and prints timing statistics.

## Output layout

Each invocation creates a new directory and never overwrites an earlier run:

```text
scripts/ground_truth_test/experiment_logs/
└── run_2026-09-02_14-35-21/
    ├── metadata.json
    ├── vehicle_state.csv
    ├── gps.csv
    ├── mission.csv
    ├── telemetry_timing.csv
    ├── network.csv
    └── events.csv
```

## Selected measurements

| Measurement | Source | Reason |
|---|---|---|
| GPS trajectory | `/ap/v1/navsat` (`sensor_msgs/NavSatFix`) | Measured ArduPilot DDS GPS after delivery through ns-3 to the GCS-side agent; avoids commanded positions. |
| Local pose, attitude and quaternion | `/ap/v1/pose/filtered` (`geometry_msgs/PoseStamped`) | ArduPilot-filtered state in documented ROS ENU coordinates. |
| Velocity and groundspeed | `/ap/v1/twist/filtered` (`geometry_msgs/TwistStamped`) | Measured ENU velocity; groundspeed is derived as `sqrt(vx²+vy²)`. |
| Relative altitude and application telemetry continuity | `/uav1/rel_alt` (`std_msgs/Float32`) | `drone_bridge` emits one message for each received MAVLink `GLOBAL_POSITION_INT`, making it the closest safe receiving-side continuity stream to the real TLOG message. |
| Armed/flying state | `/uav1/armed` and `/ap/v1/status` | Existing receiving-side state topics. |
| Human-readable mode | `/uav1/mode` (`std_msgs/String`) | Existing bridge mode output. |
| Commanded GUIDED target | `/uav1/goto` (`geographic_msgs/GeoPoint`) | Records exactly what the independent mission publishes. |
| ArduPilot target | `/ap/v1/goal_lla` (`geographic_msgs/GeoPointStamped`) | Preserves the target reported by the autopilot. |
| Physical link samples | `/tmp/ns3_snr_mw.csv` | Passive per-received-frame signal/noise/SNR source already written by ns-3. |

`local_x_east_m`, `local_y_north_m`, and `local_z_up_m` are ENU and are never
mixed with NED. ENU yaw is counter-clockwise from East; `heading_deg` is
separately converted to clockwise from North. GPS-derived East/North values
use a WGS-84 local tangent approximation about `/ap/v1/gps_global_origin/filtered`,
falling back to the configured launch pad `6.0773722, 80.1907552` until that
topic is received.

## Parameters

The defaults match this launcher. Parameters can be overridden without editing
the script, for example:

```bash
python3 experiment_logger.py --ros-args \
  -p vehicle_id:=1 \
  -p notes:="repeat 2, clear weather" \
  -p output_directory:=/home/randilsk/FYP/multi_uav_sim/experiment_logs
```

Topic names, GPS origin, GCS ENU antenna position, arrival radius, telemetry
gap threshold, and ns-3 SNR file path are also ROS parameters declared in the
script.

## Deliberately unavailable or excluded data

- `drone_bridge` preserves the arrival of each `GLOBAL_POSITION_INT` through
  `/uav1/rel_alt`, but it does not republish the MAVLink sequence number,
  component ID, or `time_boot_ms`. The corresponding CSV fields remain empty.
  A second pymavlink TCP client is intentionally not opened because it could
  compete with `drone_bridge` and change the experiment.
- Timing gaps are logged as timing gaps, never as packet loss. ROS message loss
  is not labelled wireless loss.
- `network.csv` contains only values genuinely present in the ns-3 SNR file,
  plus distance derived from the latest measured ENU pose and configured GCS
  antenna position. It does not invent transmitter identity, packet-loss,
  RTT, throughput, or application delivery counters.
- ns-3 does not flush its SNR file after every row. Consequently, wall time in
  `network.csv` is the time a completed row became observable to the logger and
  may be shared by a buffered batch; raw `ns3_time_s` is retained unchanged.
- The mission uses repeated GUIDED targets rather than an uploaded MAVLink
  waypoint mission, so `current_waypoint` is intentionally blank. Target
  revisions, measured distance, closest approach, and arrival events remain
  available for analysis.
- A trustworthy Gazebo `/clock` is not visible inside `gcsns`. The ArduPilot
  DDS clock is retained as `source_clock_s`, and ns-3 time is retained as
  `ns3_time_s`; neither is silently treated as computer wall time.

# Three-UAV comparison-pose mission

This mission controls UAV1, UAV2, and UAV3 from one ROS 2 node inside
`gcsns`. It uses the existing GCS-side `drone_bridge` interfaces created by
`scripts/netns_3uav/launch_three_uav_netns.sh`.

## Mission phases

1. Verify GPS, relative altitude, armed state, arm/takeoff services, goto
   subscribers, and optional yaw interfaces for all three UAVs.
2. Arm and call the existing Trigger takeoff service for all UAVs. The bridge
   takeoff target defaults to 10 m; the mission then commands all three to
   climb vertically and stabilize at 25 m.
3. Stage UAV3 10 m north, wait for stability, then stage UAV2 10 m south.
   UAV1 holds its runtime horizontal position.
4. Check staging separation, final separation, and the closest approaches of
   the three planned straight-line horizontal segments.
5. Acquire the final formation using a **staggered concurrent formation
   acquisition**:

   ```text
   UAV3 released first
       ↓
   clears UAV1 path-conflict region
       ↓
   UAV1 released while UAV3 continues
       ↓
   safe remaining-path condition reached
       ↓
   UAV2 released
       ↓
   all three travel concurrently
       ↓
       all final GPS positions stable
       ↓
       UAV2 10 m south / UAV3 10 m north
       ↓
       both return to their final GPS targets
       ↓
   experiment ready
   ```

   Releases are based on live GPS position, projected path progress, and
   geometric clearance—not fixed sleep delays. Actual pairwise separation is
   monitored throughout; a violation prevents any unreleased UAV from being
   released but does not replace the existing target-hold behavior with a new
   collision controller.
6. Require independent position stability at all three final GPS targets.
7. Run a position-only heading-correction maneuver: UAV2 travels 10 m south
   and UAV3 10 m north, then both return to the unchanged final GPS targets.
   Completion requires position stability only; no yaw command or yaw
   feedback is required.
8. Continue refreshing the final targets indefinitely.

## Final manually calibrated conference experiment poses

The GPS positions were selected with `scripts/mission/waypoint_finder.py`.
The final GPS positions are unchanged. The short north/south return legs provide
the desired final travel directions without relying on the unsupported direct
yaw-control path. The mission does not calculate headings from an automatic GPS
target bearing.

Only position and yaw are commanded. Yaw uses `/ap/vN/cmd_gps_pose` with the
existing `GlobalPosition` interface; roll and pitch remain under ArduPilot
position-control authority.

| UAV | Latitude | Longitude | Altitude | Yaw |
|-----|----------|-----------|----------|-----|
| UAV1 | 6.079429 | 80.193085 | 25.0 m | Position only |
| UAV2 | 6.079339 | 80.193222 | 25.0 m | Return north after temporary south leg |
| UAV3 | 6.079609 | 80.193214 | 25.0 m | Return south after temporary north leg |

## Dry run

Start the three-UAV launcher first. Then, from another host terminal:

```bash
sudo ip netns exec gcsns sudo -H -u multi_uav bash -lc '
source /opt/ros/humble/setup.bash
source /home/multi_uav/FYP/ardu_ws/install/setup.bash
source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
cd /home/multi_uav/FYP/multi_uav_simulation
python3 scripts/three_uav_experiment/goto_three_uav_comparison_poses.py \
  --staging-distance 10.0 --minimum-separation 5.0 \
  --dry-run
'
```

Dry-run discovers the real interfaces and reads current GPS/heading state, but
does not call arm/takeoff and does not publish goto or yaw commands. It prints
the staging and final positions, path intersections/closest approaches, the
UAV1/UAV3 conflict location and UAV3 release distance, UAV2 release analysis,
final separation, expected release sequence, and all three calibrated poses.

## Actual run

```bash
sudo ip netns exec gcsns sudo -H -u multi_uav bash -lc '
source /opt/ros/humble/setup.bash
source /home/multi_uav/FYP/ardu_ws/install/setup.bash
source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
cd /home/multi_uav/FYP/multi_uav_simulation
python3 scripts/three_uav_experiment/goto_three_uav_comparison_poses.py \
  --staging-distance 10.0 \
  --horizontal-tolerance 1.0 --vertical-tolerance 0.75 \
  --stable-seconds 5.0 --minimum-separation 5.0 \
  --timeout 1000
'
```

The calibrated altitude is the default. `--alt` remains an optional common
debugging override; the conference experiment uses 25.0 m.

If the geometric check rejects a path or a post-takeoff phase fails, the node
keeps refreshing its last safe targets where possible. It does not
automatically land or disarm. Ctrl+C stops only this mission node and issues no
land, disarm, SITL, Gazebo, ROS, or NS-3 shutdown command.

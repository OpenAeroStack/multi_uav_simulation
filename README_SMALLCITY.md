# Small City World — Setup Guide

How to get the `small_city` Gazebo world and the 3-UAV city mission running on a
new machine. Assumes the base simulation stack (ROS 2 Humble, Gazebo Classic 11,
ArduPilot SITL with AP_DDS, and the `multi_uav_sim` repo) is already installed
and the airport mission works.

## 1. What you need

Two folders side by side under `~/FYP/`:

```
~/FYP/
├── multi_uav_sim/        # main project repo (missions, launch, models)
└── small_city_gazebo/    # city world asset package (this package)
```

`small_city_gazebo` contents:

```
small_city_gazebo/
├── worlds/
│   └── small_city_base.world    # original world (unmodified)
├── models/                      # all 38 model dependencies
├── city_terrain/                # pre-built heightmap paging cache
└── README.md
```

## 2. Copy the asset package

Copy the whole `small_city_gazebo` folder to the new machine:

```bash
# from the old machine
scp -r ~/FYP/small_city_gazebo user@newmachine:~/FYP/
```

or zip it and transfer however you like. No installation step — it is just
models and a world file.

## 3. Copy the heightmap cache (important — avoids slow first load)

The `city_terrain/` folder inside the package holds Gazebo's pre-built
heightmap cache. Copy it into Gazebo's paging directory on the new machine:

```bash
mkdir -p ~/.gazebo/paging
cp -r ~/FYP/small_city_gazebo/city_terrain ~/.gazebo/paging/
```

Without this, the first launch takes noticeably longer while Gazebo rebuilds
the terrain cache (it is fast on every run after that, so this step is a
nice-to-have, not a hard requirement).

> Note: if the terrain ever renders as yellow/black stripes, the cache is
> corrupted. Fix: `rm -rf ~/.gazebo/paging/city_terrain`, launch the world
> once so Gazebo rebuilds it, then re-copy the fresh cache into the package.

## 4. Environment variables

The launch script (`launch/launch_city_dds.sh`) already exports these, but for
manually running the world you need:

```bash
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=~/FYP/small_city_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=~/FYP/small_city_gazebo:$GAZEBO_RESOURCE_PATH
```

- `GAZEBO_MODEL_DATABASE_URI=""` stops Gazebo phoning the online model
  database on every start (removes a long startup delay).
- Make sure `~/.bashrc` does **not** also append the old
  `gazebo_models_worlds_collection` paths — duplicate/stale paths cause
  broken texture lookups.

## 5. Sanity test — world only, no SITL

```bash
gazebo --verbose ~/FYP/small_city_gazebo/worlds/small_city_base.world
```

Expected: city loads with buildings, roads, pond, mountain; heightmap loads in
well under a second (`Loading heightmap cache data: ...`); no
`Unable to resolve uri` errors for city models. The `model://gazebo` error and
the OBJLoader `d`/`Tr` warnings are known-harmless and can be ignored.

## 6. Run the full 3-UAV city mission

The mission world (`city_3uav.world`) and everything below live in the main
repo, not this package.

```bash
# Terminal 1 — Gazebo + SITL x3 + micro_ros_agent x3 + drone_bridge x3
bash ~/FYP/multi_uav_sim/launch/launch_city_dds.sh
```

Wait until all three bridges print `✓ DDS GPS flowing`, then:

```bash
# Terminal 2 — the mission
source /opt/ros/humble/setup.bash
source ~/FYP/multi_uav_sim/ros2/install/setup.bash
ros2 run uav_controller city_mission
```

Mission profile: all 3 UAVs take off from the stage area (south of the city),
UAV1 (cluster head) flies to the city centre, does a circle patrol and hovers
as relay at 60 m; UAV2 sweeps the pond zone at 40 m; UAV3 sweeps the mountain
zone at 50 m; all RTL (cluster head last).

Monitor:

```bash
ros2 topic echo /cluster/status                 # cluster telemetry
ros2 run rqt_image_view rqt_image_view          # pick /cluster/cam/uav1|2|3
```

## 7. Ground-truth GPS waypoints

The mission's waypoints were captured by flying to each location and reading
ArduPilot's own GPS (not estimated from world coordinates):

| Location    | Latitude   | Longitude    |
|-------------|------------|--------------|
| Spawn/stage | 37.337570  | -121.886055  |
| City centre | 37.338562  | -121.886223  |
| Pond        | 37.340176  | -121.887016  |
| Mountain    | 37.340092  | -121.884521  |

To find new waypoints, use the keyboard flight tool:

```bash
python3 ~/FYP/multi_uav_sim/scripts/waypoint_finder.py --uav 1 --alt 40
# t = takeoff, w/a/s/d = move 10m, W/A/S/D = 100m, r/f = alt ±5m, p = print GPS
```

## 8. Common issues

| Symptom | Fix |
|---|---|
| `Address already in use` on launch | `pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f arducopter; sleep 3` then relaunch |
| Yellow/black striped terrain | Corrupted heightmap cache — see note in step 3 |
| `Unable to resolve uri [model://...]` for city models | `GAZEBO_MODEL_PATH` missing the package — re-check step 4 |
| Very slow first load / "Waiting for model database update" | `GAZEBO_MODEL_DATABASE_URI=""` not set before launch |
| Drones don't arm / EKF errors | World physics must be `max_step_size=0.004`, `real_time_update_rate=-1`, and `spherical_coordinates` present — `city_3uav.world` already has these |

## Credits

World and models originate from
[gazebo_models_worlds_collection](https://github.com/mlherd/gazebo_models_worlds_collection)
(GPLv3), which aggregates assets from 3DGEMS, RotorS, TU Delft, ARTI-Robots,
Clearpath Robotics, and Fetch Robotics. See the package README for details.
# Adding More Drones to the Simulation

This guide walks through adding a 4th drone (UAV4) to the existing 3-drone setup.
The same pattern repeats for UAV5, UAV6, and so on.

---

## How the System Works (Quick Overview)

Each drone has four linked pieces that must all match:

```
Gazebo model  ←→  ArduPilot SITL  ←→  micro_ros_agent  ←→  ROS2 drone_bridge node
(physics)         (flight control)     (DDS bridge)          (your mission code talks to this)
```

- **Gazebo** simulates the physical drone using unique FDM ports.
- **ArduPilot SITL** runs the flight controller and connects to Gazebo.
- **micro_ros_agent** receives DDS telemetry from ArduPilot and passes it into ROS2.
- **drone_bridge** is a ROS2 node that exposes simple topics and services (`/uav4/arm`, `/uav4/takeoff`, etc.) for your mission scripts.

---

## Port Mapping Pattern

For drone number `N` (starting from 1):

| What            | Formula                  | UAV1  | UAV2  | UAV3  | UAV4  |
|-----------------|--------------------------|-------|-------|-------|-------|
| SITL instance   | `-I(N-1)`                | `-I0` | `-I1` | `-I2` | `-I3` |
| System ID       | `N`                      | 1     | 2     | 3     | 4     |
| MAVLink TCP port| `5760 + 10*(N-1)`        | 5760  | 5770  | 5780  | 5790  |
| DDS UDP port    | `2019 + (N-1)`           | 2019  | 2020  | 2021  | 2022  |
| Gazebo FDM in   | `9002 + 10*(N-1)`        | 9002  | 9012  | 9022  | 9032  |
| Gazebo FDM out  | `9003 + 10*(N-1)`        | 9003  | 9013  | 9023  | 9033  |

---

## Step 1 — Create the Gazebo Model

Copy an existing model folder:

```bash
cp -r models/iris_3 models/iris_4
```

Then open `models/iris_4/model.sdf` and update the FDM ports in the `libArduPilotPlugin.so` section:

```xml
<fdm_port_in>9032</fdm_port_in>
<fdm_port_out>9033</fdm_port_out>
```

Also open `models/iris_4/model.config` and change the `<name>` tag to `iris_4`.

---

## Step 2 — Add the Drone to the World File

Open `worlds/multi_uav.world` and add a new model block near the other drones:

```xml
<!-- UAV4 — SITL instance 3 — FDM ports 9032/9033 -->
<model name="iris_4_demo">
  <pose>6 0 0 0 0 0</pose>
  <include>
    <uri>model://iris_4</uri>
  </include>
</model>
```

Space the drones 2 metres apart along the X axis (`x = 0, 2, 4, 6, ...`).

---

## Step 3 — Create the DDS Parameter File

Create `params/uav4_dds.parm`:

```
DDS_ENABLE 1
DDS_UDP_PORT 2022
DDS_DOMAIN_ID 0

ARMING_CHECK 0
```

This tells ArduPilot to publish telemetry via DDS on port 2022, which the micro_ros_agent will listen on.

---

## Step 4 — Update the Launch Script

Open `launch/launch_multi_dds.sh` and make three additions:

**Add the parameter file variable** (near the top with the other UAV_DEFAULTS lines):
```bash
UAV4_DEFAULTS="$BASE,$PROJECT_DIR/params/uav4_dds.parm"
```

**Add a micro_ros_agent instance** (in section [2/4]):
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2022 &
sleep 1
```

**Add a SITL instance** (in section [3/4]):
```bash
$BINARY --model gazebo-iris --speedup 1 --sysid 4 \
    --defaults $UAV4_DEFAULTS --sim-address=127.0.0.1 -I3 &
sleep 2
```

**Add a drone_bridge node** (in section [4/4]):
```bash
ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=4 \
    -p mavlink_port:=5790 \
    -p takeoff_altitude:=20.0 &
sleep 1
```

---

## Step 5 — Add UAV4 to Your Mission Script

If you are using the ROS2 mission node (recommended), add UAV4 to the list of drones it controls. The bridge will automatically expose these ROS2 topics and services:

| Topic / Service       | Type                         | Description              |
|-----------------------|------------------------------|--------------------------|
| `/uav4/gps`           | `sensor_msgs/NavSatFix`      | GPS position             |
| `/uav4/rel_alt`       | `std_msgs/Float32`           | Altitude above home (m)  |
| `/uav4/battery`       | `std_msgs/Float32`           | Battery percentage       |
| `/uav4/mode`          | `std_msgs/String`            | Flight mode              |
| `/uav4/armed`         | `std_msgs/Bool`              | Armed state              |
| `/uav4/goto`          | `geographic_msgs/GeoPoint`   | Send a waypoint          |
| `/uav4/arm` (service) | `std_srvs/Trigger`           | Arm the drone            |
| `/uav4/takeoff`       | `std_srvs/Trigger`           | Take off                 |
| `/uav4/land`          | `std_srvs/Trigger`           | Land                     |
| `/uav4/rtl`           | `std_srvs/Trigger`           | Return to launch         |

If you are using the pure MAVLink mission script (`scripts/multi_drone_misison.py`), add one entry to the `DRONES` list:

```python
{"name": "UAV4", "port": 5790, "sysid": 4, "direction": "backward"},
```

The barrier count (`N_DRONES = len(DRONES)`) and thread spawning update automatically.

---

## Quick Checklist

Before running, verify each of these for UAV4:

- [ ] `models/iris_4/` folder exists with updated FDM ports (9032 / 9033)
- [ ] `models/iris_4/model.sdf` has the correct `<fdm_port_in>` and `<fdm_port_out>`
- [ ] `worlds/multi_uav.world` includes `iris_4`
- [ ] `params/uav4_dds.parm` exists with `DDS_UDP_PORT 2022`
- [ ] Launch script adds `micro_ros_agent` on port **2022**
- [ ] Launch script adds SITL with `--sysid 4` and `-I3`
- [ ] Launch script adds `drone_bridge` with `uav_id:=4` and `mavlink_port:=5790`
- [ ] Mission script references port 5790 (or `/uav4/...` topics)
- [ ] No port conflicts with existing drones

---

## Troubleshooting

**Drone does not appear in Gazebo**
- Check that `iris_4` is in `worlds/multi_uav.world` and the model folder exists.

**`drone_bridge` prints "GPS not ready"**
- The micro_ros_agent for that drone is not running or is on the wrong port.
- Check that `DDS_UDP_PORT` in the `.parm` file matches the `--port` in the launch script.

**SITL starts but never connects to Gazebo**
- FDM ports in `model.sdf` do not match the `-I` index. Use the port table above to double-check.

**Multiple drones respond on the same MAVLink port**
- Two SITL instances are using the same `-I` flag. Each drone must have a unique instance index.

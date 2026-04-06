# Adding More Drones to Multi-UAV Simulation

This guide explains how to scale the current 3-UAV setup to 4 or more UAVs.

It covers all required updates in:
- `worlds/multi_uav.world`
- `models/iris_*/model.sdf`
- `launch/launch_multi_uav.sh`
- `scripts/multi_drone_misison.py`

## Current Mapping Pattern

Each drone uses a matching set of IDs/ports based on its instance index.

For drone number `N` (1-based):
- `sysid = N`
- `instance = N - 1` (used by `-I` in SITL)
- MAVLink TCP port = `5760 + 10 * (N - 1)`
- Gazebo FDM in = `9002 + 10 * (N - 1)`
- Gazebo FDM out = `9003 + 10 * (N - 1)`

Examples:
- UAV1: sysid 1, `-I0`, TCP 5760, FDM 9002/9003
- UAV2: sysid 2, `-I1`, TCP 5770, FDM 9012/9013
- UAV3: sysid 3, `-I2`, TCP 5780, FDM 9022/9023
- UAV4: sysid 4, `-I3`, TCP 5790, FDM 9032/9033

## Step 1: Add a New Drone Model Folder

Your world currently references `model://iris_1`, `model://iris_2`, `model://iris_3`.
For UAV4, create `models/iris_4` by copying an existing model folder.

Example:

```bash
cp -r models/iris_3 models/iris_4
```

Then update these files inside `models/iris_4`:

1. `models/iris_4/model.config`
- Change `<name>` to `iris_4`.
- If there are repeated author names like `iris_3`, update them to `iris_4` for clarity.

2. `models/iris_4/model.sdf`
- Keep plugin structure the same.
- In the `libArduPilotPlugin.so` section, update:
  - `<fdm_port_in>` to `9032`
  - `<fdm_port_out>` to `9033`

Important: each drone must have unique FDM ports.

## Step 2: Add the Drone to `multi_uav.world`

Open `worlds/multi_uav.world` and add a new model block:

```xml
<!-- UAV 4 - SITL instance 3 - ports 9032/9033 -->
<model name="iris_4_demo">
  <pose>6 0 0 0 0 0</pose>
  <include>
    <uri>model://iris_4</uri>
  </include>
</model>
```

Notes:
- Keep a safe spacing between drones (`x = 0, 2, 4, 6, ...` works well).
- Ensure model names are unique (`iris_4_demo`, `iris_5_demo`, etc).

## Step 3: Update `launch/launch_multi_uav.sh`

Add a new SITL launch block for each extra drone.

Example for UAV4:

```bash
echo "=== Launching ArduCopter SITL instance 3 (UAV4) ==="
$BINARY --model gazebo-iris --speedup 1 --sysid 4 \
  --defaults $DEFAULTS \
  --sim-address=127.0.0.1 -I3 &
```

Also update:
- Status messages that currently say "3 UAVs".
- MAVProxy connection hints to include port `5790`.

Recommended: put launch blocks inside a loop so scaling is easier.

## Step 4: Update `scripts/multi_drone_misison.py`

Add one entry per new drone in `DRONES`.

Example for UAV4:

```python
{"name": "UAV4", "port": 5790, "sysid": 4, "direction": "backward"},
```

What updates automatically already:
- `N_DRONES = len(DRONES)`
- All barriers (`threading.Barrier`) use `N_DRONES`
- Thread spawning loops over `DRONES`

So the main required script change is adding entries to the `DRONES` list.

## Step 5: Optional MAVProxy Connections

For 4 drones:

```bash
# UAV1
mavproxy.py --master=tcp:127.0.0.1:5760 --logfile=~/mav1.tlog
# UAV2
mavproxy.py --master=tcp:127.0.0.1:5770 --logfile=~/mav2.tlog
# UAV3
mavproxy.py --master=tcp:127.0.0.1:5780 --logfile=~/mav3.tlog
# UAV4
mavproxy.py --master=tcp:127.0.0.1:5790 --logfile=~/mav4.tlog
```

## Quick Checklist Before Running

- New model folder exists (`models/iris_4`, etc).
- New model has unique FDM ports in `model.sdf`.
- New model is included in `worlds/multi_uav.world`.
- Launch script has matching `sysid` and `-I` for each drone.
- Mission script `DRONES` list has matching TCP ports.
- No port collisions between drones.

## Typical Port Collision Symptoms

- Drone does not arm or does not move.
- SITL instance starts but never links to Gazebo.
- Multiple drones respond on the same MAVLink port.

If this happens, verify all three layers match for each UAV:
- world model include
- model FDM ports
- launch script (`sysid`, `-I`)
- mission script TCP port

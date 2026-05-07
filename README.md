# Multi-UAV Simulation Framework

This repository contains a high-performance, containerless simulation
framework for Multi-UAV swarming. It integrates **ArduPilot SITL**,
**Gazebo**, and **ns-3** to create a highly realistic testing
environment.

Unlike standard simulations where multiple drones collide on `localhost`
ports, this architecture uses strictly isolated **Linux Network
Namespaces** and virtual ethernet bridges to give each UAV its own
dedicated IP address, routing table, and MAVLink stream.

------------------------------------------------------------------------

##  Architecture Highlights

To achieve complete network isolation while allowing the 3D physics
engine and network simulator to communicate, this framework implements
several advanced routing techniques.

### Strict Subnet Isolation (`/24`)

Each drone operates on its own dedicated `/24` subnet (e.g.,
`10.42.1.x`, `10.42.2.x`) to prevent routing collisions when commanding
the swarm.

### Two-Way UDP Physics Mirrors (`socat`)

This approach overcomes the hardcoded `127.0.0.1` socket binding in the
ArduPilot Gazebo plugin. The framework uses `socat` to build symmetrical
relays that catch motor commands from the namespace, send them across
the virtual bridge to Gazebo, and seamlessly return the 3D physics data
back to ArduPilot.

### Headless MAVProxy Daemons

Each namespace runs a lightweight background `mavproxy.py` router to
broadcast the drone's telemetry to the standard `14550` port. This
allows a single Python swarm controller script to command the entire
fleet simultaneously.

### Custom SDF Routing

The Gazebo model files (`model.sdf`) are explicitly configured with
`<fdm_addr>` tags pointing to the exact namespace IP address of each
drone.

------------------------------------------------------------------------

##  Prerequisites

Ensure the following dependencies are installed on your Ubuntu machine:

-   **Ubuntu** (native installation recommended)
-   **ArduPilot SITL** (ArduCopter)
-   **Gazebo** (with `libArduPilotPlugin.so`)
-   **ns-3** (Network Simulator 3)
-   **Python 3** (with `pymavlink`)
-   **MAVProxy**
-   **socat**

Install socat if needed:

``` bash
sudo apt-get install socat
```

------------------------------------------------------------------------

##  Quick Start Guide

### 1. Initialize the Network Infrastructure

Before launching any physics or drones, you must build the virtual
bridges, `veth` pairs, and routing tables.

``` bash
cd scripts/
sudo ./setup_network.sh
```

------------------------------------------------------------------------

### 2. Launch the Simulation Stack

The master launch script handles the boot sequence. It will:

-   Start Gazebo with the **3-drone world**
-   Build the **socat two-way UDP mirrors** for all three drones
-   Boot **ArduPilot SITL** inside the network namespaces
-   Launch **MAVProxy daemons** to route telemetry

Run:

``` bash
./launch_multi_uav.sh
```

Wait until the terminal prints:

    === All 3 SITL instances running ===

------------------------------------------------------------------------

### 3. Execute the Autonomous Swarm Mission

Once the drones have booted and acquired a **3D GPS lock**, launch the
central Python controller to orchestrate the flight.

Open a **new terminal window** and run:

``` bash
cd scripts/
python3 multi_drone_mission.py
```

**Note:**\
The mission script utilizes a synchronization **Barrier**. All three
drones must reach the **20.0 m takeoff altitude** before the swarm is
permitted to proceed to Waypoint 1.

------------------------------------------------------------------------

## 🛠️ Common Troubleshooting

### "No route to host"

**Cause:**\
The virtual ethernet bridges do not exist, or there is a subnet
collision.

**Fix:**\
Ensure you ran:

``` bash
sudo ./setup_network.sh
```

Verify the script uses a **/24 subnet mask**, not `/16`.

------------------------------------------------------------------------

### "Connection refused" / "Link 1 Down"

**Cause:**\
MAVProxy cannot connect to ArduPilot's locked `127.0.0.1:5760` port,
usually because ArduPilot crashed or has not finished booting.

**Fix:**\
Ensure you are **not running MAVProxy as root**:

``` bash
sudo -u $USER mavproxy.py
```

------------------------------------------------------------------------

### "Frozen Time" / Drones Won't Arm

**Cause:**\
ArduPilot is not receiving physics data from Gazebo, causing its
internal clock to freeze at `0.00` and preventing MAVLink heartbeats.

**Fix:**

1.  Check the Gazebo GUI to ensure the simulation clock is ticking (not
    paused).
2.  Verify that your drone's `model.sdf` file contains the correct
    `<fdm_addr>` pointing to its specific namespace IP (e.g.,
    `10.42.1.2`).
3.  Confirm the `socat` relays are running in the background.

------------------------------------------------------------------------

##  Repository Structure

    /models/
        Drone SDF files with updated <fdm_addr> plugin targets

    /worlds/
        Gazebo environment files

    /scripts/setup_network.sh
        Generates the Linux network namespaces

    /scripts/launch_multi_uav.sh
        Automated boot sequence and socat relay generation

    /scripts/multi_drone_mission.py
        Pymavlink swarm controller

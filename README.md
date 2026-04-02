# Multi-UAV ArduCopter SITL with Gazebo

This project provides a simulation environment for running multiple ArduCopter instances in SITL (Software-in-the-Loop) with Gazebo. It includes launch scripts, world files, and control scripts for both multi-UAV and single-UAV simulations.

## File Structure

- `launch/`: Contains shell scripts to launch the simulations.
  - `launch_multi_uav.sh`: Launches a 3-drone simulation.
  - `launch_single_uav.sh`: Launches a single drone simulation.
- `scripts/`: Contains Python scripts for controlling the UAVs.
  - `takeoff_all.py`: Connects to the 3 simulated drones, arms them, and commands a synchronized takeoff.
- `worlds/`: Contains Gazebo world files.
  - `multi_uav.world`: A world with 3 Iris quadcopters.
  - `single_uav.world`: A world with a single Iris quadcopter.
- `*.parm`: ArduPilot parameter files. These are examples and not directly used by the default launch scripts.

## Setup

Before running the simulation, ensure you have a working ArduPilot and Gazebo installation. You also need to source the ArduPilot environment script.

1.  **Source the setup script:**
    ```bash
    source ~/setup_ardupilot.sh
    ```

## How to Run

### Multi-UAV Simulation

1.  **Launch the simulation:**
    This script will build the ArduCopter binary, start Gazebo, and launch three SITL instances.

    ```bash
    bash launch/launch_multi_uav.sh
    ```

2.  **Run the takeoff script:**
    In a new terminal (after sourcing the setup script), run the following command to make the drones take off.

    ```bash
    python3 scripts/takeoff_all.py
    ```

    The drones will connect, switch to GUIDED mode, arm, and take off to an altitude of 5 meters.

### Single UAV Simulation

1.  **Launch the simulation:**
    This will start Gazebo and a single SITL instance.

    ```bash
    bash launch/launch_single.sh
    ```

2.  **Control the drone:**
    You can connect to the drone using MAVProxy from another terminal:
    ```bash
    mavproxy.py --master=tcp:127.0.0.1:5760
    ```
    From the MAVProxy command line, you can issue commands like `mode guided`, `arm throttle`, and `takeoff 5`.

## Configuration

### Changing the Number of UAVs

The number of UAVs is hardcoded to 3. To change it, you need to modify the following files:

1.  **`launch/launch_multi_uav.sh`**: Add or remove SITL instance launch blocks. Each drone needs a unique `sysid` and instance number (`-I`).
2.  **`worlds/multi_uav.world`**: Add or remove `<model>` blocks for each drone. Ensure each drone has a unique name and pose.
3.  **`scripts/takeoff_all.py`**: Modify the script to connect to the correct number of drones and MAVLink ports. It is recommended to use a loop instead of hardcoded connections for scalability.

##source file i used in my setup 

file name is setup_ardupilot.sh and below is the content

setup_ardupilot.sh                             
source /usr/share/gazebo-11/setup.bash
source "/home/ubuntu/ardupilot/Tools/completion/completion.bash"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/FYP/gazebo_models_worl>
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/ubuntu/FYP/gazebo_model>
export GAZEBO_MODEL_DATABASE_URI=""
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/ardupilot_gazebo/models

export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/ubuntu/ardupilot_gazebo>

export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/FYP/multi_uav_sim/worlds
echo "ArduPilot environment loaded!"






# multi_uav_sim

A standalone Gazebo + ArduPilot SITL simulation for 3 UAVs. All Gazebo models are bundled in the repo — the only external dependencies are Gazebo 11, ArduPilot, and the ArduPilot Gazebo plugin.

---

## System Requirements

- Ubuntu 20.04 (tested)
- Gazebo Classic 11 (**not** Gazebo Garden/Harmonic/Fortress)
- ArduPilot (built from source)
- ArduPilot Gazebo plugin (`khancyr/ardupilot_gazebo`)
- MAVProxy
- Python 3

---

## Step 1 — Install Gazebo 11

```bash
sudo apt update
sudo apt install gazebo11 libgazebo11-dev
```

Verify:
```bash
gazebo --version
# Should print: Gazebo multi-robot simulator, version 11.x.x
```

---

## Step 2 — Install ArduPilot

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

Install dependencies:
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

Build ArduCopter for SITL:
```bash
./waf configure --board sitl
./waf copter
```

Verify the binary exists:
```bash
ls build/sitl/bin/arducopter
```

---

## Step 3 — Install ArduPilot Gazebo Plugin

```bash
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

Verify the plugin is installed:
```bash
ls /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/ | grep ArduPilot
# Should show: libArduPilotPlugin.so
```

---

## Step 4 — Install MAVProxy

```bash
pip3 install MAVProxy
```

---

## Step 5 — Clone This Repo

```bash
git clone <your-repo-url>
cd multi_uav_sim
```

---

## Step 6 — Configure `setup.sh`

Open `setup.sh` and set `ARDUPILOT_HOME` to wherever you cloned ArduPilot:

```bash
# Edit this line in setup.sh:
export ARDUPILOT_HOME="$HOME/ardupilot"
```

Common paths:
```bash
export ARDUPILOT_HOME="$HOME/ardupilot"                   # installed in home directory
export ARDUPILOT_HOME="/opt/ardupilot"                    # installed in /opt
export ARDUPILOT_HOME="/media/user/drive/ardupilot"       # on external drive (not recommended)
```

> **Note:** Everything else in `setup.sh` is automatic — model paths, resource paths, and the Gazebo database are all configured for you.

---

## Step 7 — Run the Simulation

### Multi UAV (3 drones):
```bash
bash launch/launch_multi_uav.sh
```

### Single UAV:
```bash
bash launch/launch_single.sh
```

The script will:
1. Kill any previous Gazebo/ArduPilot instances
2. Build the ArduCopter binary
3. Launch Gazebo with the world file
4. Launch 3 ArduCopter SITL instances (or 1 for single)

---

## Step 8 — Connect via MAVProxy

Open a new terminal for each UAV:

```bash
# UAV 1
cd ~ && mavproxy.py --master=tcp:127.0.0.1:5760 --logfile=~/mav1.tlog

# UAV 2
cd ~ && mavproxy.py --master=tcp:127.0.0.1:5770 --logfile=~/mav2.tlog

# UAV 3
cd ~ && mavproxy.py --master=tcp:127.0.0.1:5780 --logfile=~/mav3.tlog
```

> **Important:** Always run MAVProxy from your home directory (`cd ~`) to avoid permission errors when writing log files.

---

## Project Structure

```
multi_uav_sim/
├── launch/
│   ├── launch_multi_uav.sh       # launch 3 UAV simulation
│   └── launch_single.sh          # launch single UAV simulation
├── models/                       # all Gazebo models (bundled)
│   ├── iris_1/                   # UAV 1 model (SITL ports 9002/9003)
│   ├── iris_2/                   # UAV 2 model (SITL ports 9012/9013)
│   ├── iris_3/                   # UAV 3 model (SITL ports 9022/9023)
│   ├── iris_with_standoffs/      # base iris mesh and physics
│   ├── iris_with_ardupilot/      # iris with ardupilot config
│   └── gimbal_small_2d/          # 2D gimbal with camera
├── scripts/
│   ├── takeoff_all.py            # arm and takeoff all 3 UAVs
│   └── single_drone_auto.py      # autonomous single drone flight
├── worlds/
│   ├── multi_uav.world           # 3 UAV world
│   └── single_uav.world          # single UAV world
├── setup.sh                      # configure environment variables
└── README.md
```

---

## How It Works

```
setup.sh
  └── sets ARDUPILOT_HOME, GAZEBO_MODEL_PATH, GAZEBO_RESOURCE_PATH

launch_multi_uav.sh
  ├── starts Gazebo (loads world + models from repo/models)
  │     └── libArduPilotPlugin.so (pre-installed, bridges Gazebo ↔ SITL)
  │           └── UDP 9002/9003, 9012/9013, 9022/9023
  ├── starts ArduCopter SITL x3 (flight controller simulation)
  │     └── TCP 5760, 5770, 5780
  └── MAVProxy connects via TCP → MAVLink → your Python scripts
```

---

## Port Reference

| UAV | Gazebo UDP in | Gazebo UDP out | MAVProxy TCP |
|-----|--------------|----------------|--------------|
| UAV 1 (sysid 1) | 9002 | 9003 | 5760 |
| UAV 2 (sysid 2) | 9012 | 9013 | 5770 |
| UAV 3 (sysid 3) | 9022 | 9023 | 5780 |

---

## Troubleshooting

**Ports already in use:**
```bash
pkill -f arducopter && pkill -f gzserver && pkill -f gzclient
sleep 5
```

**Gazebo models not found:**
Make sure you are running from a fresh terminal after editing `setup.sh`. Do not manually `source setup.sh` before running the launch script — the launch script sources it automatically.

**ArduPilot Gazebo plugin not found:**
```bash
ls /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/ | grep ArduPilot
```
If missing, repeat Step 3.

**MAVProxy permission denied:**
Always run MAVProxy from your home directory:
```bash
cd ~ && mavproxy.py --master=tcp:127.0.0.1:5760 --logfile=~/mav1.tlog
```

---

## Notes

- Requires **Gazebo Classic 11** — not compatible with Gazebo Harmonic/Garden/Fortress
- ArduPilot binary is built fresh on each launch — this takes ~2-3 seconds if already built
- All models are bundled in `models/` — no internet connection required to run the simulation
- `GAZEBO_MODEL_DATABASE_URI=""` is set in `setup.sh` to disable online model fetching
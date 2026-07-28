# Raspberry Pi 5 Setup — Edge Vision Node (Stage B)

Goal: get a **boxed Raspberry Pi 5** to the point where it runs UAV2's onboard vision
(`camera_relay` + `detector`) and swaps in for the `uav2ns` namespace in the HITL rig.
The autopilot (SITL), Gazebo, ns-3 and the GCS all stay on the host PC.

> **Match the host exactly** or ROS 2 won't talk across machines:
> **Ubuntu 22.04 (jammy) · 64-bit ARM (arm64) · ROS 2 Humble.**
> (Host confirmed: Ubuntu 22.04.5, ROS Humble.)

This guide is **phased**. Do Phase 3 fully (the Pi becomes a working ROS vision box on
its own), then Phase 4 (wire it into the sim the easy way), then Phase 5 (add the ns-3
impaired link for the real HITL measurement).

---

## What you need (hardware)

- Raspberry Pi 5 (4 GB or 8 GB).
- **Official 27 W USB-C PD power supply** (the Pi 5 is picky — underpowered supplies cause
  random reboots that look like software bugs).
- **microSD card** (32 GB+, decent brand) — or an NVMe HAT + SSD if you have one.
- A second computer (your host PC works) to **flash** the card.
- **USB-Gigabit-Ethernet adapter** for the Pi↔host link (used in Phase 4/5).
- For first boot, either: a monitor + USB keyboard, **or** go fully headless (we enable
  SSH during flashing — recommended, no monitor needed).

---

## Phase 3 — Turn the boxed Pi into a working ROS vision box

### 3.1 Flash Ubuntu 22.04 (on your host PC)

1. Install **Raspberry Pi Imager** on the host:
   ```bash
   sudo apt update && sudo apt install -y rpi-imager
   ```
   > **NOTE:** current Pi Imager only lists the newest LTS (24.04) for the Pi 5 — **22.04
   > will NOT appear** in the OS menu. So we flash the official 22.04 image with "Use
   > custom." Do NOT use 24.04 — ROS 2 Humble needs 22.04.
2. Download the official 22.04 arm64 Pi image on the host (no need to unzip — Imager reads
   `.img.xz` directly):
   ```bash
   cd ~/Downloads
   wget https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz
   ```
3. Insert the microSD card, run `rpi-imager` and choose:
   - **Device:** Raspberry Pi 5
   - **OS:** scroll to the bottom → **Use custom** → select the `ubuntu-22.04.5-...raspi.img.xz`
   - **Storage:** your microSD card → **Write**
4. **Headless WiFi + SSH — the Ubuntu way (cloud-init).** Pi Imager's "Edit Settings"
   dialog is for *Raspberry Pi OS* and usually does NOT apply to Ubuntu images, so configure
   WiFi via cloud-init instead:
   - After writing, **unplug/replug the microSD** so the FAT partition **`system-boot`**
     mounts.
   - Edit the file **`network-config`** on that partition:
     ```yaml
     version: 2
     wifis:
       wlan0:
         dhcp4: true
         optional: true
         access-points:
           "YOUR_WIFI_NAME":
             password: "YOUR_WIFI_PASSWORD"
     ```
   - SSH is **already enabled by default** on Ubuntu Server images — nothing to do.
   - (Optional: set the hostname by editing `/etc/hostname` after first boot, or via the
     `user-data` cloud-init file.)
5. Eject the card, put it in the Pi, power on. **Default login: `ubuntu` / `ubuntu`** — you'll
   be forced to change the password on first login.

> **Even simpler alternative:** if you can plug the Pi into your router with a spare
> **Ethernet cable** for the first boot, skip the `network-config` edit entirely — it gets an
> IP over DHCP and SSH works immediately. Configure WiFi later from the console if you want.

### 3.2 First boot + SSH in (from the host)

The Pi will join your WiFi. Find it and SSH in:
```bash
# find it by hostname (mDNS) — often just works:
ping uav2-pi.local
ssh anton@uav2-pi.local
# ...or find its IP from your router's client list and: ssh anton@<pi-ip>
```
Then update it:
```bash
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

### 3.3 Install ROS 2 Humble (on the Pi)

```bash
# locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS 2 apt repo
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install (ros-base is enough for a headless vision node; add colcon + rosdep)
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions

# make it available every shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Sanity check:
```bash
ros2 --help >/dev/null && echo "ROS 2 Humble OK"
```

### 3.4 Python env for YOLO (on the Pi)

`ultralytics` (YOLO) pulls in PyTorch; on ARM64 that's a big but available wheel.
Keep it in a venv so it doesn't fight ROS's system packages.
```bash
sudo apt install -y python3-venv python3-pip libgl1 libglib2.0-0
python3 -m venv ~/yolo_env --system-site-packages   # see ROS/cv2 from system
source ~/yolo_env/bin/activate
pip install --upgrade pip
pip install ultralytics                              # brings torch (CPU) for arm64
# grab the small model weights (same one the host uses)
python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"   # downloads yolov8n.pt to CWD
```
> `--system-site-packages` lets the venv's python also import ROS's `rclpy`, `cv2`, `numpy`
> from `/opt/ros`, so the detector node can run with venv python (same trick as the host).

### 3.5 Get the code onto the Pi + build

```bash
# clone your repo (use whatever remote/branch you use; this is the HITL branch)
git clone <your-repo-url> ~/multi_uav_simulation
cd ~/multi_uav_simulation
git checkout ground-vs-edge-processing-RPi

# build ONLY the vision package (the Pi doesn't need gazebo/ns-3 packages)
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_vision
echo "source ~/multi_uav_simulation/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.6 Sanity test — YOLO works on ARM

Confirm the detector logic runs on the Pi before any networking:
```bash
source ~/yolo_env/bin/activate
python - <<'PY'
from ultralytics import YOLO
m = YOLO('yolov8n.pt')
r = m('https://ultralytics.com/images/bus.jpg')   # or any local jpg with people
print("persons:", sum(int(c)==0 for c in r[0].boxes.cls))
PY
```
Non-zero persons = YOLO works on the Pi. **Phase 3 done.** ✅
(Expect it to be slower than the host x86 — a few fps on CPU. That slowness is exactly what
the edge-vs-ground comparison is about.)

---

## Phase 4 — Wire the Pi in the EASY way (no ns-3 yet)

Prove the **compute path** first with one plain wired link and flat DDS — no impairment.

1. Cable the Pi to the host via the **USB-Gigabit-Ethernet adapter**. Give both ends static
   IPs on a private subnet, e.g. host `192.168.50.1/24`, Pi `192.168.50.2/24`. Confirm
   `ping` both ways.
2. Same ROS domain on both: `export ROS_DOMAIN_ID=0` (host and Pi).
3. On the **host**, run Gazebo + SITL as usual (`make gazebo`, `make sitl`) so
   `/uav1/camera/image_raw` (or a uav2 camera) is publishing, and run `gcs_receiver` on the
   host.
4. On the **Pi**, run the edge nodes against the host's camera topic:
   ```bash
   source ~/yolo_env/bin/activate
   ros2 run uav_vision camera_relay --ros-args -p uav_id:=2 -p processing_mode:=edge -p frame_rate_hz:=2.0
   # separate shell:
   python ~/multi_uav_simulation/install/uav_vision/lib/uav_vision/detector \
       --ros-args -p uav_id:=2 -p processing_mode:=edge -p model_path:=$HOME/yolo_env/yolov8n.pt
   ```
**Success:** the Pi detects people in the SITL feed and detections appear on the host's
`gcs_receiver`. No `fastdds_udp_only.xml` needed — separate machines already use UDP.

> Discovery over a single wired link is usually fine with default multicast. If nodes don't
> find each other, set unicast peers (FastDDS `ROS_STATIC_PEERS` / a discovery XML) — see
> the "hard problems" note in `docs/HITL_INTEGRATION_PLAN.md`.

---

## Phase 5 — Insert the ns-3 impaired link (full HITL)

Replace the `uav2ns` veth in bridge `br-uav2` with the **physical USB-Ethernet NIC** facing
the Pi, so Pi→GCS detections ride the simulated Wi-Fi:

- Pi wireless IP `10.42.0.12` (routed through host `br-uav2 → tap-uav2 → ns-3`).
- Pi sensor IP `172.31.2.2` (direct, unimpaired — carries the camera).
- The GCS lives only on `10.42.0.x`, so DDS routes detections through ns-3 automatically —
  same trick the namespaces use.
- **Clock sync:** run `chrony` (or PTP) between Pi and host, or the `metrics_logger`
  latency numbers (receipt_wall − send_wall across two machines) are meaningless.

See `docs/HITL_INTEGRATION_PLAN.md` §Phase 5–6 for the bridge-rewire details and the
edge-vs-ground data collection.

---

## Quick reference — where things run

| Component | Host PC | Raspberry Pi 5 (UAV2) |
|---|---|---|
| Gazebo + camera | ✅ | |
| ArduPilot SITL (autopilot) | ✅ | |
| ns-3 wireless channel | ✅ | |
| `gcs_receiver` + `metrics_logger` | ✅ | |
| `camera_relay` + `detector` (edge YOLO) | | ✅ |

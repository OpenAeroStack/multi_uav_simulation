# Raspberry Pi 4B Setup — Edge Vision Node (Stage B)

Goal: get a **Raspberry Pi 4B** to the point where it runs UAV2's onboard vision
(`camera_relay` + `detector`) and swaps in for the `uav2ns` namespace in the HITL rig.
The autopilot (SITL), Gazebo, ns-3 and the GCS all stay on the host PC.

> **Board matters — this guide is for the Raspberry Pi 4B.** The Pi 4B fully supports
> **Ubuntu 22.04**, so it runs **exactly the same stack as the host**:
> **Ubuntu 22.04 (jammy) · 64-bit ARM (arm64) · ROS 2 Humble.** No version mismatch, no
> workarounds — an exact match with the host means DDS "just works."
>
> (Note: the **Pi 5** is different — it needs Ubuntu 24.04 and won't boot 22.04. If you ever
> switch to a Pi 5, you'd use 24.04 + ROS 2 Jazzy, or run Humble in Docker. This guide assumes
> the **Pi 4B**.)

This guide is **phased**. Do Phase 3 fully (the Pi becomes a working ROS vision box on
its own), then Phase 4 (wire it into the sim the easy way), then Phase 5 (add the ns-3
impaired link for the real HITL measurement).

---

## What you need (hardware)

- **Raspberry Pi 4B** (4 GB or 8 GB) — *not* a Pi 5; see the note above.
- **Official 15 W USB-C power supply** (underpowered supplies cause random reboots that
  look like software bugs).
- **microSD card** (32 GB+, decent brand).
- A second computer (your host PC works) to **flash** the card.
- **USB-Gigabit-Ethernet adapter for the HOST.** The Pi 4B has a built-in gigabit port, but
  the host laptop has no RJ45. Buy a **USB 3.0** adapter with an RTL8153 or AX88179 chipset —
  a cheap QTS1081B managed only 4.5 Mbps with 24% loss here, and `ethtool` misreported it as
  100 Mbps full-duplex. **Trust `iperf3`, not `ethtool`:** link speed is what the NIC claims,
  throughput is what it delivers. A working adapter gives ~834 Mbps steady.
- **Cat6 cable**, host ↔ Pi direct. One cable is enough — Phase 5 splits it into two logical
  links with VLANs.
- For first boot, either: a monitor + USB keyboard, **or** go fully headless (we enable
  SSH during flashing — recommended, no monitor needed).

---

## Phase 3 — Turn the boxed Pi into a working ROS vision box

### 3.1 Flash Ubuntu 22.04 (on your host PC)

1. Install **Raspberry Pi Imager** on the host:
   ```bash
   sudo apt update && sudo apt install -y rpi-imager
   ```
2. Insert the microSD card, run `rpi-imager` and choose:
   - **Device: Raspberry Pi 4** ← IMPORTANT. If you pick "Pi 5" here, Imager HIDES 22.04
     (22.04 is not Pi-5-compatible). Selecting Pi 4 makes 22.04 appear.
   - **OS:** *Other general-purpose OS → Ubuntu → **Ubuntu Server 22.04.x LTS (64-bit)***
     (Server, not Desktop — lighter for a headless companion computer.)
   - **Storage:** your microSD card
3. When prompted, open **OS Customisation → General** and set:
   - **Set hostname:** `uav2-pi`
   - **Set username and password:** `anton` / *(a password you'll remember — this is your SSH login)*
   - **Configure wireless LAN:** SSID + password; **Wireless LAN country = your actual country**
     (e.g. `LK`) — a wrong country blocks the WiFi channels. Prefer the **2.4 GHz** band for
     headless first boot (more reliable than 5 GHz).
4. **Services tab → Enable SSH** → Use password authentication.
5. **Save → Write.** Eject → into the Pi → power on → wait 2–3 min for first boot.

> **Bulletproof backup if WiFi won't connect:** the Pi 4B has a built-in Ethernet port. Plug a
> cable from it to your host's USB-Ethernet adapter (or a router LAN port) and reach it over the
> wire — no WiFi needed. This is also the link used later in Phase 4/5.
>
> **cloud-init only applies on FIRST boot** — if you change WiFi/SSH after the Pi has already
> booted once, editing `network-config` on the card does nothing; **re-flash** to re-apply.

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

### 3.3 Install ROS 2 Humble (on the Pi — matches the host exactly)

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
# IMPORTANT (Pi 4): install CPU-only torch FIRST. The default arm64 torch wheel is a
# CUDA build whose CPU kernels use instructions the Pi 4 (Cortex-A72) lacks -> YOLO
# inference crashes with "Illegal instruction" (SIGILL). The +cpu wheels fix it.
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install ultralytics
# PIN these two — see the warning below. ultralytics pulls newer versions that
# break cv_bridge, so install them AFTER ultralytics to force the downgrade.
pip install "numpy==1.26.4" "opencv-python==4.10.0.84"
# grab the small model weights (same one the host uses)
python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"   # downloads yolov8n.pt to CWD
```
> `--system-site-packages` lets the venv's python also import ROS's `rclpy`, `cv2`, `numpy`
> from `/opt/ros`, so the detector node can run with venv python (same trick as the host).

> ### ⚠ Never run `pip install -U` in this venv
>
> `cv_bridge` is a **compiled C++ extension** built against specific NumPy and OpenCV
> versions. Two separate upgrades break it, and neither failure is ARM-specific:
>
> | Upgrade | Error | Why |
> |---|---|---|
> | NumPy ≥ 2.0 | `AttributeError: _ARRAY_API not found` | built against the NumPy 1.x C ABI |
> | OpenCV 5.0 | `KeyError: 16` in `cv2_to_imgmsg` | OpenCV 5 changed the `CV_8UC3` constants cv_bridge reads to build its type table (16 *is* `CV_8UC3`) |
>
> The two are linked: **`opencv-python >= 4.11` requires `numpy >= 2`**, so only the older
> pair can coexist. Installing `opencv-python<5` on its own silently pulls NumPy 2 back in —
> pin both together, as above.
>
> This pair is exactly what the host has, which is why detector code developed there
> "just worked" until it was moved to the Pi.
>
> Verify with:
> ```bash
> source /opt/ros/humble/setup.bash
> ~/yolo_env/bin/python -c "
> import numpy, cv2; print(numpy.__version__, cv2.__version__)
> from cv_bridge import CvBridge; import numpy as np
> b = CvBridge()
> print(b.cv2_to_imgmsg(np.zeros((4,4,3), np.uint8), 'bgr8').encoding, 'OK')"
> ```
> Expect `1.26.4 4.10.0 bgr8 OK`. Test **both** directions — reading worked fine while
> writing was broken, so an import check alone is not enough.

### 3.4b Optional — NCNN for ~2× faster inference

NCNN is ARM-NEON optimised and is what ultralytics benchmarks as fastest on Raspberry Pi.
Measured on this Pi 4B: **2,540 ms → ~1,000 ms per frame** at the same input size.

```bash
pip install ncnn
```

Export on the **host** (faster, and the `pnnx` converter ships an x86 wheel) and copy over —
NCNN's `.param`/`.bin` files are architecture-independent:

```bash
# on the HOST:
venv/bin/python -c "
from ultralytics import YOLO
YOLO('yolov8n.pt').export(format='ncnn', imgsz=[544,960], half=False)"
scp -r yolov8n_ncnn_model/ anton@10.0.0.2:/home/anton/models/
```

> **`imgsz` must match what the detector runs**, and the order is `[height, width]`.
> NCNN bakes in a **fixed input shape**; a plain `imgsz=960` produces a *square* 960×960
> input (19.9 GFLOPS, 43% grey padding) instead of the rectangular 960×544 that a 1280×720
> frame actually letterboxes to (11.3 GFLOPS). A mismatch is silently ignored, not raised.
>
> **`half=False`** — the Cortex-A72 has no fp16 SIMD, the same gap that causes the
> CUDA-torch SIGILL above.

Then just point the detector at the directory instead of the `.pt` file; no code change is
needed, since `YOLO()` accepts an NCNN model directory.

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

## Phase 4 — Camera over the wired link ✅ DONE (2026-08-05)

Static addresses on both ends of the cable. **Make them persistent** — every hand-added
address on this project eventually vanished, because a network manager reconciles the kernel
against its own config and deletes anything it did not create.

The two machines need **different tools**:

```bash
# HOST (Ubuntu Desktop → NetworkManager). Name comes from the MAC, so a
# different adapter means a different profile.
nmcli connection show                       # find the wired profile name
sudo nmcli connection modify "Wired connection 2" ipv4.method manual \
     ipv4.addresses 10.0.0.1/24 ipv4.never-default yes ipv6.method disabled
sudo nmcli connection up "Wired connection 2"
```

```bash
# PI (Ubuntu Server → systemd-networkd / netplan)
sudo tee /etc/netplan/99-cat6-link.yaml >/dev/null <<'EOF'
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: no
      dhcp6: no
      addresses: [10.0.0.2/24]
EOF
sudo chmod 600 /etc/netplan/99-cat6-link.yaml
sudo netplan try          # auto-reverts in 120 s if it breaks your SSH
```

`ipv4.never-default yes` matters — without it NM may route all traffic down the cable and
break WiFi. No gateway or DNS on either side: this is a private point-to-point link.

**Kernel socket buffers — mandatory on both machines.** A 1280×720 RGB frame is 2.76 MB,
about 1,900 UDP fragments. The default 208 KB receive buffer holds ~7% of one frame, so
reassembly never completes and the subscriber **silently receives nothing** — the topic
appears in `ros2 topic list` but no data ever arrives.

```bash
sudo tee /etc/sysctl.d/60-ros2-dds.conf >/dev/null <<'EOF'
net.core.rmem_max = 536870912
net.core.rmem_default = 134217728
net.ipv4.ipfrag_high_thresh = 134217728
EOF
sudo sysctl --system
```

**Pin DDS to the cable.** Both machines are on WiFi *and* Ethernet, and Fast DDS otherwise
announces a locator on every interface — the camera can end up on WiFi, where large samples
fragment and are dropped. Use `config/fastdds_hitl_eth.xml` on both:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/uav2_ws/config/fastdds_hitl_eth.xml
```

> **Fast DDS 2.6 reads the interface list once**, at participant creation, with no dynamic
> detection. An address assigned *after* a process starts can never be used by it — so
> bring the addresses up **before** launching Gazebo, and restart anything that started
> earlier.

**Verified result:** 19.8 Hz at the Pi, 481 Mbps, 42,544 pkt/s, **0 dropped packets**.

---

## Phase 5 — Detections across the ns-3 impaired link ✅ DONE (2026-08-05)

The Pi has one Ethernet port but needs two links, so `eth0` is split with 802.1Q VLANs:

| VLAN | Pi interface | Address | Carries | Impaired? |
|---|---|---|---|---|
| 10 | `eth0.10` | `10.0.0.2` | camera **in** | ❌ plain cable |
| 42 | `eth0.42` | `10.42.0.12` | detections **out** | ✅ via ns-3 |

The Pi occupies the **UAV2 slot** of the 4-node ns-3 binary (node 0 = GCS, node 1 = SITL,
node 2 = the Pi). On the host, VLAN 42 is bridged into `br-uav2 → tap-uav2 → ns-3`.

Run the host launcher **first** (it creates `br-uav2` and the host-side VLANs), then on the
Pi:

```bash
sudo bash ~/pi_hitl_link.sh
```

Verify **both** paths — this is the test that proves the split is real:

```bash
ping -c 3  10.0.0.1        # camera link  → ~2 ms
ping -c 30 10.42.0.10      # radio link   → ~59 ms, high jitter
```

Measured: camera 2.2 ms / 0.8 ms jitter; radio min 3.3, avg 59.0, max 158.0 ms, 40.8 ms
jitter, 0% loss. Use **≥30 pings** — the first packet includes ARP across the simulated
channel and a 2-ping sample reported a misleading 167 ms average.

> If the radio link answers as fast as the cable, detections are **not** crossing ns-3 and
> every latency number will be meaningless.

> **⚠ Not persistent.** `pi_hitl_link.sh` uses plain `ip` commands, so a Pi reboot drops
> back to untagged `eth0` while the host still expects VLAN 10. The symptom is
> `Destination Host Unreachable` with the carrier up and the link at 1000 Mbps. Re-run the
> script after every reboot until it is moved into netplan with a `vlans:` section.

**Clock sync is still outstanding** — run `chrony` between the Pi and host before
collecting any latency data, or `metrics_logger`'s `receipt_wall − send_wall` across two
unsynchronised clocks produces plausible-looking nonsense.

---

## Running the detector on the Pi

```bash
ssh anton@10.0.0.2
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/uav2_ws/config/fastdds_hitl_eth.xml

~/yolo_env/bin/python ~/yolo_detect_node.py --ros-args \
    -p model_path:=/home/anton/models/yolov8n.pt \
    -p show_window:=False
```

`show_window:=False` is required — the Pi is headless and `cv2.imshow` has no display.
Swap `model_path` to `/home/anton/models/yolov8n_ncnn_model` for the NCNN build.

> **Sensor subscriptions must use `BEST_EFFORT`, depth 1.** Gazebo's camera publisher is
> `RELIABLE`; a `RELIABLE` subscriber makes it wait for an acknowledgement of every frame,
> and the Pi (≈1 fps) dragged the **whole simulation's camera down to 0.27 Hz**. Worse, when
> the Pi rebooted mid-run its unacknowledged samples left the publisher stalled at its
> retransmission heartbeat. A frame that arrives late is worthless anyway.

> **The Pi has its own copy of the code.** `~/uav2_ws/src/uav_vision` does not track the
> host repo. After any edit on the host:
> ```bash
> rsync -av ros2/uav_vision/ anton@10.0.0.2:~/uav2_ws/src/uav_vision/
> ssh anton@10.0.0.2 'cd ~/uav2_ws && source /opt/ros/humble/setup.bash && \
>     colcon build --packages-select uav_vision'
> ```
> Stale code once left the detector subscribed to a topic nobody published — it started
> cleanly and then sat silent.

---

## Measured performance (Pi 4B)

| Engine | imgsz | Per frame | Rate |
|---|---|---|---|
| PyTorch | 960×544 | **2,540 ms** | 0.39 fps |
| PyTorch | 640×384 | ~1,100 ms | 0.9 fps |
| **NCNN** | 960×544 | **~1,000 ms** | 0.74 fps |
| NCNN | 640×384 | ~470 ms *(predicted)* | — |

Resolution and engine effects are separable and roughly multiplicative; inference scales
close to linearly with pixel count.

The same model runs in **623 ms on a static image** but ~1,343 ms in the live pipeline — the
Pi loses roughly half its inference capacity to deserialising camera frames it discards.
Throttling the camera at source (`update_rate` 20 → 5 in the model SDF) recovered a quarter
of that. `camera_relay`'s `frame_rate_hz` cannot: it throttles *after* receipt, so the bytes
have already crossed the link.

---

## Quick reference — where things run

| Component | Host PC | Raspberry Pi 4B (UAV2) |
|---|---|---|
| Gazebo + camera | ✅ | |
| ArduPilot SITL (autopilot) | ✅ | |
| ns-3 wireless channel | ✅ | |
| `gcs_receiver` (inside `gcsns`) | ✅ | |
| `yolo_detect_node.py` (edge YOLO) | | ✅ |
| Address on the camera VLAN | `10.0.0.1` (`eth-cam`) | `10.0.0.2` (`eth0.10`) |
| Address on the radio VLAN | `10.42.0.10` (in `gcsns`) | `10.42.0.12` (`eth0.42`) |

See `docs/HITL_RUNBOOK.pdf` for the full command sequence and a troubleshooting table.

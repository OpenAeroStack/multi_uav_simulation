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

### 3.4b Faster inference — OpenVINO or NCNN

PyTorch is the slowest option on this board by 4x. Measured on this Pi 4B with
`scripts/bench_backends.py` (10 calls each, median, correctness checked before speed):

| Back-end | Median | vs PyTorch |
|---|---|---|
| **YOLO11n OpenVINO** | **236 ms** | **4.35x** |
| YOLOv8n NCNN 384x640 | 270 ms | 3.80x |
| YOLO11n MNN | 342 ms | 3.00x — *slower than NCNN here* |
| YOLOv8n PyTorch 640 | 1,027 ms | 1.00x |

OpenVINO on ARM is not emulation: the aarch64 wheel ships
`libopenvino_arm_cpu_plugin.so`, built on **Arm Compute Library**, so it uses NEON exactly
as NCNN does. `FULL_DEVICE_NAME` reports `Raspberry Pi 4 Model B`.

**Published Pi 5 rankings do not transfer.** The Pi 5's Cortex-A76 is ARMv8.2-A; the Pi 4B's
Cortex-A72 is ARMv8.0-A and reports `i8sdot:0, fp16:0, i8mm:0`. That is why MNN inverts and
why OpenVINO wins by only 1.15x here instead of 3.6x. It is also why **`half=True` gains
nothing** — the runtime advertises `['FP32', 'INT8', 'BIN']`, no FP16.

```bash
pip install openvino ncnn
```

Export on the **host** and copy over — both formats are architecture-independent:

```bash
# on the HOST:
venv/bin/python -c "
from ultralytics import YOLO
YOLO('yolo11n.pt').export(format='openvino', imgsz=[384,640])"
rsync -a yolo11n_openvino_model/ anton@10.0.0.2:~/models/yolo11n_openvino_model/

venv/bin/python -c "
from ultralytics import YOLO
YOLO('yolov8n.pt').export(format='ncnn', imgsz=[384,640], half=False)"
rsync -a yolov8n_ncnn_model/ anton@10.0.0.2:~/models/yolov8n_384x640_ncnn_model/
```

> ### ⚠ Never pass `imgsz` to an exported model
>
> An earlier revision of this guide said *"a mismatch is silently ignored, not raised."*
> **That is wrong, and it is the dangerous kind of wrong.** For an export the input shape is
> fixed at export time, and `imgsz` also overrides the shape ultralytics uses to *decode*
> the output coordinates. Reproduced on a 384x640 export:
>
> | | call 1 | calls 2-6 |
> |---|---|---|
> | NCNN, `imgsz=640` passed | 3 persons, conf 0.85 | **2 persons, conf 0.74** — no error |
> | NCNN, `imgsz` dropped | 3 persons, 0.85 | 3 persons, 0.85 |
> | OpenVINO, `imgsz=640` passed | `RuntimeError: shape mismatch` | — |
>
> OpenVINO raises. **NCNN does not** — it settles into a stable, plausible, wrong answer.
> Both detectors now drop `imgsz` for anything that is not a `.pt`. Note `imgsz` is
> `[height, width]`, the opposite of the usual W x H.
>
> `half=False` for NCNN — the Cortex-A72 has no fp16 SIMD, the same gap behind the
> CUDA-torch SIGILL above.

Point the detector at the model directory instead of the `.pt`; no code change is needed.

### 3.5 Get the code onto the Pi + build

The working Pi uses **`~/uav2_ws`**, with only `uav_vision` in it — not a full clone. Match
that, because `run_hitl.sh` hardcodes the path:

```bash
mkdir -p ~/uav2_ws/src ~/uav2_ws/config ~/models
```
```bash
# FROM THE HOST — the Pi does NOT track the repo; it is rsynced
rsync -av --exclude='__pycache__' ros2/uav_vision/ anton@<pi>:~/uav2_ws/src/uav_vision/
scp config/fastdds_hitl_eth.xml anton@<pi>:~/uav2_ws/config/
scp scripts/yolo_detect_node.py scripts/bench_backends.py anton@<pi>:~/
```
```bash
# ON THE PI
cd ~/uav2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select uav_vision
echo "source ~/uav2_ws/install/setup.bash" >> ~/.bashrc
```

Also set up key auth and passwordless sudo now — Ansible will need both later:

```bash
ssh-copy-id anton@<pi>                                        # from the host
echo "anton ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/90-anton-nopasswd
sudo chmod 440 /etc/sudoers.d/90-anton-nopasswd               # on the Pi
```

> **After any host-side edit, re-sync and rebuild.** Stale code once left the detector
> subscribed to a topic nobody published — it started cleanly and then sat silent.

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

> ### ⚠ Do NOT create a plain `eth0` address here
>
> Earlier revisions told you to write `99-cat6-link.yaml` giving `10.0.0.2` to untagged
> `eth0`, and then to add VLANs in Phase 5. **Doing both puts one address on two
> interfaces.** Netplan merges files in number order without warning, so both take effect,
> and three unrelated-looking symptoms follow: ping and SSH still work; multicast silently
> never arrives; and **Fast DDS segfaults at participant creation** (exit 139, no message)
> because the whitelist entry matches two devices and produces duplicate locators.
>
> Skip straight to the VLAN file in Phase 5 — it carries the address. If a machine already
> has `99-cat6-link.yaml`, delete it.

Check before every session; it must print `1`:

```bash
ip -4 addr show | grep -c "10.0.0.2/24"     # on the Pi
grep -l "eth0" /etc/netplan/*.yaml          # only ONE file may touch eth0
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
net.core.wmem_max = 536870912
net.core.wmem_default = 134217728
net.ipv4.ipfrag_high_thresh = 134217728
net.ipv4.conf.all.arp_ignore = 1
net.ipv4.conf.all.arp_announce = 2
EOF
sudo sysctl --system
```

> **Both directions, or neither works.** Earlier revisions of this guide set only `rmem`.
> With `wmem_max` left at the default, the DDS profile's 16 MB `<sendBufferSize>` is
> silently clamped to 208 KB and the **publisher** discards fragments before they reach the
> NIC. That failure is invisible to every interface counter — `tx_dropped` stays 0 on the
> host and `rx_errors` stays 0 on the Pi, because nothing was ever transmitted. Measured
> symptom: 0.19 Hz delivered against a 5 Hz source. It cost days.
>
> The two `arp_*` lines stop the Pi answering ARP for its cable address over WiFi, which
> would otherwise let traffic you believe is on the cable take another path.

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

Make it persistent with netplan — **this file carries the addresses**, so there must be no
other netplan file touching `eth0`:

```bash
sudo tee /etc/netplan/60-hitl-vlans.yaml >/dev/null <<'EOF'
network:
  version: 2
  ethernets:
    eth0: {dhcp4: no, optional: true}
  vlans:
    eth0.10: {id: 10, link: eth0, addresses: [10.0.0.2/24]}
    eth0.42: {id: 42, link: eth0, addresses: [10.42.0.12/24]}
EOF
sudo chmod 600 /etc/netplan/60-hitl-vlans.yaml
sudo netplan try          # auto-reverts in 120 s if it breaks your SSH
```

`optional: true` stops boot waiting two minutes for a DHCP reply that will never come.
Run the host launcher first — it creates `br-uav2` and the host-side VLANs.

> `scripts/netns/pi_hitl_link.sh` did this with plain `ip` commands and did **not** survive
> a reboot. It is superseded by the netplan file above and kept only for a machine that has
> not been configured yet.

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

### Clock sync ✅ SOLVED

The Pi takes time **from the host over the camera link**, not from the internet — that link
is 1.4 ms away and always present, so it beats public NTP on both accuracy and availability,
and the rig needs no internet at all.

```bash
# HOST — chrony replaces systemd-timesyncd, which is client-only and cannot serve
sudo apt install -y chrony
sudo tee -a /etc/chrony/chrony.conf > /dev/null <<'EOF'

allow 10.0.0.0/24
local stratum 10
EOF
sudo systemctl restart chrony

# PI
sudo apt install -y chrony
sudo sed -i '1i server 10.0.0.1 iburst prefer minpoll 4 maxpoll 6' /etc/chrony/chrony.conf
sudo systemctl restart chrony
```

Verify on the Pi — `chronyc sources` should show `^*` on `10.0.0.1`, and `chronyc tracking`
a `System time` offset of a few microseconds (measured: 2-60 us against a 75 ms radio
latency, an error of well under 0.1%). A Pi 4B has **no battery-backed clock**, so without
this it starts each boot from whatever `fake-hwclock` last wrote — one was found 2 h 55 m
behind, which would have made every latency figure nonsense while looking plausible.

---

## Running the detector on the Pi

```bash
ssh anton@10.0.0.2
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/uav2_ws/config/fastdds_hitl_eth.xml

~/yolo_env/bin/python ~/yolo_detect_node.py --ros-args \
    -p model_path:=$HOME/models/yolo11n_openvino_model \
    -p show_window:=False
```

`show_window:=False` is required — the Pi is headless and `cv2.imshow` has no display.
Swap `model_path` for `yolov8n_384x640_ncnn_model` (270 ms) or `yolov8n.pt` (1,027 ms).
**Do not pass `imgsz` with either exported model** — see §3.4b.

In normal use you do not run this by hand: `./scripts/netns/run_hitl.sh` starts the whole
stack including the Pi's detector over SSH, and tears it down with Ctrl+C. Expect ~12 s to
load OpenVINO and a ~13 s first inference (it compiles the model on the first call), then
~250 ms steady.

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

| Engine | imgsz | Per frame | In pipeline |
|---|---|---|---|
| **OpenVINO** | 384×640 | **236 ms** | ~250 ms |
| NCNN | 384×640 | 270 ms | 277 ms (3.47 fps) |
| PyTorch | 640 | 1,027 ms | 1,013 ms (0.98 fps) |

Delivered configuration: camera **640×384 at 5 Hz**, FOV 0.6 rad, 45° pitch, `conf=0.4`,
class 0 only. Inference went from 1,343 ms to 236 ms — 5.7x — entirely through
configuration: removing the annotated return stream, matching the camera to the model
input, and PyTorch → NCNN → OpenVINO.

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

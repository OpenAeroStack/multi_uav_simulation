# HITL Integration Plan — Raspberry Pi 4B edge node ↔ SITL host

**Author:** anton · **Branch:** `ground-vs-edge-processing-RPi` (sub-branch of
`ground-vs-edge-processing`) · **Last reconstructed:** 2026-07-31

> **Board correction (2026-07-28):** the edge node is a **Raspberry Pi 4B**, not a Pi 5.
> Earlier revisions of this document said "Pi 5" throughout and that was wrong. The
> distinction matters a great deal: the Pi 4B officially supports **Ubuntu 22.04**, so it
> runs the *exact same stack as the host* — Ubuntu 22.04 + ROS 2 Humble, natively, with an
> identical DDS implementation. A Pi 5 would have forced Ubuntu 24.04 + ROS 2 Jazzy (or
> Humble inside Docker) and a cross-distro DDS pairing. Always confirm the physical board
> before writing setup steps.

---

## 1. Goal

Move UAV vision compute off the host and onto **real silicon** — a Raspberry Pi 4B acting
as the drone's companion computer — while the autopilot (ArduPilot SITL), Gazebo physics,
the ns-3 wireless channel and the ground station all remain on the host PC.

This converts the pure-software *edge vs ground processing* experiment into a
**hardware-in-the-loop (HITL)** one, where the "edge" arm of the comparison is a genuine
resource-constrained ARM CPU rather than a thread on a fast x86 laptop. The headline
result the experiment is built to produce:

| Mode | What crosses the impaired radio link | Bottleneck |
|---|---|---|
| **Ground processing** | Raw camera frames (~72 Mbps) | The radio link |
| **Edge processing** | Detection messages (**76 bytes**) | The Pi's CPU (~1–3 s/frame) |

That tradeoff is only meaningful if the edge CPU is real. Hence HITL.

## 2. Locked decisions

- **Raspberry Pi 4B** as the edge compute node — Ubuntu 22.04 + ROS 2 Humble, **native**
  (no Docker, no Jazzy).
- **Wired USB-Gigabit-Ethernet** for the Pi↔host sensor link. The host has no RJ45 port.
- **Vision only, first.** The Pi runs `camera_relay` + `detector`. It does **not** command
  the drone. All autopilot logic stays in SITL on the host. Closing the control loop
  through the Pi is explicitly out of scope for now.
- **Two physically separate links**, mirroring the pure-sim namespace design (§3).
- Work stays on `ground-vs-edge-processing-RPi`. Do not commit HITL changes to the shared
  `ground-vs-edge-processing` branch.

## 3. Architecture — the two-link design

This is the single most important idea in the whole setup, and it is inherited directly
from the pure-simulation namespace topology.

**A drone has two very different data paths, and they must not be conflated:**

**As built (2026-08-05).** The Pi has a single Ethernet port, so the two links are carried
over one cable and separated with 802.1Q VLANs:

```
   HOST                                              PI 4B  (the UAV2 slot)
   ────                                              ─────────────────────
   Gazebo ─── eth-cam  VLAN 10 ═══ cable ═══ eth0.10 ──► detector (YOLOv8n)
              10.0.0.1                       10.0.0.2         │
              ① SENSOR LINK — unimpaired                      │  ~118 B
                                                              │  detections
   gcsns ◄── tap-gcs ◄── ns-3 ◄── tap-uav2 ◄── br-uav2 ◄──────┘
   10.42.0.10             ▲       eth-rf VLAN 42      eth0.42
   gcs_receiver     loss · latency · fading           10.42.0.12
                    ② WIRELESS LINK — IMPAIRED

   (separately, on the host only:)
   SITL ◄── FDM 9002/9003 over veth 172.31.1.1/.2 ──► Gazebo   never impaired
```

**① The sensor link is deliberately unimpaired.** It carries the Gazebo camera feed to the
Pi. In reality this is a ribbon cable between a camera module and the companion computer
*inside one airframe* — degrading it would be physically meaningless and would corrupt the
experiment. It never enters a bridge that ns-3 can see.

**② The wireless link is the real radio**, simulated by ns-3. Only detections cross it.
**This is what the experiment measures.**

**Routing enforces the split with no firewall rules.** Gazebo lives on `10.0.0.1` and can
only reach the Pi at `10.0.0.2`; `gcsns` lives on `10.42.0.10` and can only reach it at
`10.42.0.12`. Neither can take the other's path even by mistake.

Why VLANs rather than two cables: putting the physical NIC into `br-uav2` would work, but a
bridge floods every frame to all ports — the camera's ~111 Mbps would pour into `tap-uav2`
and through ns-3 as well, destroying the whole point. Two physical adapters per machine
would also work; VLANs achieve the same separation with the hardware already present.

The same principle governs the **FDM link** (`172.31.1.1 ↔ 172.31.1.2` over a veth pair):
the 1 kHz physics conversation between SITL and Gazebo bypasses ns-3 entirely, because it
represents a flight controller's own IMU wiring, not a radio.

**Resolved: the Pi takes the UAV2 slot.** The 4-node ns-3 binary has node 0 = GCS,
node 1 = SITL in `uav1ns`, node 2 = the Pi (`10.42.0.12`). `tap-uav2` was previously a bare
unused stub and is now bridged to the Pi's VLAN 42 leg, matching the original plan's intent
that the Pi replaces `uav2ns`. `uav1ns` keeps `10.42.0.11` for SITL, unchanged.

**Verified by measurement**, from the Pi (30 pings each):

| Link | Path | Latency |
|---|---|---|
| `10.0.0.1` | VLAN 10, direct cable | avg **2.2 ms**, jitter 0.8 ms |
| `10.42.0.10` | VLAN 42 → ns-3 → gcsns | min 3.3 / avg **59.0** / max 158.0 ms, jitter 40.8 ms, 0% loss |

~27× the latency and ~50× the jitter. The 3.3 ms minimum shows ns-3 is not applying a fixed
delay: it is a contended CSMA/CA channel where packets queue and back off. Use ≥30 pings —
the first packet includes ARP across the simulated channel and skews short samples badly
(a 2-ping sample reported a misleading 167 ms average).

## 4. Phase status

| Phase | Description | Status |
|---|---|---|
| 1 | ns-3 3.38 on the host | ✅ DONE 2026-07-26 |
| 2a | Flat (no netns) vision pipeline baseline | ✅ DONE 2026-07-26 |
| 2b | netns + ns-3 host rehearsal | ✅ DONE 2026-07-28 |
| 3 | Prepare the Pi 4B (OS, ROS, YOLO) | ✅ DONE 2026-07-29 |
| — | Host netns SITL reboot loop | ✅ RESOLVED 2026-07-31 |
| — | Pi↔host link bandwidth | ✅ RESOLVED 2026-07-30 (935 Mbps) |
| **4** | **Camera over the wired sensor link, detector on the Pi** | ✅ **DONE 2026-08-05** |
| **5** | **Detections across the ns-3 impaired link** | ✅ **networking DONE 2026-08-05** |
| 6 | Run the edge-vs-ground HITL comparison | ⬜ **NEXT** — blocked on clock sync |

---

## 5. Phase detail

### Phase 1 — ns-3 3.38 on the host ✅ DONE (2026-07-26)

- Installed at `~/ns-allinone-3.38/ns-3.38`; scenario staged in `scratch/multi_uav_simulation`.
- `./ns3 configure` with ROS 2 sourced → Tap Bridge ON, rclcpp found.
- Binary: `build/scratch/multi_uav_simulation/ns3.38-three_uav_tapbridge_integrated-default`,
  linked against `librclcpp.so` + `std_msgs`.
- **Remember:** the scratch copy is a **copy** of the repo's `ns3/`. After editing
  `ns3/*.cc`, re-copy into `scratch/multi_uav_simulation/` and rebuild, or you will be
  running stale code.

### Phase 2a — Flat baseline ✅ DONE (2026-07-26)

Whole vision pipeline validated with no namespaces and no sudo:
`Gazebo(single_uav.world) + SITL(127.0.0.1) → camera_relay(edge) → detector(YOLO) → gcs_receiver`.

- `/uav1/camera/image_raw` at ~6 Hz after throttling.
- Detector: **~44 ms/frame** on host x86.
- **Edge detection message = 76 bytes.** This is the number the entire experiment turns on.

### Phase 2b — netns + ns-3 host rehearsal ✅ DONE (2026-07-28)

Full end-to-end run with the impaired link, driven via **`Makefile.edge`** (symlinked to
`Makefile`) with one-word targets: `netns`, `gazebo`, `ns3`, `pos`, `sitl`, `relay`,
`detector`, `gcs`, plus `status`, `down`, `netns-down`.

Uses `worlds/small_city_single_uav.world` and `config/fastdds_udp_only.xml` (the SHM-bypass
fix), with `NS3_EXECUTABLE_PATH` set so the TAP creator can be found.

Since superseded for single-UAV runs by **`scripts/netns/launch_single_uav_netns.sh`**,
which performs the whole 8-stage bring-up in one script with a health gate after each stage
(see §7).

### Phase 3 — Prepare the Pi 4B ✅ DONE (2026-07-29)

Ubuntu 22.04 + ROS 2 Humble + `uav_vision` (all four nodes) + YOLO all working on the Pi;
verified `persons: 4` on a test image. Full guide: [`docs/PI_SETUP.md`](PI_SETUP.md).
Reusable one-shot installer: [`scripts/pi_setup.sh`](../scripts/pi_setup.sh).

Pi facts:
- Host `192.168.0.165`, hostname `uav2-pi.local`, user `anton`.
- Passwordless SSH from the host (`~/.ssh/id_ed25519`); passwordless sudo via
  `/etc/sudoers.d/90-anton-nopasswd`.
- Workspace `~/uav2_ws` (`src/uav_vision` rsynced from the repo), model at
  `~/models/yolov8n.pt`, venv `~/yolo_env`.
- Prefer SSH to the **IP** over `uav2-pi.local` — mDNS resolution intermittently fails.

**Performance:** YOLOv8n on the Pi 4 CPU runs **~6.5 s for the first frame, ~1–3 s warm**,
versus ~44 ms on the host. That is not a defect — *it is the experiment*. NCNN export could
gain roughly 2–3× later if a faster edge arm is wanted.

### Phase 4 — Camera over the wired sensor link ✅ DONE (2026-08-05)

The Pi receives the Gazebo camera over the cable and runs YOLO on it. Measured on the host
NIC while the Pi consumed the stream, with the camera at 20 Hz:

| | |
|---|---|
| Frame rate at the Pi | 19.8 Hz (20 requested) |
| Wire throughput | **481 Mbps** (57.3 MB/s), 42,544 pkt/s |
| **Dropped packets** | **0** |
| Link utilisation | 58% of 834 Mbps |

Zero drops at 42,500 packets/second. The wire figure exceeds the payload by ~5%, which is
exactly the IP/UDP/Ethernet header overhead.

**Detection confirmed visually**, not just by counts: the annotated stream
(`/uav1/camera/annotated`) was viewed in `rqt_image_view` during a flight and the boxes land
on the actual `person_standing` models.

**Recall: 2–4 of the 5 people per frame (~40–80%).** Not a defect — the five models sit in a
plus pattern only 1 m apart, so from the 45° oblique view the near figures partly occlude
the far ones, and at ~53 px tall the partly-hidden ones fall under `conf=0.4`. Ground truth
is exactly 5 and known from the world file, so **recall is measurable, not estimated** — use
it as a second axis alongside latency in Phase 6.

**What unblocked it:** raising `net.core.rmem_max` to 536870912 on both machines. A 2.76 MB
frame is ~1,900 UDP fragments; the default 208 KB buffer holds ~7% of one frame, so
reassembly never completed and the subscriber silently received nothing.

### Phase 5 — Detections across the ns-3 impaired link ✅ networking DONE (2026-08-05)

Detections now cross the simulated radio; the camera does not. See §3 for the VLAN design
and the measured 2.2 ms vs 59.0 ms proof.

Three changes made it work:

1. **`launch_single_uav_netns.sh` STEP 1c** — creates `br-uav2`, splits the Pi-facing NIC
   into VLAN 10 (`eth-cam`, carries `10.0.0.1`) and VLAN 42 (`eth-rf`, no IP, bridged to
   `tap-uav2`). Auto-skips when no Pi NIC is present, so host-only runs are unaffected.
2. **`scripts/netns/pi_hitl_link.sh`** — run with sudo on the Pi; creates `eth0.10`
   (`10.0.0.2`) and `eth0.42` (`10.42.0.12`).
3. **`gcs_receiver` moved into `gcsns`.** In the root namespace its only route to the Pi is
   the unimpaired cable, and every latency number would be meaningless.

The DDS whitelist in `config/fastdds_hitl_eth.xml` gained `10.42.0.10` and `10.42.0.12`.

**Still to do for Phase 5 proper:** collect detection latency and delivery statistics across
the channel as the drone flies out and buildings occlude the path. Loss is currently 0%
because the drone stays close to the GCS; `/ns3_link_rssi`, `/ns3_link_snr` and
`/link_obstacle_loss` are the topics to correlate against.

### Phase 6 — Edge-vs-ground HITL comparison ⬜ **NEXT**

Same experiment, real edge node. The edge arm is complete; the ground arm needs
`camera_relay` in ground mode so JPEG frames cross ns-3 instead of detections.

| | crosses the radio | inference runs | measured |
|---|---|---|---|
| **Edge** ✅ | ~118 B | on the Pi | 2,540 → ~1,000 ms |
| **Ground** ⬜ | ~100 KB JPEG/frame | on the host | ~44 ms |

With a 59 ms / 41 ms-jitter channel, the bandwidth difference is where the result lives.

> **⚠ Blocked on clock sync.** `metrics_logger` computes latency as
> `receipt_wall − send_wall`. Across two machines with unsynchronised clocks these numbers
> are meaningless — and plausible-looking, which is worse. Run chrony between the Pi and the
> host **before** collecting any Phase 6 data.

---

## 6. Network hardware — resolved the hard way

The Pi↔host link consumed several days. The findings are worth keeping because they were
counter-intuitive.

**The bandwidth wall.** Gazebo publishes `/uav1/camera/image_raw` at ~56 Hz ×
640×480×3 ≈ 900 KB/frame ≈ **400 Mbps**. Over WiFi the Pi received **zero frames** — large
DDS samples fragment and drop. Note that `camera_relay`'s `frame_rate_hz` throttle applies
*after* receipt, so it does not reduce link bandwidth; throttling must happen at the source.
**This bandwidth wall is precisely why the design mandates a wired sensor link.**

**The adapter saga.** The first USB adapter (`0fe6:9700`, labelled DM9601 by `lsusb`, in
reality a QTS1081B) gave contradictory readings — the host reported 100 Mbps full-duplex
while the Pi reported 10 Mbps on the *same direct cable*, which is impossible. Cheap USB
NICs frequently report their advertised maximum rather than the negotiated rate.

Settled by measuring **actual throughput with `iperf3`** rather than trusting `ethtool`:

| Setup | iperf3 result |
|---|---|
| Old QTS1081B adapter | ~4.5 Mbps, 24% loss |
| Swapped cable only | ~4.5 Mbps — **cable ruled out** |
| **New UGREEN USB3 gigabit (Realtek)** | **935 Mbps, 0 retransmits** |

Changing one variable at a time proved the adapter was the culprit. The camera's 72 Mbps
now fits with ~13× headroom; even ten drones (720 Mbps) would fit.

**Two concepts worth keeping straight:** *link speed* (`ethtool`, instant, no traffic
needed, and evidently lie-prone) versus *actual throughput* (`iperf3`, requires real data,
tells the truth). Also *admin-UP* (`ip link set eth0 up`) versus *operational/carrier*
state — `/sys/class/net/eth0/speed` returns "Invalid argument" while the interface is
admin-DOWN.

### Addressing — imperative loses to declarative

Every `ip addr add` on this project eventually vanished, and the lesson took three
repetitions to land. There are two layers:

```
  manager config on disk   NetworkManager / systemd-networkd   ← enforced, permanent
           │ reconciles, deleting anything it did not create
           ▼
  kernel live state        what `ip addr show` prints          ← temporary
```

A manager periodically compares the kernel against its own config and **removes addresses
it did not create**. So `ip addr add` always loses eventually. Check what owns an interface
before configuring it by hand:

```bash
systemctl is-active NetworkManager systemd-networkd dhcpcd
nmcli device status          # the STATE column
```

The two machines need **different tools**, which is easy to get wrong:

| | Host (Ubuntu Desktop) | Pi (Ubuntu Server) |
|---|---|---|
| Manager | NetworkManager | systemd-networkd |
| Tool | `nmcli connection modify` | netplan YAML |
| Config | `/etc/NetworkManager/system-connections/` | `/etc/netplan/99-cat6-link.yaml` |

Host, made persistent (survived an adapter unplug/replug):

```bash
sudo nmcli connection modify "Wired connection 2" ipv4.method manual \
     ipv4.addresses 10.0.0.1/24 ipv4.never-default yes ipv6.method disabled
```

`ipv4.never-default yes` matters — without it NM may install a default route via the cable
and break WiFi internet.

**Note the interface name is derived from the MAC** (`enx` + hex), so a *different* USB
adapter gets a different name and the profile stops matching. That happened when the
adapter was swapped: NM created `Wired connection 2` for the new MAC while the old profile
sat unused.

> **⚠ Two known-fragile items (2026-08-05)**
> - **The Pi's VLANs are not persistent.** `pi_hitl_link.sh` uses plain `ip` commands, so
>   every Pi reboot drops it back to untagged `eth0` while the host still expects VLAN 10 —
>   the symptom is `Destination Host Unreachable` with the carrier up. Needs a netplan file
>   with a `vlans:` section.
> - **NetworkManager re-adds `10.0.0.1` to the parent interface**, fighting the VLAN
>   `eth-cam` that should own it. The result is the same address on two interfaces and ARP
>   going to the wrong one. Fix is `ipv4.method disabled` on the wired profile, leaving the
>   script's VLAN as the only holder.

---

## 7. Host-side pipeline — current state

Single-UAV runs are driven by one script,
[`scripts/netns/launch_single_uav_netns.sh`](../scripts/netns/launch_single_uav_netns.sh),
in eight stages, each gated on a health check so failures surface at the correct step:

```
[0] cleanup       kill stale procs; delete namespaces / bridges / taps
[1] topology      gcsns + uav1ns; veth ↔ bridge ↔ tap
[2] mgmt link     172.31.1.1 ↔ .2  ← FDM path, bypasses ns-3
[3] ns-3          binds the TAPs        gate: 4 carriers up + ping across
[4] gazebo        root ns, FDM plugin   gate: alive after 30 s
[5] micro_ros_agent   in gcsns          gate: UDP 2019 listening
[6] SITL          in uav1ns             gate: UDP 9003 listening
[7] verification  DDS + MAVLink         gate: /ap/v1/navsat publisher
[8] drone_bridge  in gcsns              gate: alive after 3 s
```

Mission, detector and `metrics_logger` are deliberately **not** auto-launched, so
edge/ground/nav-only runs stay independently controllable.

### The SITL reboot loop — RESOLVED 2026-07-31

Step 6 used to reboot-loop (~25 restarts in 15 s) as soon as the Gazebo FDM connected with
DDS enabled inside `uav1ns`. Each reboot dropped the FDM link, so DDS never published and
step 7 failed with *"No publisher on /ap/v1/navsat"*.

**Fix:** wrap `arducopter` in `strace -f -e trace=none -o /dev/null`. Being ptrace-traced
suppresses the auto-reboot. `-e trace=none` decodes no syscalls — strace is there purely
for the ptrace side-effect, not for logging. Requires `sudo apt install -y strace`.

Two things worth remembering:

1. **The loop was invisible to the liveness check.** SITL's reboot is `execv()` of itself
   (`HAL_SITL::actually_reboot`), which **preserves the PID** — so `kill -0 "$SITL_PID"`
   cheerfully reported "alive" throughout. The only evidence was repeated
   `Loaded defaults from …` lines in the log. A stronger gate would count those lines, or
   require the same PID to persist for N seconds.
2. **The mechanism is not proven.** Most likely a timing or signal-delivery race that
   ptrace perturbs. This is a *workaround*, not a fix, and may regress on an ArduPilot
   update. To diagnose properly, capture the trace instead of discarding it:
   `strace -f -tt -e trace=execve,sendto,recvfrom -e signal=all -o /tmp/sitl_trace.log …`
   then read the ~200 lines before each `execve`.

The trigger is the *combination* of DDS enabled **+** SITL inside `uav1ns` **+** FDM over
the veth (`--sim-address 172.31.1.1`). Flat FDM on `127.0.0.1` with DDS works fine. The
ArduPilot commit and kernel version differences were red herrings, and `--wipe` was not the
cause.

---

## 8. Environment gotchas

**Host:**
- All simulation processes (gzserver / SITL / ns-3 / DDS) must run with the command sandbox
  **disabled** — the sandbox kills gzserver (exit 144) and blocks DDS. `pkill` also returns
  144 under the sandbox; use `kill <PID>` from `pgrep` instead.
- `detector` needs `ultralytics`, which lives only in the repo `venv` → run it as
  `venv/bin/python install/uav_vision/lib/uav_vision/detector` with `PYTHONPATH` including
  `/opt/ros/humble/lib/python3.10/site-packages` and the `uav_vision` install site. The
  other three vision nodes run fine under `ros2 run` (system python3 has rclpy + cv2 +
  numpy).
- `ARDUPILOT_HOME=~/ardu_ws/src/ardupilot`. Launch env for flat runs:
  `scratchpad/flat_env.sh`.
- The netns scripts are **not idempotent** — re-running `make netns` over existing
  namespaces fails with "Cannot find device vethNn" because the veth is already inside the
  namespace. Run `make netns-down` first.
- `netns_down.sh` also pkills gzserver, so a teardown kills a running Gazebo. Restart it.
- Gazebo's many `[Err] Failed to find mesh file model://.../*.obj` lines are harmless
  missing decorative meshes (ambulances, cars, stop lights), **not** a crash.
- `make gazebo` runs gzserver headless. `gzclient` is an optional viewer and can hang
  "not responding" on the heavy city heightmap — Force Quit is safe, it does not touch
  gzserver.
- SITL's log line `Bind 127.0.0.1:9003 for SITL in` is **cosmetically wrong** — the source
  binds `0.0.0.0` and prints a hardcoded string. Do not build theories on it.

**Pi 4B:**
- **`pip install ultralytics` pulls a CUDA torch wheel (`…+cu130`)** whose CPU kernels use
  dot-product / fp16 SIMD that the Cortex-A72 lacks (`/proc/cpuinfo` flags: `fp asimd
  evtstrm crc32 cpuid` — **no** `asimddp`, no `fphp`). YOLO then dies with **Illegal
  instruction (SIGILL, exit 132)**.
  **Fix — install CPU-only torch first:**
  `pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu`, *then*
  ultralytics. Now baked into `scripts/pi_setup.sh` and `PI_SETUP.md` §3.4.
- Raspberry Pi Imager: you **must** select **Device = Raspberry Pi 4**. Selecting "Pi 5"
  hides the Ubuntu 22.04 option, because 22.04 is not Pi 5 compatible.
- cloud-init applies `network-config` **only on first boot** — editing the card after a
  boot does nothing. Re-flash to re-apply WiFi/SSH settings.
- For headless first boot prefer 2.4 GHz and set the correct WLAN country (LK).
- **The venv is version-pinned and must stay that way:**
  `numpy==1.26.4` and `opencv-python==4.10.0.84`. **Never run `pip install -U` there.**
  `cv_bridge` is a compiled C++ extension and two separate upgrades break it:
  - NumPy 2.x → `AttributeError: _ARRAY_API not found` (built against the 1.x C ABI)
  - OpenCV 5.0 → `KeyError: 16` in `cv2_to_imgmsg` (16 is `CV_8UC3`; OpenCV 5 changed the
    constants cv_bridge reads to build its type table)

  They are linked, because `opencv-python >= 4.11` *requires* `numpy >= 2` — only the older
  pair coexists. That pair is exactly what the host has, which is why the `anton-ap_dds`
  branch always worked there. Neither failure is ARM-specific.
- **The Pi has its own copy of the code** at `~/uav2_ws/src/uav_vision`; it does not track
  the host repo. After any edit: `rsync -av ros2/uav_vision/ anton@10.0.0.2:~/uav2_ws/src/uav_vision/`
  then `colcon build --packages-select uav_vision`. Stale code once left the detector
  subscribed to a topic nobody published — it started cleanly and then sat silent.

**QoS — a slow subscriber can throttle the simulator:**
- Gazebo's camera publisher is **RELIABLE**. A RELIABLE subscriber makes it wait for an
  acknowledgement of every frame, so the Pi (≈1 fps, and also sending a 2.76 MB annotated
  image back) dragged the **whole simulation's camera down to 0.27 Hz**.
- Worse, when the Pi rebooted mid-run its unacknowledged samples left the publisher stalled
  at its retransmission heartbeat — frames arriving exactly 3.144 s apart, ±0.4 ms. That
  fixed spacing is the tell: render times vary, timers do not.
- **Sensor streams must use `BEST_EFFORT`, depth 1.** A frame that arrives late is worthless
  anyway, and the publisher can then never be held back. Keep `RELIABLE` for detections,
  commands and configuration — things that must not be lost.

**Diagnosis order — check `/clock` first.** The world runs in lockstep
(`real_time_update_rate: -1`), so Gazebo only steps when SITL sends servo packets. If SITL
stops, `/clock` freezes and every downstream rate decays to zero. A stalled sim looks
exactly like a slow camera; hours were lost chasing rendering and QoS when the actual cause
was `/clock` not ticking.

---

## 9. Remaining hard problems

1. **Clock sync across machines — the Phase 6 blocker.** `metrics_logger` computes latency
   as `receipt_wall − send_wall`. Across two machines with unsynchronised clocks these
   numbers are meaningless, and plausible-looking, which is worse. Run chrony between the
   Pi and the host **before** collecting any Phase 6 data. Now more urgent than ever: with
   the radio link adding ~59 ms, end-to-end latency is the headline measurement.
2. **Persistence on the Pi.** Its VLANs (`eth0.10`, `eth0.42`) come from
   `pi_hitl_link.sh` and are lost on every reboot. Needs netplan with a `vlans:` section.
3. **NetworkManager vs the host VLAN.** NM keeps re-adding `10.0.0.1` to the parent
   interface, duplicating the address `eth-cam` should own. Fix with
   `ipv4.method disabled` on the wired profile.
4. **DDS discovery over a lossy link.** Default multicast discovery is fragile through
   bridges plus packet loss. Loss is 0% today only because the drone stays close to the
   GCS; once it flies out, consider unicast discovery peers (Fast DDS static discovery).
5. **The strace workaround is unexplained** and could regress on an ArduPilot update.
6. **NCNN @ 640 unmeasured.** One export (`imgsz=[384,640]`) completes the timing matrix
   and should put the edge arm near 470 ms, ~5× the original baseline.
7. **`camera_relay` ground mode untested** — the other half of the Phase 6 comparison.

---

## 10. Measured reference

| Metric | Value |
|---|---|
| Link throughput (iperf3, steady) | ~834 Mbps, 0 retransmits |
| Camera | 1280×720 RGB, **2.76 MB/frame** |
| Camera at 20 Hz / 5 Hz | 481 Mbps / ~111 Mbps |
| Camera link latency | 2.2 ms, 0.8 ms jitter |
| **Radio link latency (ns-3)** | **59.0 ms avg**, 40.8 ms jitter, 0% loss |
| Inference — PyTorch @ 960×544 | **2,540 ms** (0.39 fps) |
| Inference — PyTorch @ 640×384 | ~1,100 ms |
| Inference — NCNN @ 960×544 | **~1,000 ms** (0.74 fps) |
| Inference — NCNN @ 640×384 | ~470 ms *(predicted, not measured)* |
| Detection payload vs image | **~118 B vs 2.76 MB** |
| Recall | 2–4 of 5 people per frame |

Resolution and engine effects are **separable and roughly multiplicative** — inference
scales close to linearly with pixel count (0.47 pixel ratio → 0.43 time ratio).

Throttling the camera 20 → 5 Hz alone cut inference from 1,343 to ~1,000 ms. The same model
on a static image runs in 623 ms, so the Pi was losing roughly half its inference capacity
to deserialising frames it discarded.

---

## 11. File map

| Path | Purpose |
|---|---|
| `scripts/netns/run_hitl.sh` | **One-command HITL run** — pipeline + Pi detector + GCS, with pre-flight checks |
| `scripts/netns/launch_single_uav_netns.sh` | 8-stage host bring-up; STEP 1c adds the VLANs + `br-uav2` |
| `scripts/netns/pi_hitl_link.sh` | **Run on the Pi** — splits `eth0` into VLAN 10 / VLAN 42 |
| `scripts/netns/kill_all_netns.sh` | Teardown (also kills gzserver) |
| `scripts/netns/wireless_up.sh` · `management_up.sh` | Multi-UAV namespace/link creation |
| `scripts/yolo_detect_node.py` | The working detector (from `anton-ap_dds`) + timing; runs on the Pi |
| `scripts/pi_setup.sh` | One-shot Pi 4B installer (incl. CPU-torch fix) |
| `scripts/make_runbook_pdf.py` | Regenerates `docs/HITL_RUNBOOK.pdf` |
| `docs/HITL_RUNBOOK.pdf` | Printable command sequence + troubleshooting |
| `docs/PI_SETUP.md` | Full Pi flashing + setup guide |
| `config/fastdds_hitl_eth.xml` | **DDS profile for HITL** — interface whitelist, UDP-only |
| `config/fastdds_udp_only.xml` | UDP-only DDS profile (SHM bypass), host-only runs |
| `ros2/uav_vision/uav_vision/` | `camera_relay`, `detector`, `gcs_receiver`, `metrics_logger` |
| `params/uav1_dds_netns.parm` | DDS pointed at the agent in `gcsns` (10.42.0.10:2019) |
| `models/iris_1_netns/model.sdf` | Camera (1280×720, 45° pitch, 0.6 rad FOV, 5 Hz) + FDM plugin |
| `worlds/small_city_single_uav_netns.world` | City world, lockstep physics, 5 `person_standing` |

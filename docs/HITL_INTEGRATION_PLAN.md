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

```
   ┌──────────────────────── HOST PC ────────────────────────┐
   │                                                          │
   │  Gazebo (physics + camera)      ArduPilot SITL           │
   │        │                              │                  │
   │        │  ① SENSOR LINK               │  FDM 9002/9003   │
   │        │  172.31.x.x — UNIMPAIRED     │  (never impaired)│
   │        │  wired Ethernet to the Pi    │                  │
   │        │                              │                  │
   │        │        ns-3 wireless channel (TapBridge)        │
   │        │        loss · latency · fading · range          │
   │        │              ▲                                  │
   │        │              │ ② WIRELESS LINK                  │
   │        │              │ 10.42.0.x — IMPAIRED             │
   │   gcsns: gcs_receiver + metrics_logger                   │
   └────────┼──────────────┼──────────────────────────────────┘
            │              │
   ┌────────▼──────────────▼─────────┐
   │      RASPBERRY PI 4B            │
   │  camera_relay → detector (YOLO) │
   └─────────────────────────────────┘
```

**① The sensor link (172.31.x.x) is deliberately unimpaired.** It carries the Gazebo
camera feed to the Pi. In reality this is a ribbon cable between a camera module and the
companion computer *inside one airframe* — degrading it would be physically meaningless
and would corrupt the experiment. It is a direct wired path that never touches ns-3.

**② The wireless link (10.42.0.x) is the real radio**, simulated by ns-3. Everything the
drone sends to the ground station crosses it and receives realistic loss, latency and
fading. **This is what the experiment measures.**

Because the GCS is only reachable on the `10.42.0.x` subnet, DDS routes Pi→GCS traffic
through the impaired channel automatically — the same routing trick the namespaces use.
No application code needs to know which link it is on.

The same principle governs the **FDM link** on the host (`172.31.1.1 ↔ 172.31.1.2` over a
veth pair): the 1 kHz physics conversation between SITL and Gazebo bypasses ns-3 entirely,
because it represents a flight controller's own IMU wiring, not a radio.

> **⚠ Open decision — which namespace does the Pi replace?**
> Earlier notes locked "2 drones total; the Pi replaces **`uav2ns`** (10.42.0.12 /
> 172.31.2.2), UAV1 stays fully on the host." But all recent host-side work has converged
> on the **single-UAV** pipeline (`launch_single_uav_netns.sh`), which creates only
> `gcsns` + `uav1ns` and stubs out UAV2/UAV3 as bare unbridged TAPs. Phase 4 as written
> also has the Pi consuming `/uav1/camera/image_raw`.
> **This must be settled before Phase 5.** Two coherent options:
> - **(a) Pi = uav1ns** (10.42.0.11 / 172.31.1.2). Simplest: the single-UAV pipeline is
>   already built and debugged; the Pi just takes over the one drone's vision. Recommended
>   unless a second drone is needed for the result.
> - **(b) Pi = uav2ns** (10.42.0.12 / 172.31.2.2). Keeps a host-side drone for
>   side-by-side comparison in one run, but requires reviving the multi-UAV netns scripts.

## 4. Phase status

| Phase | Description | Status |
|---|---|---|
| 1 | ns-3 3.38 on the host | ✅ DONE 2026-07-26 |
| 2a | Flat (no netns) vision pipeline baseline | ✅ DONE 2026-07-26 |
| 2b | netns + ns-3 host rehearsal | ✅ DONE 2026-07-28 |
| 3 | Prepare the Pi 4B (OS, ROS, YOLO) | ✅ DONE 2026-07-29 |
| — | Host netns SITL reboot loop | ✅ RESOLVED 2026-07-31 |
| — | Pi↔host link bandwidth | ✅ RESOLVED 2026-07-30 (935 Mbps) |
| 4 | Camera over the wired sensor link, detector on the Pi | ⬜ **NEXT** |
| 5 | Insert the ns-3 impaired link (full HITL) | ⬜ Not started |
| 6 | Run the edge-vs-ground HITL comparison | ⬜ Not started |

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

### Phase 4 — Camera over the wired sensor link ⬜ **NEXT**

Prove the HITL compute path with no ns-3 impairment yet: one plain wired link, same
`ROS_DOMAIN_ID`, flat DDS. Host runs Gazebo + SITL + `gcs_receiver`; the Pi runs
`camera_relay` + `detector`.

**Success criterion:** the Pi detects people in the live SITL/Gazebo feed and detections
appear on the host.

**Critical setup note:** the Pi and host are on **both** WiFi and Ethernet simultaneously.
DDS must be pinned to the `10.0.0.x` Ethernet interface, or it will discover peers over
WiFi and the camera stream will collapse (see §6). Bind the interface explicitly in the
Fast DDS profile rather than hoping DDS picks correctly.

### Phase 5 — Insert the ns-3 impaired link ⬜

Replace the namespace veth in the relevant bridge with the physical path to the Pi. The Pi
gets a wireless address behind `br-uavN` → `tap-uavN` → ns-3, alongside its direct
`172.31.x.x` sensor address. Resolve the §3 open decision (uav1ns vs uav2ns) first.

### Phase 6 — Edge-vs-ground HITL comparison ⬜

Same experiment, real edge node. Collect CSVs for both modes and compare latency, delivered
bandwidth, and detection throughput as the ns-3 channel degrades.

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

### ⚠ Current network state (2026-07-31) — needs re-establishing

- Host UGREEN adapter `enx00e04c680050` is **UP but has no IP address**. The `10.0.0.1/24`
  was set with `ip addr add`, which is **temporary and cleared on reboot**. The Pi's
  `eth0` `10.0.0.2/24` needs the same treatment.
- Host WiFi is currently on `172.20.10.2/28` (`wlp0s20f3`), **not** the `192.168.0.x`
  network the Pi joined. The Pi is therefore unreachable on both paths right now.
- **Before Phase 4:** re-add both static IPs and bring the Pi back onto a shared network
  (or use the direct cable). Making these addresses persistent — a netplan file on each
  side — would remove a recurring source of lost time.

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

---

## 9. Remaining hard problems

1. **Clock sync across machines.** `metrics_logger` computes latency as
   `receipt_wall − send_wall`. Across two machines with unsynchronised clocks these numbers
   are meaningless. Run chrony (or PTP) between the Pi and the host **before** collecting
   any Phase 6 latency data. Currently unaddressed and it will silently corrupt results.
2. **DDS interface selection.** With the Pi and host on both WiFi and Ethernet, DDS must be
   pinned to the fast wired path. Blocking for Phase 4.
3. **DDS discovery over a lossy link.** Default multicast discovery is fragile through
   bridges plus packet loss. Use unicast discovery peers (Fast DDS static discovery /
   CycloneDDS `Peers`) once ns-3 impairment is inserted in Phase 5.
4. **Persistent static IPs.** The `10.0.0.x` addresses are re-added by hand after every
   reboot on both machines. Worth a netplan file on each side.
5. **Namespace assignment for the Pi** — the §3 open decision. Must be settled before
   Phase 5.
6. **The strace workaround is unexplained** and could regress on an ArduPilot update.

---

## 10. File map

| Path | Purpose |
|---|---|
| `scripts/netns/launch_single_uav_netns.sh` | Single-UAV 8-stage bring-up (gcsns + uav1ns) |
| `scripts/netns/wireless_up.sh` · `management_up.sh` | Multi-UAV namespace/link creation |
| `scripts/netns/netns_down.sh` · `kill_all_netns.sh` | Teardown (also kills gzserver) |
| `scripts/pi_setup.sh` | One-shot Pi 4B installer (incl. CPU-torch fix) |
| `docs/PI_SETUP.md` | Full Pi flashing + setup guide |
| `Makefile.edge` (→ `Makefile`) | One-word targets for the flat/netns host rehearsal |
| `ros2/uav_vision/uav_vision/` | `camera_relay`, `detector`, `gcs_receiver`, `metrics_logger` |
| `config/fastdds_udp_only.xml` | UDP-only DDS profile (SHM bypass) |
| `params/uav1_dds_netns.parm` | DDS pointed at the agent in `gcsns` (10.42.0.10:2019) |
| `models/iris_1_netns/model.sdf` | ArduPilot FDM plugin: 172.31.1.1/.2, ports 9002/9003 |
| `worlds/small_city_single_uav_netns.world` | City world, lockstep physics tuned for SITL |

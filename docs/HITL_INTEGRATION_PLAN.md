# HITL Integration Plan — Raspberry Pi 4B edge node ↔ SITL host

**Author:** anton · **Branch:** `ground-vs-edge-processing-RPi` (sub-branch of
`ground-vs-edge-processing`) · **Last updated:** 2026-08-13

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
| **Edge processing** | Detection messages (**72–433 B**, see §12) | The Pi's CPU (~250 ms/frame) |

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
              ① SENSOR LINK — unimpaired                      │  72-433 B
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

**Verified by measurement**, from the Pi:

| Link | Path | Latency |
|---|---|---|
| `10.0.0.1` | VLAN 10, direct cable | min 1.0 / avg **1.4** / max 1.8 ms, jitter 0.12 ms, 0% loss (60 pings) |
| `10.42.0.10` | VLAN 42 → ns-3 → gcsns | min 10.1 / avg **74.5** / max 158.0 ms, jitter 38.4 ms, 0% loss (30 pings) |

> **Caveat (2026-08-13):** these were taken while ns-3 mobility was frozen, so they
> describe a **static 38.4 m** link, not a flight. The 53x ratio and the shape of the
> result stand — the radio path is genuinely contended while the cable is not — but the
> absolute radio figure is one point on a curve, not a mission average. See §5.

**~53× the latency**, and jitter larger by more than two orders of magnitude. The ~7× spread
*within* the radio path (10.1 ms minimum against a 74.5 ms mean) shows ns-3 is not applying a
fixed delay: it is a contended CSMA/CA channel where packets queue and back off. Use ≥30
pings — the first packet includes ARP across the simulated channel and skews short samples
badly (a 2-ping sample reported a misleading 167 ms average). The radio figure repeats: an
independent run gave 77.4 ms, within 4%.

**The sensor figures were taken while the camera was streaming.** Both logical links share
one cable, so this had to be checked rather than assumed. Latency under the full 110 Mbps
load is the same as on an idle cable, and jitter is in fact lower — a continuously active
interface avoids idle-state wake-up delay. `fq_codel` is active on both machines with a
5 ms target, and byte queue limits had reduced the Pi's driver ring to about one packet.

**Bring-up order.** `eth-cam` is persistent (NetworkManager on the host, netplan on the Pi),
so the camera link is up from boot and does not depend on the simulator. `eth-rf`,
`br-uav2` and the taps are created by the launch script and torn down with it, which is
correct: without ns-3 there is no radio.

```bash
# Camera link works with nothing running:
ping -c 3 10.0.0.2        # from the host

# Radio link needs the stack:
./scripts/netns/sitl_init.sh            # wait for PIPELINE READY
ping -c 30 10.42.0.10 | tail -2         # from the Pi
```

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
| — | Clock sync between the machines | ✅ RESOLVED 2026-08-10 (60 us) |
| — | Persistent addressing, both machines | ✅ RESOLVED 2026-08-10 |
| — | **Frozen ns-3 mobility** (nodes never moved) | ✅ **RESOLVED 2026-08-13** |
| — | ARP answered on the wrong interface | ✅ RESOLVED 2026-08-13 |
| 6 | Run the edge-vs-ground HITL comparison | ⬜ **NEXT** — ground arm never run |

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
  Superseded: the payload is **72–433 B** and scales with detection count — see §12.
  76 B was the Phase 2a message, when a frame carried at most one detection.

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

**Detection confirmed visually**, not just by counts: boxes land on the actual
`person_standing` models, at confidences of 0.42–0.59.

The edge node does **not** publish an annotated image. Verify visually by running the
detector once with `-p show_window:=True` on a machine that has a display, or by drawing the
boxes host-side from the `bbox` values already carried in `/detections/uav1`. Do this before
accepting any timing result — a detector that boxes ground texture passes every numerical
check.

**Recall: 2–4 of the 5 people per frame (~40–80%), at 1280x720.** Not a defect — the five models sit in a
plus pattern only 1 m apart, so from the 45° oblique view the near figures partly occlude
the far ones, and at ~53 px tall the partly-hidden ones fall under `conf=0.4`. Ground truth
is exactly 5 and known from the world file, so **recall is measurable, not estimated** — use
it as a second axis alongside latency in Phase 6.

**What unblocked it:** raising **both** `net.core.rmem_max` *and* `net.core.wmem_max` to
536870912 on both machines. A 2.76 MB frame is ~1,900 UDP fragments.

* `rmem_max` too small → the default 208 KB buffer holds ~7% of one frame, reassembly never
  completes, and the subscriber silently receives nothing.
* `wmem_max` too small → the profile's 16 MB `<sendBufferSize>` is silently clamped to
  208 KB, and the **publisher** discards fragments before they reach the NIC. Measured
  symptom: 0.19 Hz delivered against a 5 Hz source, host transmitting 2 Mbps instead of
  110 Mbps. See §8.

Both are checked on both machines by `rpi_init.sh` (step 0). Persist them in
`/etc/sysctl.d/60-ros2-dds.conf` on **both** machines — a `sysctl -w` is lost at reboot, and
the failure it causes does not look like a buffer problem.

### Phase 5 — Detections across the ns-3 impaired link ✅ networking DONE (2026-08-05)

Detections now cross the simulated radio; the camera does not. See §3 for the VLAN design
and §10 for the measured 1.5 ms vs 77.4 ms proof.

Three changes made it work:

1. **`launch_single_uav_netns.sh` STEP 1c** — creates `br-uav2`, splits the Pi-facing NIC
   into VLAN 10 (`eth-cam`, carries `10.0.0.1`) and VLAN 42 (`eth-rf`, no IP, bridged to
   `tap-uav2`). Auto-skips when no Pi NIC is present, so host-only runs are unaffected.
2. **`/etc/netplan/60-hitl-vlans.yaml` on the Pi** — creates `eth0.10` (`10.0.0.2`) and
   `eth0.42` (`10.42.0.12`) persistently, so they survive reboots. See §6.
3. **`gcs_receiver` moved into `gcsns`.** In the root namespace its only route to the Pi is
   the unimpaired cable, and every latency number would be meaningless.

The DDS whitelist in `config/fastdds_hitl_eth.xml` gained `10.42.0.10` and `10.42.0.12`.

**Still to do for Phase 5 proper:** collect detection latency and delivery statistics across
the channel as the drone flies out and buildings occlude the path. `/ns3_link_rssi`,
`/ns3_link_snr` and `/link_obstacle_loss` are the topics to correlate against.

> **Correction (2026-08-13).** This section previously said *"Loss is currently 0% because
> the drone stays close to the GCS."* That explanation was wrong. Loss was ~0% because
> **every ns-3 node was frozen at its start position for the whole flight** — see
> "Frozen ns-3 mobility" below. With mobility working, the GCS↔UAV distance varies
> 75.6–224.7 m across a mission and delivery loss rose from 0.11% to 0.70%.

### Frozen ns-3 mobility — RESOLVED 2026-08-13

**Every channel measurement taken before this date describes a stationary fleet.**

`launch_netns_v2.sh` started `scripts/world_pos_publisher.py`, and even warned
*"ns-3 mobility will be static"* if it could not find it.
`launch_single_uav_netns.sh` — which replaced it, and which `run_hitl.sh` calls — never
started it at all. Nothing published `/uav_world_positions`, so ns-3 kept the formation set
by its own CLI defaults (`distance=50`, `uavAltitude=20`) from the first second to the last:

```
t=2.0s    UAV1=(0.0,0.0,30.0)   d01=38.4m
t=356.0s  UAV1=(0.0,0.0,30.0)   d01=38.4m      <- after a full 2-lap patrol
```

Nothing errors. The link still carries traffic, pings still return, and the numbers look
entirely plausible — they simply describe a fixed 38.4 m link instead of the aircraft that
is flying. `DynamicObstacleLossModel` has no changing geometry, so RSSI and SNR never
respond to the mission. The only visible symptom was ns-3's own integration check:

```
missing node IDs: GCS(0) UAV1(1) UAV2(2) UAV3(3)
```

**Two fixes:**

1. **STEP 4b in `launch_single_uav_netns.sh`** starts the publisher after Gazebo, in the
   root namespace, with Gazebo's DDS profile (it cannot discover `/gazebo/model_states`
   otherwise). It must `set +u` before sourcing ROS — the script runs `set -euo pipefail`
   and `/opt/ros/humble/setup.bash` reads `AMENT_TRACE_SETUP_FILES` unset, which aborts the
   subshell before python starts. The first attempt failed exactly this way.
2. **`mirror=2:1`** (new parameter in `world_pos_publisher.py`) copies UAV1's position onto
   node 2. The Pi is ns-3 node 2 while the aircraft it serves is node 1, so with one UAV in
   the world node 2 was never covered. Both are on the same airframe, so this is the
   physically correct position, not a fudge. Remove it if the Pi ever shares node 1.

**After the fix**, measured over one 2-lap patrol:

| | before | after |
|---|---|---|
| GCS position | `(-24, 0, 0)` — CLI default | `(-70, -7, 2.9)` — the Gazebo model |
| GCS↔UAV1 distance | fixed 38.4 m | **75.6 – 224.7 m** (~9.5 dB of path loss swing) |
| Pi's node (2) | frozen 50 m away | `d12 = 0.0 m`, tracks the aircraft |
| Missing nodes | all four | only `UAV3(3)` — correct, nothing is there |
| Detection loss | 2 / 1,767 = 0.11% | **6 / 855 = 0.70%** |

**Two lessons worth keeping.** A silent degradation is worse than a crash: the health gate
*did* print the warning, but `run_hitl.sh` only grepped the pipeline log for
`PIPELINE READY`, so a stage that failed without stopping the run said nothing. It now
surfaces any `WARNING` after bring-up. And `set -u` plus a third-party setup script is a
recurring trap — every other stage in that launcher escapes it by using `bash -lc`.

**Still open:** the obstacle ray-caster reports only link `(0,1)`, so
`missing link pairs: (0,2) …` persists and the GCS↔Pi path — the one detections actually
take — gets distance and fading but **no building shadowing**. The same mirroring argument
applies: node 2 sits at identical coordinates, so `(0,1)`'s loss is what `(0,2)` should see.

### Phase 6 — Edge-vs-ground HITL comparison ⬜ **NEXT**

Same experiment, real edge node. The edge arm is complete; the ground arm needs
`camera_relay` in ground mode so JPEG frames cross ns-3 instead of detections.

| | crosses the radio | inference runs | measured |
|---|---|---|---|
| **Edge** ✅ | 72–433 B | on the Pi | 1,343 → **236 ms** (§10) |
| **Ground** ⬜ | ~100 KB JPEG/frame | on the host | ~44 ms |

With a 77 ms / 55 ms-jitter channel, the bandwidth difference is where the result lives.

> **Clock sync is done** (§9): the Pi tracks the host to within 60 us over the
> camera link, so `metrics_logger`'s `latency_ms` is now valid. Any CSV recorded
> before 2026-08-10 is not — delete them so they cannot be mixed in.

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
| Config | `/etc/NetworkManager/system-connections/` | `/etc/netplan/60-hitl-vlans.yaml` |

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

**Both sides are now persistent (2026-08-10).** The camera link is up from boot and does
not depend on the simulator, which is what "plug and play" means here.

**Host** — the address lives on the VLAN, and the parent holds none:

```bash
sudo nmcli connection modify "Wired connection 2" \
     ipv4.addresses "" ipv4.gateway "" ipv4.method disabled
sudo nmcli connection add type vlan con-name eth-cam ifname eth-cam \
     dev <enx...> id 10 ip4 10.0.0.1/24
sudo nmcli connection modify eth-cam ipv4.never-default yes connection.autoconnect yes
```

`never-default` matters: without it the host may try to route the internet down the Pi
cable and lose WiFi. The launch script no longer deletes `eth-cam` at start or teardown —
it only creates it if missing.

**Pi** — `/etc/netplan/60-hitl-vlans.yaml`:

```yaml
network:
  version: 2
  ethernets:
    eth0: {dhcp4: no, optional: true}
  vlans:
    eth0.10: {id: 10, link: eth0, addresses: [10.0.0.2/24]}
    eth0.42: {id: 42, link: eth0, addresses: [10.42.0.12/24]}
```

`optional: true` stops boot waiting two minutes for a DHCP reply that will never come.

> **Keep a second route to the Pi while changing any of this.** WiFi is the rescue path; a
> wrong netplan file with no WiFi means fetching a monitor and keyboard. Note also that
> `netplan apply` briefly takes WiFi down.

> **One failure mode remains.** If the cable is unplugged, the parent goes down and
> NetworkManager does not re-activate the VLAN when the carrier returns. Symptom:
> `eth-cam` shows `LOWERLAYERDOWN` or disappears. Fix: `sudo nmcli connection up eth-cam`.

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
  acknowledgement of every frame, so the Pi at ≈1 fps dragged the **whole simulation's
  camera down to 0.27 Hz**.
- Worse, when the Pi rebooted mid-run its unacknowledged samples left the publisher stalled
  at its retransmission heartbeat — frames arriving exactly 3.144 s apart, ±0.4 ms. That
  fixed spacing is the tell: render times vary, timers do not.
- **Sensor streams must use `BEST_EFFORT`, depth 1.** A frame that arrives late is worthless
  anyway, and the publisher can then never be held back. Keep `RELIABLE` for detections,
  commands and configuration — things that must not be lost.

**Socket buffers — check BOTH directions.** `rmem_max` and `wmem_max` are separate
parameters and an error in either produces the same symptom: too few frames arriving.
- Send-side starvation is **invisible to every interface counter**. `tx_dropped` on the host
  and `rx_errors` on the Pi both stay at 0, because the fragments are discarded at the
  socket layer and never transmitted. The wire looks perfectly healthy.
- The symptom surfaces three layers up. When this bit, the detector reported 0 humans in 93
  of 94 frames and the patrol mission timed out at every waypoint — both of which invite a
  detector or flight-control explanation, and neither of which was the cause.
- **Restart the stack after changing them.** Fast DDS applies socket options only at
  participant creation, so a running `gzserver` keeps the old buffer.
- Isolate source from transport by measuring the rate **on the host** as well as on the Pi.
  Host at 5 Hz and Pi at 0.19 Hz localises the fault to transport in one step. Use a
  `BEST_EFFORT` subscriber to do it — `ros2 topic hz` defaults to RELIABLE and will silently
  match nothing against a BEST_EFFORT publisher, which looks identical to a dead topic.

**One address must live on exactly ONE interface.** A leftover
`/etc/netplan/99-cat6-link.yaml` gave `10.0.0.2` to the untagged `eth0` while
`60-hitl-vlans.yaml` gave the same address to `eth0.10`. Netplan merges files in number
order without warning about the conflict, so both took effect. Three unrelated-looking
symptoms followed:

- `ping` and SSH worked — unicast picks one device and either happens to work
- multicast never arrived — the kernel resolves a group join by looking up which device
  owns the address, found `eth0` first (index 2 before index 4), and registered the
  membership there. The host sends tagged, so it arrives on `eth0.10`. Proven by repeating
  the join with an explicit interface index (`ip_mreqn`): membership then landed on
  `eth0.10` and every packet arrived.
- **Fast DDS segfaulted at participant creation** — the whitelist entry matched two
  devices, producing duplicate locators. No error message, no output, just exit 139.

Check before every session; it must print `1`:

```bash
ip -4 addr show | grep -c "10.0.0.2/24"       # on the Pi
grep -l "eth0" /etc/netplan/*.yaml            # only one file should touch eth0
```

The general lesson: read `ip addr show` in full rather than checking that your address is
present. **The fault was not a missing address; it was an extra one.**

**Diagnosis order — check `/clock` first.** The world runs in lockstep
(`real_time_update_rate: -1`), so Gazebo only steps when SITL sends servo packets. If SITL
stops, `/clock` freezes and every downstream rate decays to zero. A stalled sim looks
exactly like a slow camera; hours were lost chasing rendering and QoS when the actual cause
was `/clock` not ticking.

---

## 9. Time synchronisation ✅ SOLVED 2026-08-10

`metrics_logger` computes latency as `receipt_wall − send_wall`, one timestamp per machine.
The Pi's clock was found to be **2 h 55 m behind** the host, which would have made every
such figure nonsense while looking plausible in sign and shape.

Three facts combined to cause it:

1. **A Pi 4B has no battery-backed clock** (`timedatectl` reports `RTC time: n/a`), so every
   boot starts from whatever `fake-hwclock` last wrote.
2. The Pi had **no internet route**, so `systemd-timesyncd` could never correct it.
3. `NTPSynchronized=no` is not an error anywhere; the clock is simply wrong.

**The fix: the Pi takes time from the host over the camera link**, not from the internet.
That link is 1.4 ms away and always present, so it beats public NTP over WiFi on both
accuracy and availability — and the testbed no longer needs an internet connection at all.

```bash
# HOST — chrony replaces systemd-timesyncd, which is client-only and cannot serve
sudo apt install -y chrony
sudo tee -a /etc/chrony/chrony.conf > /dev/null <<'EOF'

allow 10.0.0.0/24
local stratum 10        # keep serving even if the host's own upstream is unreachable
EOF
sudo systemctl restart chrony

# PI — prefer the host, keep the public pools as fallback
sudo apt install -y chrony
sudo sed -i '1i server 10.0.0.1 iburst prefer minpoll 4 maxpoll 6' /etc/chrony/chrony.conf
sudo systemctl restart chrony
```

Verify on the Pi — `chronyc sources` should show `^*` on `10.0.0.1`:

```
Reference ID : 0A000001 (10.0.0.1)
Root delay   : 0.00137 s        <- matches the measured cable RTT
System time  : 0.000061 s slow  <- 60 microseconds
```

**60 µs against a 75 ms radio latency is an error of 0.08%**, so the clock no longer limits
anything. For comparison, public NTP over WiFi gave a root delay of 174 ms.

Hardware PTP was considered and is not available: `ethtool -T` reports
`PTP Hardware Clock: none` on both the Pi's built-in NIC and the host's USB adapter. Software
PTP would reach 10–50 µs, no better than chrony already achieves here.

On a deployed aircraft the companion computer would take time from the flight controller's
GPS receiver. The arrangement here — time from the vehicle's authoritative source over a
short unimpaired link — mirrors that, and is in fact more accurate than the ~1–10 ms typical
of MAVLink `SYSTEM_TIME`.

---

## 10. Inference optimisation ✅ 2026-08-11

Inference went from **1,343 ms to 236 ms**, a factor of 5.7, entirely through
configuration. No hardware was changed and no accuracy was lost.

| Change | Inference | Why |
|---|---|---|
| starting point | 1,343 ms | 1280x720 camera, PyTorch, annotated frame returned to the host |
| remove the annotated return stream | 1,093 ms | the Pi was serialising and sending 2.76 MB per detection |
| camera 1280x720 -> 640x384 | 1,013 ms | matched the sensor to the model input; 110 -> 29.5 Mbps |
| PyTorch -> NCNN | 271 ms | ARM-optimised inference engine, 3.8x |
| NCNN -> OpenVINO | **236 ms** | a further 13% |

Delivered rate rose from **0.98 fps to 3.47 fps** measured in the live pipeline,
so the edge node now processes roughly two thirds of a 5 Hz camera stream
instead of one frame in five.

### Back-end comparison

Nine configurations, same image, same threshold, on the Pi 4B. Run it with
`scripts/bench_backends.py`.

| Back-end | Median | Speed-up | Detections |
|---|---|---|---|
| **YOLO11n OpenVINO** | **236 ms** | **4.35x** | 3 |
| YOLOv8n NCNN 384x640 | 270 ms | 3.80x | 3 |
| YOLO11n NCNN 384x640 | 271 ms | 3.78x | 3 |
| YOLOv8n PyTorch imgsz=320 | 291 ms | 3.53x | 3 |
| YOLO11n MNN | 342 ms | 3.00x | 3 |
| YOLOv8n NCNN 544x960 | 579 ms | 1.77x | 3 |
| YOLOv8n PyTorch imgsz=480 | 589 ms | 1.74x | 3 |
| YOLO11n PyTorch | 1,001 ms | 1.03x | 3 |
| YOLOv8n PyTorch imgsz=640 | 1,027 ms | 1.00x | 3 |

All nine returned the same detection count and were stable across ten calls,
so this is a comparison of speed alone.

### Published Pi 5 rankings do not transfer to a Pi 4B

LearnOpenCV benchmarked the same back-ends on a Pi 5 (see `docs/references/`).
The ordering is different:

| Back-end | Pi 5 (published) | Pi 4B (measured here) |
|---|---|---|
| OpenVINO | 80.9 ms, **3.6x faster than NCNN** | 236 ms, only **1.15x** faster |
| MNN | 115.8 ms, 2.5x faster than NCNN | **342 ms, SLOWER than NCNN** |
| NCNN | 292.1 ms | 270 ms |

The reason is the instruction set, and MNN prints it at startup:

```
The device supports: i8sdot:0, fp16:0, i8mm:0, sve2:0, sme2:0
```

The Pi 5's Cortex-A76 is ARMv8.2-A and has all of those. The Pi 4B's
Cortex-A72 is ARMv8.0-A and has none. Back-ends that lean on them lose most of
their advantage, which is why MNN's ranking inverts.

Note also that the Pi 4B beats the Pi 5 on NCNN here (270 ms against 292 ms).
That is not the board; it is the export shape. 384x640 is 60% of the pixels of
the 640x640 square export used in the published test.

### FP16 gives nothing on this board

`quantize=16` was exported and measured: **256 ms against 262 ms**, inside the
run-to-run spread. `fp16:0` above is why. The A72 can store and convert
half-precision but cannot compute in it, so NCNN converts FP16 weights back to
FP32 before use and the bandwidth saving is cancelled by the conversion. FP16
would be expected to help on a Cortex-A76.

INT8 is not offered by the ultralytics NCNN exporter, and `i8sdot:0` means the
gain would be far smaller than the usual 2x. Doing it by hand with
`ncnn2table` and `ncnn2int8` is the remaining lever; it is left as future work
because it risks accuracy on subjects that are only about 27 px tall.

### The trap that nearly went unnoticed

An NCNN model exported at 384x640 and called with `imgsz=640` returns:

```
[3, 13, 13, 13, 13, 13]
```

Correct on the first call, then thirteen boxes spanning most of the image at
confidence 0.88, on every call after. Nothing errors.

The cause: for NCNN the input shape is fixed at export time, and `imgsz`
overrides the shape ultralytics uses to decode the output coordinates.
`imgsz=640` means 640x640 square, which is not 384x640. Note that `imgsz` is
`[height, width]`, the opposite order to the usual W x H notation.

**Both detectors now drop `imgsz` for any model that is not a `.pt`** —
`scripts/yolo_detect_node.py` and `ros2/uav_vision/uav_vision/detector.py`. The
ROS node had no guard at all until 2026-08-11, and `run_hitl.sh` was passing it
`imgsz=960` with the 544x960 NCNN export, so the one-command path was running
the failure above. Any detection counts from a `run_hitl.sh` run before that
date are suspect. Testing for a directory is not enough: `.mnn` is a single
file and is equally frozen.

**Reproduced on the host** (bus.jpg, YOLO11n exported at 384x640), which also
shows the two engines fail differently:

| | call 1 | calls 2-6 |
|---|---|---|
| NCNN, `imgsz=640` passed | 3 persons, conf 0.85 | **2 persons, conf 0.74** — silent |
| NCNN, `imgsz` dropped | 3 persons, 0.85 | 3 persons, 0.85 |
| OpenVINO, `imgsz=640` passed | `RuntimeError: shape [1,3,384,640] vs (1,3,640,640)` | — |

OpenVINO raises; **NCNN does not**, and settles into a stable wrong answer that
looks entirely plausible. That is why the fault survived so long.

`scripts/bench_backends.py` calls every back-end ten times and rejects any whose
detection count varies, however fast it is — a benchmark that timed the model
once would have recommended the broken model.

---

## 11. Remaining hard problems

1. **Search pattern covers the wrong area.** Correlating the detector and mission logs
   (possible only now the clocks agree) shows detections cluster at the **centre** and at
   Corner 1, with nothing at the other seven corner arrivals. The patrol flies a rectangle
   *around* the subjects. A perimeter is the wrong shape for finding people in an area;
   parallel sweeps would cover it. Do NOT change the waypoint coordinates themselves —
   they are correct, see the note in §5.
2. **`camera_relay` ground mode untested** — the other half of the Phase 6 comparison. The
   code exists and has never been run, so there is currently no comparison at all.
3. **DDS discovery over a lossy link.** Default multicast discovery is fragile through
   bridges plus packet loss. Loss is 0% today only because the drone stays close to the
   GCS; once it flies out, consider unicast discovery peers (Fast DDS static discovery).
4. **The strace workaround is unexplained** and could regress on an ArduPilot update. It is
   also not free: the tracer consumes ~5.7% CPU, because `-e trace=none` suppresses decoding
   but not the per-syscall ptrace stops. Worth testing `--seccomp-bpf`.
5. **NCNN @ 640 unmeasured.** One export (`imgsz=[384,640]`) completes the timing matrix.
6. ~~**ARP answers on the wrong interface.**~~ ✅ **RESOLVED 2026-08-13.**
   `net.ipv4.conf.all.arp_ignore = 1` and `arp_announce = 2` are applied on the Pi and
   verified live. Keep them in `/etc/sysctl.d/60-ros2-dds.conf`, not `sysctl -w`.

7. **Obstacle loss covers only link (0,1).** The ray-caster never reports `(0,2)`, so the
   GCS↔Pi path — the one detections cross — has no building shadowing. See §5.

---

## 12. Measured reference

Delivered configuration: camera 640x384 at 5 Hz, FOV 0.6 rad, pitch 45 deg;
YOLO11n OpenVINO on the Pi, `conf=0.4`, class 0 only.

| Metric | Value |
|---|---|
| Link negotiation | 1000 Mb/s full duplex |
| Link throughput (iperf3, steady) | ~834 Mbps, 0 retransmits |
| **Camera, delivered** | **640x384 RGB, 0.74 MB/frame** — 29.5 Mbps at 5 Hz (3% of the link) |
| Camera, before optimisation | 1280x720, 2.76 MB/frame — 481 Mbps at 20 Hz, 110 Mbps at 5 Hz |
| **Camera delivered to the Pi** | **5.000 Hz** vs 5 Hz source, sigma = 4.5 ms |
| Real-time factor (`/clock`) | 0.998 |
| Camera link latency, 60 pings | min 1.0, **avg 1.4**, max 1.8, jitter 0.12 ms |
| Camera link latency **under 110 Mbps load** | unchanged — no bufferbloat |
| **Radio link latency (ns-3), 30 pings** | min 10.1, **avg 74.5**, max 158.0, jitter 38.4 ms, 0% loss — *at a static 38.4 m, see §5* |
| Radio, independent repeat | avg 77.4 ms — agrees within 4% |
| Radio vs camera path | **53x latency**, jitter over 2 orders of magnitude larger |
| **Clock offset, Pi to host** | **60 us** (chrony over the camera link, root delay 1.37 ms) |
| **Inference — OpenVINO, standalone** | **236 ms** — fastest correct back-end |
| Inference — NCNN 384x640, standalone | 270 ms |
| Inference — NCNN 384x640, in pipeline | **277 ms**, 3.47 fps over 859 frames |
| Inference — PyTorch 640, standalone | 1,027 ms |
| Inference — PyTorch 640, in pipeline | 1,013 ms, 0.98 fps over 1,039 frames |
| Contention (pipeline minus standalone) | **18 ms**, constant regardless of back-end |
| Starting point, before optimisation | 1,343 ms (1280x720 + annotated return stream) |
| **Detection payload** | **72 B** empty, 143 B 1 person, 211 B 2, 361 B 4, **433 B** 5 (1,765 msgs) |
| Payload vs image | ~5,200x smaller than a 0.74 MB frame, for one detection |
| Detection quality | 1-4 of 5 subjects per frame when in view, conf 0.42-0.59 |
| Mission | 2 laps, 8 corners, all reached within 2.3 m, no timeouts |
| **GCS-UAV distance, in flight** | **75.6 - 224.7 m** (was a frozen 38.4 m before 2026-08-13) |
| **Detection delivery, moving** | **849 / 855 = 99.3%** (0.70% loss); 99.89% when frozen |

Resolution and engine effects are **separable and roughly multiplicative** — inference
scales close to linearly with pixel count (0.47 pixel ratio -> 0.43 time ratio).

**Contention is now 18 ms**, and it is a fixed cost rather than a proportion: the same
18 ms appears whether inference takes 1,013 ms or 277 ms. It is the cost of receiving and
deserialising one frame, which competes with inference for the same four cores.

The 720 ms contention originally measured was real and has been designed out. Two changes
did it: removing the 2.76 MB annotated frame the Pi returned on every detection, and
matching the camera resolution to the model input so 3.75x less data crosses the cable.
See section 10.

At 3.47 fps against a 5 Hz camera the edge node now uses about two thirds of the stream
and drops the rest. The over-supply is deliberate: it keeps the frame the detector picks up
under 200 ms old. Supplying only 3.5 Hz would save 1 MB/s and make every frame up to
285 ms stale, which on a moving aircraft is a metre of displacement.

> The radio minimum of 10.1 ms against a 74.5 ms mean — a ~7x spread on one link — is the
> tell that ns-3 is modelling a contended channel with queueing and back-off, not applying a
> fixed delay. Always take at least **30 samples**: a 2-sample ping includes ARP across the
> simulated channel and returned 167 ms for a link that measures ~75 ms over 30.

---

## 13. File map

| Path | Purpose |
|---|---|
| `scripts/netns/rpi_init.sh` | **Step 0** — verify both Pi boards: link, config, buffers, clock |
| `scripts/netns/sitl_init.sh` | **Step 1** — cold-start the host pipeline; ends at PIPELINE READY |
| `scripts/netns/detector_start.sh` | **Step 2** — Pi detectors over SSH + `gcs_receiver` per board |
| `scripts/netns/run_missions.sh` | **Step 3** — fly `two_drone_mission.py` |
| `scripts/netns/launch_single_uav_netns.sh` | 8-stage host bring-up; STEP 1c adds the VLANs + `br-uav2` |
| `scripts/netns/pi_hitl_link.sh` | One-off VLAN setup on the Pi; superseded by `/etc/netplan/60-hitl-vlans.yaml`, kept for a machine that has not been configured yet |
| `scripts/netns/kill_all_netns.sh` | Teardown (also kills gzserver) |
| `scripts/netns/wireless_up.sh` · `management_up.sh` | Multi-UAV namespace/link creation |
| `scripts/yolo_detect_node.py` | The working detector (from `anton-ap_dds`) + timing; runs on the Pi |
| `scripts/pi_setup.sh` | One-shot Pi 4B installer (incl. CPU-torch fix) |
| `scripts/make_runbook_pdf.py` | Regenerates `docs/HITL_RUNBOOK.pdf` |
| `docs/HITL_RUNBOOK.pdf` | Printable command sequence + troubleshooting |
| `docs/PI_SETUP.md` | Full Pi flashing + setup guide |
| `docs/report/hitl_edge_processing.tex` | FYP report section on the HITL work |
| `docs/report/testing_validation.tex` | FYP report section on testing and validation |
| `config/fastdds_hitl_eth.xml` | **DDS profile for HITL** — interface whitelist, UDP-only |
| `config/fastdds_udp_only.xml` | UDP-only DDS profile (SHM bypass), host-only runs |
| `ros2/uav_vision/uav_vision/` | `camera_relay`, `detector`, `gcs_receiver`, `metrics_logger` |
| `params/uav1_dds_netns.parm` | DDS pointed at the agent in `gcsns` (10.42.0.10:2019) |
| `models/iris_1_netns/model.sdf` | Camera (1280×720, 45° pitch, 0.9 rad FOV, 5 Hz) + FDM plugin |
| `worlds/small_city_single_uav_netns.world` | City world, lockstep physics, 5 `person_standing` |

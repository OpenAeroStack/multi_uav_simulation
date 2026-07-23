# Multi-UAV Simulation — System Architecture

**A hardware-in-the-loop-style co-simulation that couples a physics simulator
(Gazebo), autopilot firmware (ArduPilot SITL), and a network simulator (NS-3)
so that three drones fly in a 3-D world while a *realistic wireless link* — with
distance-based path loss, obstacle shadowing, and fading — is modelled between
them and a ground control station in real time.**

The network side is **four** nodes: a stationary ground control station (GCS)
and the three UAVs, giving six modelled links — 3 GCS↔UAV plus 3 UAV↔UAV. The
GCS links matter most: the station sits at ground level, so it is by far the
likeliest to be occluded by a building.

This document is written for someone who has **never used Gazebo, ROS 2, or
NS-3**. Every term is explained the first time it appears. Read it top to
bottom and you should be able to understand, run, and reason about the whole
stack.

---

## 0. The three simulators, in one paragraph

Think of a real drone fleet. There are three separate concerns: (1) *where the
drones are and what the world looks like* — that is **Gazebo**, a physics +
3-D world simulator; (2) *the flight-control brain of each drone* — that is
**ArduPilot SITL** ("Software In The Loop": the real autopilot firmware
compiled to run on your PC instead of on a flight controller board); and (3)
*the radio link between the drones* — that is **NS-3**, a discrete-event
network simulator that models Wi-Fi at the packet level. These three programs
run as separate OS processes and talk to each other over **ROS 2** (a
publish/subscribe messaging middleware — think of it as a shared bulletin board
of named "topics" that programs post to and read from) and, for the *actual
network packets*, over Linux **TAP devices** (virtual network cards). The
glue that makes them agree on drone positions and obstacle effects is a set of
small ROS 2 nodes and one custom NS-3 propagation-loss model.

---

## 1. Overall architecture diagram

```
                                 ROS 2 (Humble) publish/subscribe graph
    ══════════════════════════════════════════════════════════════════════════════════════

      /gazebo/model_states            /uav_world_positions            /link_obstacle_loss
          (20 Hz)                          (10 Hz)                          (10 Hz)
             │                          ┌─────┴───────┐                        │
             │                          │             │                        │
             ▼                          ▼             ▼                        ▼
   ┌───────────────────────┐   ┌──────────────────┐   ┌───────────────────────────────────────┐
   │  GAZEBO  (gzserver)   │   │ world_pos_       │   │              NS-3 process               │
   │                       │   │ publisher.py     │   │    three_uav_tapbridge_integrated       │
   │ ┌───────────────────┐ │   │  (relay node)    │   │ ┌───────────────────────────────────┐  │
   │ │ physics engine    │ │   └──────────────────┘   │ │ THREAD B: rclcpp spin (ROS I/O)   │  │
   │ │ (ODE)             │ │                          │ │  OnPositions / OnObstacleLoss     │  │
   │ ├───────────────────┤ │                          │ │   └─▶ writes PLAIN buffers only   │  │
   │ │ obstacle raycast  │─┼──────── /link_obstacle_loss│ ├──────────────┬────────────────────┤  │
   │ │ plugin (.cc)      │ │         (6 links)        │ │ THREAD A: Simulator::Run()        │  │
   │ ├───────────────────┤ │                          │ │  ApplyFeed (50 Hz) drains buffers │  │
   │ │ gazebo_ros_state  │─┼──── /gazebo/model_states  │ │  Wi-Fi PHY + propagation chain    │  │
   │ │ plugin (.so)      │ │                          │ │  DynamicObstacleLossModel          │  │
   │ └───────────────────┘ │                          │ │  TapBridge  ⇄  Linux TAP devices  │  │
   │   (4 nodes: gcs +     │                          │ │  PublishStats ─▶ /ns3_link_rssi   │  │
   │    iris_1..3)         │                          │ │                 /ns3_link_snr     │  │
   └───────────┬───────────┘                          └─┴────────────────────┬──────────────┴──┘
               │ Gazebo⇄ArduPilot lockstep                                    │  Layer-2 Ethernet
               │ (flight-dynamics model, UDP)                                 │  frames over TAP
               ▼                                                              ▼
   ┌───────────────────────┐                              ┌────────────────────────────────────────┐
   │  ArduCopter SITL × 3   │   MAVLink over TCP           │  Linux network namespaces (isolated     │
   │  (real autopilot fw)   │   127.0.0.1:5760/5770/5780   │  mini-networks) + veth + bridges + taps │
   │  sysid 1 / 2 / 3       │◀────────────────────────────▶│  gcsns  10.42.0.10  (node 0)             │
   └───────────────────────┘        │                     │  uav1ns 10.42.0.11  (node 1)             │
               ▲                     │                     │  uav2ns 10.42.0.12  (node 2)             │
               │ pymavlink           │                     │  uav3ns 10.42.0.13  (node 3)             │
               │                     │                     └────────────────────────────────────────┘
        ┌──────┴───────────────┐     │
        │ multi_drone_mission_ │─────┘  (commands the drones; see §9 note — this path
        │ new.py  (mission)    │         currently bypasses the NS-3 radio)
        └──────────────────────┘
```

**How to read it:** solid vertical/diagonal lines are *data flows*. The top
band is the ROS 2 message bus. The NS-3 box has two internal threads (A and B)
— that split is central and explained in §8. The bottom-right box is the real
Linux networking that carries *actual packets* through the simulated radio.

---

## 2. The tools, one by one

| Tool | What it is | Role in this stack |
|---|---|---|
| **Gazebo (Classic 11)** | 3-D physics + sensor simulator | Simulates the world, the drones' bodies, and the obstacles (walls). Hosts two *plugins* (shared libraries loaded into Gazebo): the **obstacle raycast plugin** and the **gazebo_ros_state** plugin. |
| **ArduPilot SITL (ArduCopter)** | The real drone autopilot firmware compiled for the PC | One process per drone. Runs the actual flight-control code, talks to Gazebo for physics (lockstep flight-dynamics), and exposes MAVLink (the drone control protocol) on a TCP port. |
| **ROS 2 (Humble)** | Publish/subscribe middleware | The "bulletin board". Every inter-process message about positions, obstacle loss, and RSSI travels as a ROS 2 *topic*. |
| **NS-3 (3.38)** | Discrete-event network simulator | Models the 802.11a (Wi-Fi) radio links among the GCS and the three drones at the packet level, applying path loss + obstacle loss + fading. Bridges *real* Linux packets into that simulated radio. |
| **`world_pos_publisher.py`** | Small ROS 2 relay node (added) | Reads ground-truth GCS + drone poses from Gazebo and republishes them in the flat array format NS-3 understands. Goes silent if Gazebo dies rather than rebroadcasting stale poses. |
| **`multi_drone_mission_new.py`** | Mission script (pymavlink) | Connects to each SITL over MAVLink and flies the choreographed mission (take off → diverge → hold → return). |
| **Linux netns + TAP + veth + bridge** | Kernel networking primitives | Give each drone its own isolated network stack so *real* traffic between drones is forced through the NS-3 radio. |

**Plugin** = a shared library (`.so`) that Gazebo loads at startup and runs
inside its own process. **Node** = a single ROS 2 participant (a program, or a
part of one, that can publish/subscribe). **Topic** = a named channel on the
ROS 2 bus with a fixed message type.

---

## 3. The ROS 2 topics (the "nervous system")

Everything the simulators tell each other flows through these topics. All the
custom ones use `std_msgs/Float32MultiArray`, which is simply a flat list of
floats — cheap and language-agnostic. The *convention* for how to interpret
that flat list is what matters, and both ends must agree exactly.

| Topic | Message type | Published by | Subscribed by | Rate | Payload layout |
|---|---|---|---|---|---|
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | `gazebo_ros_state` plugin | `world_pos_publisher.py` | 20 Hz | `name[]`, `pose[]` (every model in the world, ground truth) |
| `/uav_world_positions` | `std_msgs/Float32MultiArray` | `world_pos_publisher.py` | **NS-3**, obstacle plugin | 10 Hz | `[id, x, y, z, …]` — one 4-tuple per node, **including the GCS as id 0** |
| `/link_obstacle_loss` | `std_msgs/Float32MultiArray` | obstacle raycast plugin | **NS-3** | 10 Hz | `[i, j, loss_dB, …]` — one 3-tuple per *pair*, all 6 links |
| `/ns3_link_rssi` | `std_msgs/Float32MultiArray` | **NS-3** (`PublishStats`) | loggers / your tools | 2 Hz | `[a, b, rx_dBm, …]` — faded received power per pair |
| `/ns3_link_snr` | `std_msgs/Float32MultiArray` | **NS-3** (`PublishStats`) | loggers / your tools | 2 Hz | `[a, b, snr_dB, …]` — same power measured against the −94 dBm noise floor |
| `/marker` | `ignition.msgs.Marker` | obstacle plugin | gzclient (viewer) | 10 Hz | line per pair — red = blocked, green = clear UAV↔UAV, blue = clear GCS↔UAV (visual only) |

**Why a flat float array and not a structured message?** Because it crosses a
C++/Python boundary and a Gazebo/NS-3 boundary; a primitive array has zero
schema-compatibility risk. The cost is that the index convention is implicit —
if one side reads 3-tuples where the other wrote 4-tuples, it silently breaks.
That is exactly why §3 spells out each layout.

**Identity convention (critical).** There is now exactly **one** numbering, and
it is the NS-3 node id. Nothing anywhere applies an offset:

| id | NS-3 node | Gazebo model | netns | IP | TAP |
|---|---|---|---|---|---|
| 0 | node 0 | `gcs` | `gcsns` | 10.42.0.10 | `tap-gcs` |
| 1 | node 1 | `iris_1` | `uav1ns` | 10.42.0.11 | `tap-uav1` |
| 2 | node 2 | `iris_2` | `uav2ns` | 10.42.0.12 | `tap-uav2` |
| 3 | node 3 | `iris_3` | `uav3ns` | 10.42.0.13 | `tap-uav3` |

Gazebo models are matched by name *prefix*, so `iris_1_demo` matches `iris_1`.

> **Why the offset flag was deleted, not defaulted to 0.** Earlier versions
> carried a `--rosNodeOffset` knob that was added to every incoming id. It was
> load-bearing for exactly one intermediate state — when NS-3 had a GCS node but
> Gazebo did not, so UAVs numbered 0–2 had to be shifted to 1–3. Once the GCS
> became a real Gazebo model that shift disappeared, leaving a flag whose only
> remaining effect was to break a working setup: set it to 1 against the current
> feed and the GCS position lands on node 1, UAV1 on node 2, UAV3 falls off the
> end, and node 0 is never fed at all. An offset kept in sync by hand across
> three processes is a silent-failure trap, so the fix was to make the numbering
> agree rather than to keep the arithmetic.

Two defences remain, because a mismatch here is otherwise invisible: incoming
ids outside `0..N-1` produce a throttled **warning** instead of being silently
skipped, and `CheckIntegration()` reports at t≈10 s any node or link the feed
never covered (§6).

The `gcs_antenna_height` (2.9 m = 0.9 m cabinet + 2.0 m mast) must match between
the world's plugin block and `world_pos_publisher.py`, or NS-3 and the
ray-caster will disagree about where the ground station is.

---

## 4. The network plumbing (how real packets reach the simulated radio)

NS-3 does not just *report* a link quality — it can carry **real Linux
packets** through the simulated Wi-Fi. This is what makes tools like `iperf3`,
`ping`, or a real DDS stack testable against the RF model.

```
   uav1ns (a private network stack)                       NS-3 process
   ┌───────────────────────────┐                    ┌────────────────────────┐
   │ app (iperf3, DDS, …)       │                    │  node 1  ── Wi-Fi PHY ──┼───┐
   │ veth1n  10.42.0.11/24      │                    │            (802.11a)    │   │
   └──────────┬────────────────┘                    └────────────────────────┘   │
              │ veth pair                                     ▲                    │  simulated
        ┌─────┴─────┐        ┌──────────┐   TAP (a virtual     │ TapBridge          │  Wi-Fi
        │  br-uav1  │────────│ tap-uav1 │   NIC the kernel      │ "UseLocal" mode    │  channel
        │  (bridge) │        └──────────┘   and NS-3 share)    │                    │  (Yans)
        └───────────┘                                          ▼                    │
                                                     ┌────────────────────────┐    │
   gcsns  10.42.0.10  ── br-gcs  ── tap-gcs  ────────┤  node 0  ── Wi-Fi PHY ──┼───┤
   uav2ns 10.42.0.12  ── br-uav2 ── tap-uav2 ────────┤  node 2  ── Wi-Fi PHY ──┼───┤
   uav3ns 10.42.0.13  ── br-uav3 ── tap-uav3 ────────┤  node 3  ── Wi-Fi PHY ──┼───┘
                                                     └────────────────────────┘
```

**One bridge per node is deliberate, and it is what makes the whole setup
falsifiable.** Because no two namespaces share a bridge, the only layer-2 path
between any pair of them runs through NS-3. If they shared one, traffic would
take the Linux shortcut and every throughput or loss number would describe the
kernel's bridge rather than the simulated radio — while still looking perfectly
plausible. `verify_datapath.sh` audits this directly (§9.1).

* **Network namespace (`netns`)** — a completely isolated copy of the Linux
  network stack (its own interfaces, routes, ARP table). `uav1ns` cannot see
  `uav2ns` except through the wire you give it.
* **veth pair** — a virtual Ethernet cable: two ends, one inside the namespace
  (`veth1n`, holds the drone's IP `10.42.0.11`), one in the host (`veth1h`).
* **bridge (`br-uav1`)** — a virtual switch joining the veth end and the tap.
* **TAP device (`tap-uav1`)** — a virtual network card whose "wire" is a file
  descriptor. The Linux kernel writes frames into it; **NS-3's TapBridge reads
  them out** and injects them into the matching NS-3 node, and vice-versa.
* **TapBridge "UseLocal" mode** — NS-3 acts as a transparent Layer-2 bridge for
  the single MAC address behind the tap. The taps are created *owned by your
  user* (`ip tuntap … user`) so NS-3 can attach without running as root.

**Net effect:** the four namespaces behave as if they were four real machines on
one Wi-Fi ad-hoc network — and every frame between them is subject to the path
loss, obstacle loss, and fading NS-3 computes. Send 500 kbit/s of UDP from
`10.42.0.11` to `10.42.0.12` and it flows node1→(radio)→node2; block that link
with an obstacle and the traffic stops. (Proven with `iperf3` — see
`scripts/test_scripts/iperf3_channel_test.sh` — and, more rigorously, by
`verify_datapath.sh` in §9.1.)

`ChecksumEnabled=true` is set in NS-3 because these are *real* packets from the
kernel; NS-3 must compute valid Ethernet/IP checksums or the receiving kernel
drops them.

---

## 5. The Gazebo obstacle raycast plugin

File: `gazebo_plugins/src/obstacle_raycast_plugin.cc`. This is the component that
turns *geometry* (a wall between two drones) into a *number* (dB of extra
signal loss). It is a Gazebo **world plugin**, so it runs inside gzserver and
has direct access to every object's true pose and collision shape.

### What it does every cycle (throttled to 10 Hz)

For each **pair** `(i, j)` of the four nodes — so all six links, GCS included:

1. **Get both endpoints.** Prefer positions from `/uav_world_positions`; if that
   topic is silent, fall back to reading the model's true world pose directly.
   (This fallback is why obstacle detection kept working even before we added
   the position publisher.) For the GCS the ray starts at the antenna on top of
   the mast, `gcs_antenna_height` above the model origin — not at the base of
   the cabinet.
2. **Cast a ray** from node `i` toward node `j` (`CastRay`). Gazebo's ray
   reports the **first** solid it hits. But the first hit is often the ground
   plane or a drone's own body — not a real obstacle — so the plugin **steps
   past filtered hits and re-casts** (up to 5 hops). It also ignores hits
   *beyond* the destination (`dist > link_len − 0.5`).
3. If a genuine obstacle is hit, **measure how much material the signal crosses**
   (`ObstacleThickness`): cast a *second* ray backwards from `j` to `i` to find
   the obstacle's far (exit) face. The gap between entry and exit faces along
   the link is the true material thickness `t`.
4. **Convert to a dB loss** (`ComputeObstacleLoss`) and publish
   `[i, j, loss_dB]` on `/link_obstacle_loss`. Also draws a marker line in the
   viewer: red blocked, green clear UAV↔UAV, blue clear GCS↔UAV.

### What is never an obstacle (`IsFilteredEntity`)

Four classes of hit are stepped past rather than treated as blockage, and the
same list is shared by the forward and backward casts so the two can never
disagree:

* the **ground plane**;
* any model matching `uav_prefix` — a drone's own body or a fleet member's;
* the **GCS structure** itself, so a link is never blocked by its own antenna
  mast. (The `gcs` model's name must therefore stay free of material keywords: a
  rename that dropped "gcs" would turn the station into an obstacle that blocks
  every link it terminates.)
* anything tagged **`noloss`** — thin street furniture such as poles, signs,
  hydrants and postboxes, which a ray hits often but which attenuate nothing.

### The obstacle-loss equation

```
        L_obs = L_e + 0.5 · t          (dB)
```

* **`L_e`** — a fixed *entry/exit penetration loss* that depends on the wall
  material (matched by substring in the model name):

  | Material (name contains) | `L_e` (dB) |
  |---|---|
  | `glass` | 4 |
  | `foliage` | 5 |
  | `wood` | 8 |
  | `vehicle` | 12 |
  | `concrete` | 15 |
  | `metal` | 20 |
  | *(no recognised keyword)* | **0 — RF-transparent** |

* **`0.5 · t`** — a *bulk attenuation* term: 0.5 dB for every metre of material
  the ray passes through, with `t` clamped to ≤ 20 m. A drone just behind a wall
  loses less than one deep inside a thick structure — which is the physically
  correct direction. (An earlier version wrongly used the *free-space* distance
  from the wall to the drone, penalising far drones more; that was corrected.)

If the exit face can't be resolved, the plugin falls back to `L_obs = L_e`
alone. `L_obs = 0` means clear line-of-sight (no real obstacle on the ray).

> **The default is now 0 dB, not 15 dB concrete.** An untagged model contributes
> nothing and the thickness term is skipped entirely. This puts material choice
> firmly with the world author — but it also means a **mistyped keyword fails
> silently**, as a perfectly clear link rather than an error.
>
> This matters right now, because **`small_city_base.world` contains no tagged
> models at all**: 38 sidewalks, a gas station, an asphalt plane, the ground
> plane, the GCS and the 3 drones, none carrying a material keyword. Every link
> in that world therefore reports exactly 0 dB obstacle loss, always, and the
> obstacle half of this model is inert — runs there exercise path loss and LoS
> fading only. The city assets that would block links are vendored under
> `models/` but not yet placed in the world. For a working obstacle demo use
> `worlds/multi_uav_plugin.world`, which has a 4 dB `glass_wall` between
> `iris_1` and `iris_2`.

---

## 6. The NS-3 scenario: `three_uav_tapbridge_integrated.cc`

This is the network-side program. It builds a 4-node Wi-Fi network, wires it to
the Linux taps, embeds a ROS 2 node, and applies the custom loss model. Walking
through what it constructs:

1. **Real-time engine.** `SimulatorImplementationType = RealtimeSimulatorImpl`.
   Normally NS-3 runs *as fast as possible* in virtual time; here it is locked
   to the wall clock so it can stay in step with Gazebo and the real packet
   flow. (Consequences discussed in §10.)
2. **Four nodes** (`nodes.Create(4)`) — the GCS as node 0, the three drones as
   nodes 1–3.
3. **The propagation-loss chain** — the heart of the RF realism, a linked list
   of models each transmission passes through:
   ```
       DynamicObstacleLossModel  ──▶  LogDistancePropagationLossModel
       (obstacle shadowing +           (distance-based path loss)
        line-of-sight-aware fading)
   ```
   plus a `ConstantSpeedPropagationDelayModel` for propagation delay. The chain
   is built by hand rather than with `YansWifiChannelHelper::AddPropagationLoss`,
   because `PublishStats()` needs direct handles to *both* models to decompose
   the loss (see §6.1). It is attached to a `YansWifiChannel`.
4. **Wi-Fi PHY/MAC** — **802.11a** ad-hoc (`AdhocWifiMac`, no access point),
   `TxPower = 20 dBm`, `RxSensitivity = −82 dBm`, and
   **`ConstantRateWifiManager` at `OfdmRate6Mbps`** for both data and control.
5. **Internet stack + IPs** on the `10.42.0.0/24` subnet (largely vestigial in
   UseLocal TapBridge mode, where the Linux side's addressing is what counts).
6. **Mobility** — in ROS mode *all four* nodes get
   `ConstantVelocityMobilityModel` and are driven entirely by
   `/uav_world_positions`, the GCS included. Initial UAV layout is a triangle
   with side `distance` (default 50 m) at `uavAltitude`; the GCS default
   (−24, 0, 0) matches `small_city_base.world`.
7. **TapBridge install** — attaches node `k` to `tapNames[k]`, i.e. `tap-gcs`,
   `tap-uav1..3` (see §4). Skippable with `--enableTap=false`.
8. **Embedded ROS 2 node** (`Ns3RosNode`) — subscribes to
   `/uav_world_positions` and `/link_obstacle_loss`, publishes `/ns3_link_rssi`
   and `/ns3_link_snr`.
9. **`PublishStats`** — every 0.5 s, per pair, computes and exports the full
   loss decomposition (§6.1).
10. **`CheckIntegration`** — a one-shot report at t≈10 s (§6.2).

### 6.0 Why the GCS position is *not* hard-coded

The GCS used to be a `ConstantPositionMobilityModel` with its coordinates fixed
in the C++. That is a second source of truth for where the station is: let it
drift from the Gazebo world pose and NS-3's path loss and the ray-caster's
occlusion silently describe two different geometries. Taking the GCS position
from the same feed as everything else makes that impossible. There is a
secondary trap in the old arrangement too — `ApplyFeed` looks up
`ConstantVelocityMobilityModel` specifically, so a `ConstantPosition` GCS would
have *silently ignored every update* it was sent. A node whose mobility model
can't accept updates now logs a warning rather than dropping them.

`--standalone` is the exception: no ROS, GCS pinned, UAVs on Gauss-Markov
mobility, so the channel can be exercised in CI or debugged without Gazebo.
Gauss-Markov is never used in ROS mode — both it and the feed write position, so
whichever fired last would win and the trajectory would be nondeterministic.

### 6.1 The loss decomposition (`PublishStats`)

Every `statsPeriod` (0.5 s default), for each of the 6 links, the scenario
computes and publishes:

```
   pathloss_rx = Tx − L_pathloss                        (log-distance alone)
   obstacle_dB = smoothed shadowing                     (from Gazebo)
   faded_rx    = Tx − L_pathloss − obstacle_dB + fade   (full chain, ONE draw)
   fading_dB   = faded_rx − (pathloss_rx − obstacle_dB)
   snr_dB      = faded_rx − noiseFloor                  (−94 dBm)
```

With `--csvPath` these go to a per-link CSV alongside `distance_m`, `blocked`,
`fading_m` and `known` — the file the validation harness checks (§9.1).

**This decomposition is arithmetically valid only because the chain contains
exactly one stochastic stage.** That is the direct reason
`ns3::NakagamiPropagationLossModel` is deliberately absent from the chain (§7.4)
— adding it would turn `fading_dB` into the sum of two independent Gamma
processes, matching no distribution the model implements, and every validation
plot built on that column would quietly become meaningless.

### 6.2 `CheckIntegration` — making a broken co-simulation *look* broken

At t≈10 s (≈100 messages at the publishers' 10 Hz) the scenario reports any node
that never received a position and any link that never received an obstacle
report, then says which of the handful of one-line misconfigurations to check.

It exists because **every way this integration breaks looks identical from the
outside**: the simulation runs happily and produces plausible numbers that are
quietly wrong. If node 0's position never arrives, NS-3 leaves the GCS at its
CLI default and treats its links as clear line-of-sight — which is exactly what
a correctly working unobstructed scenario looks like. Causes it catches: no
`<model name="gcs">` in the world, `<gcs_enabled>false</gcs_enabled>` in the
plugin block, `world_pos_publisher` started with `gcs_enabled:=false`, a
publisher still on the legacy numbering, or the ROS graph simply not connected.

### 6.3 Why FlowMonitor is not used

`FlowMonitor` cannot see this traffic. TapBridge in UseLocal mode bridges the
Linux namespace to the `WifiNetDevice` at **layer 2**, so frames never traverse
the `Ipv4L3Protocol` where FlowMonitor installs its probes. `InstallAll()` would
run happily and faithfully report **zero flows** — an empty result that reads as
"no traffic" rather than "wrong tool". It is replaced by direct **PHY trace
counters** (`PhyTxBegin`, `PhyRxEnd`, `PhyRxDrop` per node) plus an optional
per-packet SNR log from `MonitorSnifferRx` (`--snrLogFile`).

---

## 7. The custom propagation model: `DynamicObstacleLossModel`

Files: `dynamic_obstacle_loss_model.{hh,cc}`. This is where Gazebo's obstacle
number and NS-3's distance model combine into a final received power, plus
fading. It computes, for every transmission between drone `a` and drone `b`:

### 7.1 Log-distance path loss (the base, from the next model in the chain)

```
        PL(d) = PL(d0) + 10 · n · log10(d / d0)              (dB)
```

* `d`  = 3-D distance between the two nodes (m).
* `d0` = 1 m reference distance, `PL(d0) = 46.73 dB` — the energy already lost
  in the first metre. This is Friis at 802.11a **channel 36 (5180 MHz)**:
  `λ = c/5.18 GHz = 0.05788 m`, `FSPL(1 m) = 20·log10(4π/λ) = 46.73 dB`. (Two
  other values were in circulation: 47.3 dB, which corresponds to ~5.5 GHz and
  is not a channel in use here, and 46.67 dB, which was close but not derived
  from the actual centre frequency.)
* `n`  = **path-loss exponent = 2.0**. `n = 2` is free-space / air-to-air
  propagation. (It was deliberately lowered from an urban 2.7, because these are
  air-to-air links high above the ground, close to free space.)

Deterministic received power before obstacles/fading:
`P_r = P_t − PL(d)` with `P_t = 20 dBm`.

### 7.2 Smoothing the raw obstacle reports (EMA)

Gazebo's rays can flicker (a ray grazing a building edge flips 0 ↔ 15 dB between
frames). To stop the link from oscillating, each new report `L_k` is
exponentially smoothed:

```
        S_k = α · L_k + (1 − α) · S_{k−1}          α = 0.3 (EmaAlpha)
```

`S_k` is the *smoothed* obstacle loss actually applied. `α = 0.3` means each new
reading nudges the running value 30 % of the way — fast enough to track a drone
flying behind a wall, slow enough to reject single-frame spikes.

### 7.3 Line-of-sight state with hysteresis

Whether the link counts as blocked (NLoS) or clear (LoS) is a *sticky* decision
with two thresholds, so it cannot flap:

```
        if  (not blocked) and  S_k > 3 dB   →  blocked = true    (BlockThresholdDb)
        if  (blocked)     and  S_k < 1 dB   →  blocked = false   (ClearThresholdDb)
```

The 3 dB / 1 dB gap is the hysteresis band — the loss must climb past 3 dB to
declare a block but fall below 1 dB to clear it, so a value hovering around 2 dB
does not toggle every frame.

### 7.4 Line-of-sight-aware Nakagami-`m` fading

Real radio signals *fade* — they fluctuate as multipath reflections add and
cancel. This is modelled as **Nakagami-`m`** fading, where the received *power*
is a random draw from a Gamma distribution whose mean is the deterministic
power `P̄` and whose shape is `m`:

```
        P_faded  ~  Gamma(shape = m, scale = P̄ / m)

        mean      = m · (P̄/m) = P̄        (fading is, on average, unbiased)
        variance  = P̄² / m                (bigger m ⇒ steadier signal)
```

* **`m = MLos = 3.0`** when the link is clear (a strong dominant path,
  Rician-like, small fluctuations).
* **`m = MNlos = 1.0`** when blocked (`m = 1` is the **Rayleigh** case: no
  dominant path, deep random fades — the signal can momentarily vanish).

So a blocked link suffers *twice*: a constant `S_k` dB drop **and** a switch to
violent Rayleigh fading, exactly as a real obstructed link behaves. In code:

```
   powerW      = 10^((P_r_dBm − 30)/10)          // dBm → watts
   fadedPowerW = Gamma(m, powerW / m)            // draw a faded sample
   return 10·log10(fadedPowerW) + 30             // watts → dBm
```

#### Why `ns3::NakagamiPropagationLossModel` is deliberately *not* in the chain

The stock model would look like the obvious thing to use. Three reasons it is
wrong here, in increasing order of severity:

1. **It picks `m` by distance** (`m0` near / `m1` mid / `m2` far). That
   heuristic exists because stock NS-3 has no idea what lies between two nodes.
   We do — Gazebo ray-casts it. So the built-in model gets it backwards in both
   directions: a 120 m open-sky link is given Rayleigh fading, while a 30 m link
   straight through a wall is given "strong dominant path" fading.
2. **It would double-penalise blocked links.** Nakagami as implemented is
   mean-preserving in linear power, so it adds *variance*, not average
   attenuation — a lower `m` fattens the lower tail. A blocked link would then
   pay twice: once in mean power (Gazebo's dB penalty) and again in deep-fade
   probability (the Rayleigh regime). Packet delivery collapses harder than
   either model intends, and you cannot attribute the collapse to either.
3. **Two stochastic stages destroy the loss decomposition** that §6.1 and the
   whole validation suite rest on.

`DynamicObstacleLossModel` therefore does its own Nakagami draw internally, with
`m` switched by the ray-caster's LoS/NLoS state instead of by distance. It is a
real Nakagami channel — just correctly informed.

`ns3::RandomPropagationLossModel` was rejected as a "log-normal shadowing" layer
for related reasons: it duplicates what Gazebo *measures* with a *guess*, and
independently it draws a fresh variate on every call with no spatial or temporal
correlation. Shadowing is by definition slowly varying over metres of movement;
what that layer actually adds is per-packet white noise labelled "shadowing".

**Chain order note.** Fading is applied before path loss, which looks wrong but
is harmless: both stages are multiplicative in linear power, and multiplication
commutes.

### 7.5 Putting it together

For a transmission between drones `a` and `b`:

```
   P_final(dBm) = Fade_m(  P_t  −  S(a,b)  )  −  PL(d_ab)

   where  Fade_m(P) applies §7.4 with m chosen by the LoS/NLoS state of (a,b).
```

NS-3's Wi-Fi PHY then compares `P_final` (and the resulting SINR) against
`RxSensitivity = −82 dBm` at the fixed `OfdmRate6Mbps` to decide whether each
packet is received. That is the moment a modelled obstacle becomes a *dropped
packet* on the real Linux link.

> **Fixed defect (recorded for posterity):** `DoCalcRxPower` originally called
> `GetNext()->CalcRxPower()` *itself*, but the NS-3 base class already walks the
> chain — so log-distance loss was applied **twice** (~−142 dBm at 50 m instead
> of −61 dBm, far below sensitivity, i.e. every link dead). The fix: apply only
> this model's own effect and let the base class do the chaining. Verified: RSSI
> is now ≈ −60 dBm at 50 m and reacts correctly to obstacles.

---

## 8. Threading model — what runs in which thread, and why

There are two levels of concurrency: **threads inside the NS-3 process**, and
**separate OS processes** for the whole stack.

### 8.1 Inside the NS-3 process: two cooperating threads

| Thread | Runs | Job |
|---|---|---|
| **A — main / simulation** | `Simulator::Run()` | The real-time discrete-event loop: Wi-Fi events, the loss-chain math, TapBridge packet I/O, `ApplyFeed` at 50 Hz, and the 0.5 s `PublishStats`. **This thread owns all NS-3 state and is the only one that ever touches an ns-3 object.** |
| **B — rclcpp spin** | `rclcpp::spin(rosNode)` | Handles ROS 2 traffic: receives `/uav_world_positions` and `/link_obstacle_loss`, sends `/ns3_link_rssi` and `/ns3_link_snr`. Its callbacks write into two **plain** buffers (`std::map<uint32_t,Vector>` and a `vector<array<double,3>>`) under a mutex, and do nothing else. |

**Why split them?** `rclcpp::spin()` **blocks forever** waiting for messages. If
it ran on the main thread it would either block the simulation or force the
simulation to poll ROS, adding latency and jitter to a loop that must stay
locked to the wall clock.

### 8.1.1 The threading rule, and the crash that produced it

> **The rclcpp thread must never touch an ns-3 refcounted object — not even to
> copy one.**

`ns3::SimpleRefCount` uses a **plain `uint32_t`** counter: `Ref()`/`Unref()` are
`m_count++` / `m_count--`, not atomics. Copying a `Ptr<>` — or a `NodeContainer`,
which holds `Ptr<Node>` — on the ROS thread while the simulation thread copies
the same object races that counter. A single lost increment drops it to zero
early, the object is deleted while still in use, and the next `CalcRxPower()`
dereferences freed memory.

That is not hypothetical. The previous design used the pattern this document
used to recommend:

```
   OnPositions()  →  Simulator::ScheduleWithContext(...)  →  mob->SetPosition(...)
```

`ScheduleWithContext` is genuinely thread-safe for *scheduling*, but the lambdas
being scheduled captured `Ptr<DynamicObstacleLossModel>` and `NodeContainer`
**by value**, and those copies were constructed on the ROS thread. It **SIGSEGV'd
inside `PropagationLossModel::CalcRxPower` after ~34 s** of an all-links-blocked
run — 6 links × 10 Hz of obstacle reports being the highest Ptr-copy rate any
scenario produces. The same latent pattern is still present in the older
`three_uav_tapbridge_obstacle_loss.cc`, where a lower message rate merely makes
it rarer.

The current design removes the hazard by construction rather than narrowing the
race:

```
   THREAD B:  OnPositions()     →  g_pendingPos[id] = Vector(x,y,z)      (plain data)
              OnObstacleLoss()  →  g_pendingLoss.push_back({a,b,dB})     (plain data)

   THREAD A:  ApplyFeed() every 20 ms  →  swap both buffers under the mutex,
                                          then mob->SetPosition(...) and
                                          obstacleLoss->SetObstacleLoss(...)
```

`ApplyFeed` is an ordinary `Simulator` event, so **nothing is scheduled from the
ROS thread any more**. The buffers are swapped rather than copied, so the lock is
held only for the swap. Positions *coalesce* (a map keyed by id — only the newest
matters), while obstacle reports *queue* (a vector — the EMA must see every
sample). The 50 Hz drain rate gives 5× headroom over the 10 Hz publishers; run it
slower than the publishers and `g_pendingLoss` grows without bound.

A `std::mutex` in the loss model still guards the obstacle map, so reads from
`DoCalcRxPower` and writes from `SetObstacleLoss` are safe even though both now
happen on thread A.

### 8.2 Across the machine: one process per concern

Each major component is a **separate OS process** (often its own terminal):

| Process(es) | Threads of note | Benefit of isolation |
|---|---|---|
| `gzserver` (Gazebo) | physics thread + a detached rclcpp spin thread (in the plugin) | Heavy physics/raycasting cannot stall the network sim; can be pinned to its own CPU core. |
| `arducopter` × 3 | each is the full autopilot, real-time master | A crash or stall of one drone's firmware doesn't take down the others or the RF model. |
| `world_pos_publisher.py` | single rclcpp spin | Decouples "where are the drones" from both producers and consumers; trivially restartable. |
| NS-3 | threads A + B (§8.1) | The RF model is a clean, independently schedulable unit. |
| mission script | pymavlink I/O | The flight choreography is separate from the infrastructure. |

**Why processes, not one big program?** Three reasons: **fault isolation** (one
component crashing doesn't kill the rest), **independent real-time scheduling**
(each can be pinned to dedicated CPU cores — see §10), and **real-app-in-the-
loop** (the Linux kernel's real networking stack participates, which is only
possible when the pieces are genuinely separate processes joined by taps/ROS).

---

## 9. How to run the stack (manual, in order)

Dependency order: **TAP devices → Gazebo + SITL + relay → NS-3 → mission.**

```bash
# ── Terminal 1 — network namespaces + TAP devices (root, once per boot) ──
sudo bash ~/multi-uav-workspace/src/multi_uav_simulation/scripts/setup_netns_tap.sh

# ── Terminal 2 — Gazebo (plugin world + state plugin) + 3× SITL + relay ──
export ARDUPILOT_HOME=~/ardupilot            # if not already exported
bash ~/multi-uav-workspace/src/multi_uav_simulation/launch/launch_multi_uav_new.sh
#   → check /tmp/world_pos_publisher.log has NO "No /model_states" warning

# ── Terminal 3 — NS-3 RF channel (needs taps AND Gazebo/ROS up) ──
source /opt/ros/humble/setup.bash
taskset -c 2,3 \
  ~/ns-3.3/build/scratch/multi_uav_simulation/ns3.38-three_uav_tapbridge_integrated-default
#   the default tap names (tap-gcs, tap-uav1..3) and the default GCS position
#   (-24, 0, 0) already match setup_netns_tap.sh and small_city_base.world,
#   so no flags are needed. Read the t=10 s integration check (§6.2).

# ── Terminal 4 — the mission ──
source /opt/ros/humble/setup.bash
python3 ~/multi-uav-workspace/src/multi_uav_simulation/scripts/multi_drone_mission_new.py

# ── Optional: pin the spawned processes + set governor (see §10) ──
bash ~/multi-uav-workspace/src/multi_uav_simulation/scripts/test_scripts/pin_realtime.sh

# ── Optional: prove the data plane end-to-end ──
bash ~/multi-uav-workspace/src/multi_uav_simulation/scripts/test_scripts/iperf3_channel_test.sh

# ── Optional: record the live channel (passive; safe at any time) ──
python3 ~/multi-uav-workspace/src/multi_uav_simulation/scripts/test_scripts/record_live_links.py \
  --tag mission1
```

Why this order: NS-3 fails to attach if the taps don't exist yet; the relay
needs Gazebo's `/model_states` to exist; NS-3 should start after Gazebo so drone
positions flow in immediately.

### 9.1 Validating the model (`scripts/test_scripts/`)

Two of these tools are **drivers**, not observers: `verify_datapath.sh` and
`run_channel_validation.py` start their own NS-3 and publish their own positions
and obstacle reports. Run either while Gazebo is up and every node position will
alternate between the real drone and the script's fixed coordinates at 10 Hz,
silently corrupting both. **Stop Gazebo first.** Only `record_live_links.py` is
passive — it creates no publisher at all and is safe to start and stop mid-flight.

**Tier 0 — `verify_datapath.sh`.** Answers the one question everything else is
conditional on: *do packets between the namespaces actually traverse the
simulated channel?* The channel model can be perfectly implemented and still
touch no packet, in which case every throughput and loss number describes the
Linux bridge instead. Its load-bearing step is the **negative control**: with
NS-3 stopped, the ping must fail *completely*. If it succeeds, a bypass path
exists and no later measurement means anything, so the script aborts there
rather than reporting a misleading pass. It passed 15/15 on 2026-07-21 — the
first time traffic had ever provably crossed the channel; before that, every PHY
counter read zero. That run also confirmed `pathloss_rx` at 10 m = −46.730 dBm
against a prediction of −46.73, and mean SNR within 0.5σ of prediction.

**Channel maths — `run_channel_validation.py`.** Stands in for Gazebo and drives
NS-3 through four scenarios: `los_clear` (m=3 distribution), `nlos_blocked`
(m=1), `hysteresis` (clear→blocked→clear, the only test of `ClearThresholdDb`),
and `ros_healthy` (one blocked link of six). It recomputes the model
*independently* rather than reading values back, so the identity checks in §6.1
fail on a mismatch instead of agreeing with themselves.

> **A green suite is not a clean run.** The first version of this harness
> reported "0 FAILURES" for a scenario in which NS-3 died with SIGSEGV at t=34 s,
> because enough rows had been written before the crash for every per-link check
> to pass on the partial data. Process exit status is now check #1 per scenario.

Known gaps, worth stating plainly: the harness has never run against real
Gazebo, and it runs with `--enableTap=false`, so the PHY counters and the
`MonitorSnifferRx` log are exercised only by `verify_datapath.sh`. Geometry is
fixed per run — mobility-driven distance variation is not swept.

> **Note on what actually crosses the radio.** `multi_drone_mission_new.py`
> connects to SITL on `127.0.0.1:5760/5770/5780` — the *root* namespace, which
> **bypasses NS-3**. So with this mission, NS-3 models the link and reports RSSI
> *in parallel*, but the mission's own MAVLink does not travel through the
> simulated radio. Only traffic between the `uav*ns` namespaces (e.g. `iperf3`,
> or the micro-ROS/DDS variant in `launch_multi_dds.sh`) is subject to the RF
> model. Decide which you want to evaluate (see recommendation R6).

---

## 10. Real-time synchronization — the deep issue, and how it's handled

This stack is a **federation of independent real-time simulators with no shared
clock**, coupled only by ROS topics and the TAP bridge. That coupling is
"soft real-time": it is only valid while *every* simulator keeps up with the
wall clock. The pacing hierarchy actually in use:

| Component | Clock behaviour | Setting |
|---|---|---|
| ArduPilot SITL | **real-time master**, paces to wall clock | `--speedup 1` |
| Gazebo physics | free-runs, stepped on demand by SITL lockstep | `real_time_update_rate = -1` |
| NS-3 | **independently** locks its event queue to the wall clock | `RealtimeSimulatorImpl`, `SynchronizationMode = BestEffort` (default) |

Two independent real-time references (SITL's clock and NS-3's wall clock) that
nothing keeps aligned. The failure modes and their fixes are in §11.

---

## 11. Recommendations (what to do, and why *not* doing it is a problem)

### R1 — Keep the position feed (`world_pos_publisher.py`) running. *(done — keep it)*
**Why:** NS-3 only moves its nodes when `/uav_world_positions` arrives. Before
this node existed, nothing published that topic, so **NS-3's nodes stayed frozen
at their initial 50 m triangle while the drones actually flew** — the distance
path loss never changed. Obstacle loss still worked (the plugin reads Gazebo
poses directly), but the *distance* component was stuck. If this node dies mid-
run, you silently regress to that broken state. The launch script now starts it
automatically; keep it that way.

**A related failure was fixed on 2026-07-21.** Killing Gazebo did *not* stop the
relay: `self._latest` was never invalidated, so it rebroadcast the last frame it
ever saw at 10 Hz forever. Downstream that is indistinguishable from a fleet
hovering perfectly still — NS-3 kept computing path loss from frozen
coordinates, and the recorder kept logging a healthy feed age because the
*topic* was live. It silently corrupted a recorder test. The relay now goes
**silent** after `stale_after` (2 s ≈ 40 missed frames) and logs an error, which
is the right failure mode: `ApplyFeed` receives nothing, `CheckIntegration`
reports the missing nodes, and the recorder's `pos_age_s` starts climbing.

### R2 — Keep the offered network load under the real-time forwarding ceiling (~1.4 Mbit/s here).
**Why:** `RealtimeSimulatorImpl` must process every packet in wall-clock time.
Above ~1.4 Mbit/s on this host, NS-3 cannot keep up; in `BestEffort` mode it
**silently drops/compresses** and you see ~90 % loss *regardless of channel
state*. That masks the obstacle effect entirely and looks like a broken network.
Test with `iperf3 -b 500K`, not `-b 20M`. Not respecting this makes every
throughput/latency number a simulator artefact, not an RF result.

### R3 — Use `HardLimit` sync mode for measurement runs.
**Why:** in the default `BestEffort` mode NS-3 hides the fact that it fell
behind. Running with
`--ns3::RealtimeSimulatorImpl::SynchronizationMode=HardLimit
--ns3::RealtimeSimulatorImpl::HardLimit=+50ms` makes NS-3 **abort** when real-
time jitter exceeds the limit — so a run that *completes* is a run whose timing
you can trust. Keep `BestEffort` for demos, `HardLimit` for data you'll report.
Without this you can't tell a valid measurement from a lagged one.

### R4 — Pin CPUs and set the `performance` governor. *(applied)*
**Why:** both NS-3 and Gazebo are real-time and jitter-sensitive. If they share
cores, each steals cycles from the other, Gazebo's real-time factor dips below
1, positions arrive late, and NS-3 models stale geometry. On this hybrid CPU:
gzserver → P-cores `0,1`; NS-3 → P-core `2,3`; SITL/relay/ROS → E-cores `4–11`;
governor `performance` so the P-cores hit 5.4 GHz under load instead of ramping
lazily. Script: `scripts/test_scripts/pin_realtime.sh`. Skipping this is the
most common cause of the drift in R2/R3.

### R5 — Give NS-3 a clean shutdown path. *(still open)*
**Why:** the main thread is blocked in `Simulator::Run()` until `simTime`, which
defaults to 24 h — so **Ctrl-C does not stop it; you must
`kill -9 $(pgrep -f ns3.38-three_uav)`**. That's error-prone and can leave taps
in a half-attached state. The fix is to call `Simulator::Stop()` from a signal
handler, or to run with a finite `--simTime`. Not doing so risks orphaned
processes and taps that block the next run.

*Partly addressed:* the **teardown order** is now correct once `Run()` does
return — `rclcpp::shutdown()` makes `spin()` return, the ROS thread is joined,
and only then does `Simulator::Destroy()` run. Destroying ns-3 objects while the
ROS thread could still be unwinding would reintroduce the cross-thread refcount
hazard of §8.1 at exactly the worst moment.

### R6 — Decide whether mission traffic should cross the RF model.
**Why:** as noted in §9, `multi_drone_mission_new.py` talks to SITL directly and
**bypasses NS-3**. If your research goal is "how does the *network* affect the
mission", that goal is not currently being measured — the RF model is a passive
observer. To route real control/telemetry through the radio, put the endpoints
inside the namespaces (the micro-ROS/DDS `launch_multi_dds.sh` variant runs the
agents inside `gcsns` for exactly this). Leaving it as-is is fine for RF-model
studies but misleading if you claim end-to-end networked-control results.

### R7 — Seed the random-number streams for reproducibility. *(done)*
**Why:** the fading model draws from a `GammaRandomVariable`. The earlier
scenario never called `AssignStreams` or set `RngRun`, so every run used the
default stream and **results were not reproducible** — worse, two "identical"
runs drew the *same* fading sequence, which quietly understates variance in any
averaged result you put in a paper. The integrated scenario now calls
`obstacleLoss->AssignStreams(1000)` and exposes `--rngRun` (default 1). Vary
`--rngRun` across repetitions and hold it fixed to replay one exactly.

### R8 — Rate adaptation is deliberately switched off; report it as such.
**Why:** the scenario uses `ConstantRateWifiManager` at `OfdmRate6Mbps`, not
`IdealWifiManager`. Two reasons. Practically, `IdealWifiManager` needs
per-station SNR feedback and is known to trip assertions on 802.11a ad-hoc in
ns-3.36–3.38. More importantly, **rate adaptation creates a feedback loop
between the channel model and the throughput measurement**: the channel changes
the rate, the rate changes the throughput, and you can no longer attribute a
result to the channel. A fixed rate keeps the experiment clean, and 6 Mbps — the
most robust OFDM rate — sits comfortably above the ~1.4 Mbit/s real-time
application ceiling, so the PHY rate is never the bottleneck; the channel is.
The trade-off to state in any write-up: this models a radio that does *not*
adapt, so it will under-state the throughput a real adaptive link would achieve
on a good channel and over-state its robustness on a bad one.

---

## 12. Parameter quick-reference

| Parameter | Value | Where |
|---|---|---|
| Nodes | 4 — GCS (id 0) + 3 UAVs (ids 1–3), 6 links | NS-3 scenario |
| Wi-Fi standard | **802.11a**, ch 36 (5180 MHz), Adhoc | NS-3 scenario |
| Tx power | 20 dBm | NS-3 PHY |
| Rx sensitivity | −82 dBm | NS-3 PHY |
| Noise floor (for SNR) | −94 dBm = −174 + 10·log10(20 MHz) + 7 dB NF | NS-3 `PublishStats` |
| Rate manager | `ConstantRateWifiManager` @ `OfdmRate6Mbps` | NS-3 scenario |
| Path-loss exponent `n` | 2.0 | LogDistance model |
| Reference loss / distance | **46.73 dB @ 1 m** (Friis, 5180 MHz) | LogDistance model |
| `m` (LoS / NLoS) | 3.0 / 1.0 | DynamicObstacleLossModel (`MLos`/`MNlos`) |
| EMA `α` | 0.3 | `EmaAlpha` |
| Block / clear thresholds | 3 dB / 1 dB | `BlockThresholdDb`/`ClearThresholdDb` |
| Material loss `L_e` | glass 4, foliage 5, wood 8, vehicle 12, concrete 15, metal 20 dB; **untagged 0** | obstacle plugin |
| Bulk attenuation | 0.5 dB/m, `t` ≤ 20 m | obstacle plugin |
| Raycast rate | 10 Hz | obstacle plugin |
| State-plugin rate | 20 Hz | world file |
| Position relay rate | 10 Hz (silent after 2 s of no `/model_states`) | `world_pos_publisher.py` |
| Feed drain rate | 50 Hz | NS-3 `ApplyFeed` |
| RSSI / SNR publish rate | 2 Hz (0.5 s, `statsPeriod`) | NS-3 `PublishStats` |
| GCS antenna height | 2.9 m (0.9 m cabinet + 2.0 m mast) | world + `world_pos_publisher.py` |
| Subnet | 10.42.0.0/24 (gcs .10, uav1–3 .11–.13) | setup + NS-3 |
| Initial formation | UAV triangle, side = `distance` (50 m default) at `uavAltitude` (20 m); GCS at (−24, 0, 0) | NS-3 scenario |
| RNG | `AssignStreams(1000)`, `--rngRun` (default 1) | NS-3 scenario |

---

*Generated as a living description of the stack as it stands. Key files:
`ns3/three_uav_tapbridge_integrated.cc`,
`ns3/dynamic_obstacle_loss_model.{hh,cc}`,
`gazebo_plugins/src/obstacle_raycast_plugin.cc`,
`scripts/world_pos_publisher.py`,
`scripts/test_scripts/{verify_datapath.sh, run_channel_validation.py,
record_live_links.py, iperf3_channel_test.sh, pin_realtime.sh}`,
`launch/launch_multi_uav_new.sh`,
`worlds/small_city_base.world`, `worlds/multi_uav_plugin.world`.*

*The 3-node predecessor `ns3/three_uav_tapbridge_obstacle_loss.cc` is still in
the tree and still builds, but it has no GCS node, uses 802.11n, and carries the
cross-thread `Ptr` hazard described in §8.1. Nothing in this document describes
it; new work should use the integrated scenario.*

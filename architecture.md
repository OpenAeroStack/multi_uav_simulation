# Multi-UAV Simulation — System Architecture

**A hardware-in-the-loop-style co-simulation that couples a physics simulator
(Gazebo), autopilot firmware (ArduPilot SITL), and a network simulator (NS-3)
so that three drones fly in a 3-D world while a *realistic wireless link* — with
distance-based path loss, obstacle shadowing, and fading — is modelled between
them in real time.**

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
   │                       │   │ publisher.py     │   │   three_uav_tapbridge_obstacle_loss     │
   │ ┌───────────────────┐ │   │  (relay node)    │   │ ┌───────────────────────────────────┐  │
   │ │ physics engine    │ │   └──────────────────┘   │ │ THREAD B: rclcpp spin (ROS I/O)   │  │
   │ │ (ODE)             │ │                          │ │  OnPositions / OnObstacleLoss     │  │
   │ ├───────────────────┤ │                          │ ├───────────────────────────────────┤  │
   │ │ obstacle raycast  │─┼──────── /link_obstacle_loss│ THREAD A: Simulator::Run()        │  │
   │ │ plugin (.cc)      │ │                          │ │  Wi-Fi PHY + propagation-loss chain│  │
   │ ├───────────────────┤ │                          │ │  DynamicObstacleLossModel          │  │
   │ │ gazebo_ros_state  │─┼──── /gazebo/model_states  │ │  TapBridge  ⇄  Linux TAP devices  │  │
   │ │ plugin (.so)      │ │                          │ │  PublishStats ─▶ /ns3_link_rssi   │  │
   │ └───────────────────┘ │                          │ └────────────────────┬──────────────┘  │
   └───────────┬───────────┘                          └──────────────────────┼─────────────────┘
               │ Gazebo⇄ArduPilot lockstep                                    │  Layer-2 Ethernet
               │ (flight-dynamics model, UDP)                                 │  frames over TAP
               ▼                                                              ▼
   ┌───────────────────────┐                              ┌────────────────────────────────────────┐
   │  ArduCopter SITL × 3   │   MAVLink over TCP           │  Linux network namespaces (isolated     │
   │  (real autopilot fw)   │   127.0.0.1:5760/5770/5780   │  mini-networks) + veth + bridges + taps │
   │  sysid 1 / 2 / 3       │◀────────────────────────────▶│  uav1ns 10.42.0.11                       │
   └───────────────────────┘        │                     │  uav2ns 10.42.0.12                       │
               ▲                     │                     │  uav3ns 10.42.0.13   (+ gcsns .10)       │
               │ pymavlink           │                     └────────────────────────────────────────┘
               │                     │
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
| **NS-3 (3.38)** | Discrete-event network simulator | Models the 802.11n (Wi-Fi) radio link between the three drones at the packet level, applying path loss + obstacle loss + fading. Bridges *real* Linux packets into that simulated radio. |
| **`world_pos_publisher.py`** | Small ROS 2 relay node (added) | Reads ground-truth drone poses from Gazebo and republishes them in the flat array format NS-3 understands. |
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
| `/uav_world_positions` | `std_msgs/Float32MultiArray` | `world_pos_publisher.py` | **NS-3**, obstacle plugin | 10 Hz | `[id, x, y, z, id, x, y, z, …]` — one 4-tuple per drone, `id` 0-based |
| `/link_obstacle_loss` | `std_msgs/Float32MultiArray` | obstacle raycast plugin | **NS-3** | 10 Hz | `[i, j, loss_dB, …]` — one 3-tuple per drone *pair* |
| `/ns3_link_rssi` | `std_msgs/Float32MultiArray` | **NS-3** (`PublishStats`) | loggers / your tools | 2 Hz | `[a, b, rx_dBm, …]` — received power per drone pair |
| `/marker` | `ignition.msgs.Marker` | obstacle plugin | gzclient (viewer) | 10 Hz | line per pair, green = clear, red = blocked (visual only) |

**Why a flat float array and not a structured message?** Because it crosses a
C++/Python boundary and a Gazebo/NS-3 boundary; a primitive array has zero
schema-compatibility risk. The cost is that the index convention is implicit —
if one side reads 3-tuples where the other wrote 4-tuples, it silently breaks.
That is exactly why §3 spells out each layout.

**Identity convention (critical):** drone **id `k`** (0-based) is:
* NS-3 node number `k` (nodes are created `0,1,2`),
* Gazebo model **`iris_{k+1}`** (i.e. id 0 → `iris_1`, matched by name prefix so
  `iris_1_demo` also matches),
* Linux namespace **`uav{k+1}ns`** with IP **`10.42.0.1{k+1}`**.

Keeping these three numbering schemes aligned is what lets an obstacle the
raycaster reports for pair `(0,1)` end up attenuating the *right* Wi-Fi link and
the *right* Linux traffic.

---

## 4. The network plumbing (how real packets reach the simulated radio)

NS-3 does not just *report* a link quality — it can carry **real Linux
packets** through the simulated Wi-Fi. This is what makes tools like `iperf3`,
`ping`, or a real DDS stack testable against the RF model.

```
   uav1ns (a private network stack)                       NS-3 process
   ┌───────────────────────────┐                    ┌────────────────────────┐
   │ app (iperf3, DDS, …)       │                    │  node 0  ── Wi-Fi PHY ──┼───┐
   │ veth1n  10.42.0.11/24      │                    │            (802.11n)    │   │
   └──────────┬────────────────┘                    └────────────────────────┘   │
              │ veth pair                                     ▲                    │  simulated
        ┌─────┴─────┐        ┌──────────┐   TAP (a virtual     │ TapBridge          │  Wi-Fi
        │  br-uav1  │────────│ tap-uav1 │   NIC the kernel      │ "UseLocal" mode    │  channel
        │  (bridge) │        └──────────┘   and NS-3 share)    │                    │  (Yans)
        └───────────┘                                          ▼                    │
                                                     ┌────────────────────────┐    │
   uav2ns 10.42.0.12  ── br-uav2 ── tap-uav2 ────────┤  node 1  ── Wi-Fi PHY ──┼───┤
   uav3ns 10.42.0.13  ── br-uav3 ── tap-uav3 ────────┤  node 2  ── Wi-Fi PHY ──┼───┘
                                                     └────────────────────────┘
```

* **Network namespace (`netns`)** — a completely isolated copy of the Linux
  network stack (its own interfaces, routes, ARP table). `uav1ns` cannot see
  `uav2ns` except through the wire you give it.
* **veth pair** — a virtual Ethernet cable: two ends, one inside the namespace
  (`veth1n`, holds the drone's IP `10.42.0.11`), one in the host (`veth1h`).
* **bridge (`br-uav1`)** — a virtual switch joining the veth end and the tap.
* **TAP device (`tap-uav1`)** — a virtual network card whose "wire" is a file
  descriptor. The Linux kernel writes frames into it; **NS-3's TapBridge reads
  them out** and injects them into NS-3 node 0, and vice-versa.
* **TapBridge "UseLocal" mode** — NS-3 acts as a transparent Layer-2 bridge for
  the single MAC address behind the tap. The taps are created *owned by your
  user* (`ip tuntap … user`) so NS-3 can attach without running as root.

**Net effect:** the three namespaces behave as if they were three real laptops
on one Wi-Fi ad-hoc network — and every frame between them is subject to the
path loss, obstacle loss, and fading NS-3 computes. Send 500 kbit/s of UDP from
`10.42.0.11` to `10.42.0.12` and it flows node0→(radio)→node1; block that link
with an obstacle and the traffic stops. (Proven with `iperf3` — see
`scripts/iperf3_channel_test.sh`.)

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

For each drone **pair** `(i, j)`:

1. **Get both endpoints.** Prefer positions from `/uav_world_positions`; if that
   topic is silent, fall back to reading the model's true world pose directly.
   (This fallback is why obstacle detection kept working even before we added
   the position publisher.)
2. **Cast a ray** from drone `i` toward drone `j` (`CastRay`). Gazebo's ray
   reports the **first** solid it hits. But the first hit is often the ground
   plane or another drone's body — not a real obstacle — so the plugin **steps
   past filtered hits and re-casts** (up to 5 hops), skipping anything named
   `ground_plane` or matching the `iris_` prefix. It also ignores hits *beyond*
   the destination drone (`dist > link_len − 0.5`).
3. If a genuine obstacle is hit, **measure how much material the signal crosses**
   (`ObstacleThickness`): cast a *second* ray backwards from `j` to `i` to find
   the obstacle's far (exit) face. The gap between entry and exit faces along
   the link is the true material thickness `t`.
4. **Convert to a dB loss** (`ComputeObstacleLoss`) and publish
   `[i, j, loss_dB]` on `/link_obstacle_loss`. Also draws a green/red marker
   line in the viewer.

### The obstacle-loss equation

```
        L_obs = L_e + 0.5 · t          (dB)
```

* **`L_e`** — a fixed *entry/exit penetration loss* that depends on the wall
  material (matched by substring in the model name):

  | Material (name contains) | `L_e` (dB) |
  |---|---|
  | `glass` | 4 |
  | `wood` | 8 |
  | `concrete` (default) | 15 |
  | `metal` | 20 |

* **`0.5 · t`** — a *bulk attenuation* term: 0.5 dB for every metre of material
  the ray passes through, with `t` clamped to ≤ 20 m. A drone just behind a wall
  loses less than one deep inside a thick structure — which is the physically
  correct direction. (An earlier version wrongly used the *free-space* distance
  from the wall to the drone, penalising far drones more; that was corrected.)

If the exit face can't be resolved, the plugin falls back to `L_obs = L_e`
alone. `L_obs = 0` means clear line-of-sight (no real obstacle on the ray).

---

## 6. The NS-3 scenario: `three_uav_tapbridge_obstacle_loss.cc`

This is the network-side program. It builds a 3-node Wi-Fi network, wires it to
the Linux taps, embeds a ROS 2 node, and applies the custom loss model. Walking
through what it constructs:

1. **Real-time engine.** `SimulatorImplementationType = RealtimeSimulatorImpl`.
   Normally NS-3 runs *as fast as possible* in virtual time; here it is locked
   to the wall clock so it can stay in step with Gazebo and the real packet
   flow. (Consequences discussed in §10.)
2. **Three nodes** (`nodes.Create(3)`) — one per drone.
3. **The propagation-loss chain** — the heart of the RF realism, a linked list
   of models each drone-to-drone transmission passes through:
   ```
       DynamicObstacleLossModel  ──▶  LogDistancePropagationLossModel
       (obstacle shadowing +           (distance-based path loss)
        line-of-sight-aware fading)
   ```
   plus a `ConstantSpeedPropagationDelayModel` for propagation delay. The chain
   is attached to a `YansWifiChannel`.
4. **Wi-Fi PHY/MAC** — 802.11n at 5 GHz, `TxPower = 20 dBm`,
   `RxSensitivity = −82 dBm`, `AdhocWifiMac` (peer-to-peer, no access point),
   `IdealWifiManager` (picks the modulation rate from the measured SNR).
5. **Internet stack + IPs** on the `10.42.0.0/24` subnet (largely vestigial in
   UseLocal TapBridge mode, where the Linux side's addressing is what counts).
6. **Mobility** — each node has a `ConstantVelocityMobilityModel`; its position
   is updated whenever `/uav_world_positions` arrives. Initial layout is a
   triangle with side `distance` (default 50 m).
7. **TapBridge install** — attaches node `k` to `tap-uav{k+1}` (see §4).
8. **Embedded ROS 2 node** (`Ns3RosNode`) — subscribes to
   `/uav_world_positions` and `/link_obstacle_loss`, publishes `/ns3_link_rssi`.
9. **`PublishStats`** — every 0.5 s, for each pair it calls the loss chain to
   compute current received power and publishes it on `/ns3_link_rssi`.

### How an incoming ROS message becomes drone motion / link loss

The ROS callbacks run on a *different thread* from the simulation (see §8), so
they never touch NS-3 state directly. Instead they **schedule** the work onto
the simulation thread:

```
   OnPositions()      →  Simulator::ScheduleWithContext(...)  →  mob->SetPosition(x,y,z)
   OnObstacleLoss()   →  Simulator::ScheduleWithContext(...)  →  obstacleLoss->SetObstacleLoss(i,j,dB)
```

`ScheduleWithContext` is the thread-safe hand-off documented for
`RealtimeSimulatorImpl`; it puts an event on the sim's queue to be executed on
the sim thread. This is the single most important pattern in the file.

---

## 7. The custom propagation model: `DynamicObstacleLossModel`

Files: `dynamic_obstacle_loss_model.{hh,cc}`. This is where Gazebo's obstacle
number and NS-3's distance model combine into a final received power, plus
fading. It computes, for every transmission between drone `a` and drone `b`:

### 7.1 Log-distance path loss (the base, from the next model in the chain)

```
        PL(d) = PL(d0) + 10 · n · log10(d / d0)              (dB)
```

* `d`  = 3-D distance between the two drones (m).
* `d0` = 1 m reference distance, `PL(d0) = 46.67 dB` (this is the free-space
  loss at 1 m for 5 GHz — the energy already lost in the first metre).
* `n`  = **path-loss exponent = 2.0**. `n = 2` is free-space / air-to-air
  propagation. (It was deliberately lowered from an urban 2.7, because these are
  UAV-to-UAV links high above the ground, close to free space.)

Deterministic received power before obstacles/fading:
`P_r = P_t − PL(d)` with `P_t = 20 dBm`.

### 7.2 Smoothing the raw obstacle reports (EMA)

Gazebo's rays can flicker (a ray grazing a wall edge flips 0 ↔ 15 dB between
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

### 7.5 Putting it together

For a transmission between drones `a` and `b`:

```
   P_final(dBm) = Fade_m(  P_t  −  S(a,b)  )  −  PL(d_ab)

   where  Fade_m(P) applies §7.4 with m chosen by the LoS/NLoS state of (a,b).
```

NS-3's Wi-Fi PHY then compares `P_final` (and the resulting SINR) against
`RxSensitivity = −82 dBm` and the rate chosen by `IdealWifiManager` to decide
whether each packet is received. That is the moment a modelled obstacle becomes
a *dropped packet* on the real Linux link.

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
| **A — main / simulation** | `Simulator::Run()` | The real-time discrete-event loop: Wi-Fi events, the loss-chain math, TapBridge packet I/O, and the 0.5 s `PublishStats`. This thread owns all NS-3 state. |
| **B — rclcpp spin** | `rclcpp::spin(rosNode)` | Handles ROS 2 traffic: receives `/uav_world_positions` and `/link_obstacle_loss`, sends `/ns3_link_rssi`. It never mutates NS-3 state directly — it *schedules* the change onto thread A via `ScheduleWithContext`. |

**Why split them?** `rclcpp::spin()` **blocks forever** waiting for messages. If
it ran on the main thread it would either block the simulation or force the
simulation to poll ROS, adding latency and jitter to a loop that must stay
locked to the wall clock. Giving ROS its own thread means:

* the real-time sim loop is never stalled by network I/O or DDS discovery;
* messages are picked up the instant they arrive, then handed to the sim safely;
* the design mirrors the classic "listener thread + thread-safe schedule"
  pattern (it replaced an older ZeroMQ listener thread with the same shape).

A `std::mutex` in the loss model guards the obstacle map, so even though the
write (`SetObstacleLoss`) and read (`DoCalcRxPower`) are marshalled onto thread
A, the code is safe by construction.

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
  ~/ns-3.3/build/scratch/multi_uav_simulation/ns3.38-three_uav_tapbridge_obstacle_loss-default \
  --tapBase=tap-uav

# ── Terminal 4 — the mission ──
source /opt/ros/humble/setup.bash
python3 ~/multi-uav-workspace/src/multi_uav_simulation/scripts/multi_drone_mission_new.py

# ── Optional: pin the spawned processes + set governor (see §10) ──
bash ~/multi-uav-workspace/src/multi_uav_simulation/scripts/pin_realtime.sh

# ── Optional: prove the data plane end-to-end ──
bash ~/multi-uav-workspace/src/multi_uav_simulation/scripts/iperf3_channel_test.sh
```

Why this order: NS-3 fails to attach if the taps don't exist yet; the relay
needs Gazebo's `/model_states` to exist; NS-3 should start after Gazebo so drone
positions flow in immediately.

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
lazily. Script: `scripts/pin_realtime.sh`. Skipping this is the most common
cause of the drift in R2/R3.

### R5 — Give NS-3 a clean shutdown path.
**Why:** the process installs a ROS SIGINT/SIGTERM handler, but the main thread
is blocked in `Simulator::Run()` until `simTime` (default 24 h) — so **Ctrl-C
does not stop it; you must `kill -9`**. That's error-prone and can leave taps in
a half-attached state. Recommend calling `Simulator::Stop()` from the signal
handler (or running with a finite `--simTime`) so the sim loop exits cleanly and
`Simulator::Destroy()` runs. Not fixing this risks orphaned processes and taps
that block the next run.

### R6 — Decide whether mission traffic should cross the RF model.
**Why:** as noted in §9, `multi_drone_mission_new.py` talks to SITL directly and
**bypasses NS-3**. If your research goal is "how does the *network* affect the
mission", that goal is not currently being measured — the RF model is a passive
observer. To route real control/telemetry through the radio, put the endpoints
inside the namespaces (the micro-ROS/DDS `launch_multi_dds.sh` variant runs the
agents inside `gcsns` for exactly this). Leaving it as-is is fine for RF-model
studies but misleading if you claim end-to-end networked-control results.

### R7 — Seed the random-number streams for reproducibility.
**Why:** the fading model has a `GammaRandomVariable`, but the scenario never
calls `AssignStreams`/sets `RngRun`, so every run uses the default stream and
**results are not reproducible** — RSSI samples and drop patterns differ each
time. For publishable/comparable results, set `RngSeed`/`RngRun` (and assign
streams) so a scenario replays identically. Without it you cannot compare two
configurations fairly, because run-to-run noise is uncontrolled.

### R8 — Be aware `IdealWifiManager` is optimistic.
**Why:** `IdealWifiManager` picks the modulation rate as if it knew the SNR
perfectly. Combined with per-packet Rayleigh fading, rate selection can jump
around and, on average, it over-estimates achievable rate versus a real adaptive
algorithm. If you need realistic rate adaptation, switch to `MinstrelHtWifi
Manager`. Not changing it is acceptable, but report it — otherwise throughput
looks better than a real radio would deliver.

---

## 12. Parameter quick-reference

| Parameter | Value | Where |
|---|---|---|
| Wi-Fi standard | 802.11n, 5 GHz, Adhoc | NS-3 scenario |
| Tx power | 20 dBm | NS-3 PHY |
| Rx sensitivity | −82 dBm | NS-3 PHY |
| Rate manager | `IdealWifiManager` | NS-3 scenario |
| Path-loss exponent `n` | 2.0 | LogDistance model |
| Reference loss / distance | 46.67 dB @ 1 m | LogDistance model |
| `m` (LoS / NLoS) | 3.0 / 1.0 | DynamicObstacleLossModel (`MLos`/`MNlos`) |
| EMA `α` | 0.3 | `EmaAlpha` |
| Block / clear thresholds | 3 dB / 1 dB | `BlockThresholdDb`/`ClearThresholdDb` |
| Material loss `L_e` | glass 4, wood 8, concrete 15, metal 20 dB | obstacle plugin |
| Bulk attenuation | 0.5 dB/m, `t` ≤ 20 m | obstacle plugin |
| Raycast rate | 10 Hz | obstacle plugin |
| State-plugin rate | 20 Hz | world file |
| Position relay rate | 10 Hz | `world_pos_publisher.py` |
| RSSI publish rate | 2 Hz (0.5 s) | NS-3 `PublishStats` |
| Subnet | 10.42.0.0/24 (gcs .10, uav1–3 .11–.13) | setup + NS-3 |
| Initial formation | triangle, side = `distance` (50 m default) | NS-3 scenario |

---

*Generated as a living description of the stack as it stands. Key files:
`ns3/three_uav_tapbridge_obstacle_loss.cc`,
`ns3/dynamic_obstacle_loss_model.{hh,cc}`,
`gazebo_plugins/src/obstacle_raycast_plugin.cc`,
`scripts/world_pos_publisher.py`, `scripts/pin_realtime.sh`,
`scripts/iperf3_channel_test.sh`, `launch/launch_multi_uav_new.sh`,
`worlds/multi_uav_plugin.world`.*

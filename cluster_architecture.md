# Cluster Architecture — Dynamic Cluster-Head Election with a Ground Station

**Design specification (v1).** Extends the existing multi-UAV / NS-3 obstacle-loss
stack with a ground station (GCS), a two-tier clustered topology, distributed
cluster-head (CH) election driven by *backhaul link quality to the GCS*, and
L3 MANET routing so member traffic is relayed through the CH to the GCS.

This is a **design document written to be defended**. Every mechanism states its
purpose, its formal rule, and the evaluator question it answers. Read
[`architecture.md`](architecture.md) first for the base stack.

> **Status:** specification only — no code yet. §10 is the phased implementation
> roadmap. §9 is the evaluator Q&A.

---

## 1. Requirements & scope

1. Add a **ground station** to the current 3-drone stack.
2. Drones form a **cluster**; one is the **cluster head (CH)**.
3. The **CH holds the backhaul** link to the GCS. Members reach the GCS **only by
   relaying through the CH** (two-tier).
4. The CH maintains a **separate link/flow to each member** (star, CH at centre).
5. The CH is elected **dynamically**: the drone with the **best current link to
   the GCS** becomes CH.
6. Election is **distributed** (no central controller); routing is a **real L3
   MANET protocol** (OLSR, with AODV as an alternative).

### 1.1 Design decisions (chosen)
| Decision | Choice | Consequence |
|---|---|---|
| Where election runs | **Distributed** among drones | No single point of failure; needs convergence + no-flap + split-brain arguments (§4) |
| Data-plane routing | **L3 OLSR** in the Linux namespaces | Real routing tables over the NS-3-modeled RF; CH advertised as gateway via OLSR **HNA** (§5) |
| Where routing/election live | **In the namespaces**, not in NS-3 | Preserves the existing L2-TapBridge model: NS-3 stays the lossy wireless channel; Linux owns IP/routing/apps |

### 1.2 Grounding assumption (physically motivated — state this up front)
The GCS is **distant and/or obstructed**, so at any instant only the
best-positioned drone has a **viable** backhaul (SINR above a usable threshold);
the others cannot reliably reach the GCS directly. Relaying through the CH is
therefore a **physical necessity**, not a cosmetic overlay. This is exactly why
"the drone with the best GCS connection" is the correct CH criterion. Where the
model still allows a weak direct member→GCS path, L3 gateway routing (§5)
administratively forces traffic through the CH.

---

## 2. Topology & reference model

```
                    GROUND STATION  (gcsns)         external net 10.99.0.0/24
                    host 10.99.0.1 (static, distant/obstructed)
                              ▲
              tier-2 backhaul │  drone↔GCS RF link (only CH has a viable one)
                              │
                       ┌───── CH ─────┐   gateway: advertises route to 10.99.0.0/24
          tier-1 relay │      │       │   tier-1 relay      MANET 10.42.0.0/24
                       ▼      ▼       ▼
                    memberA memberB  memberC
                    10.42.0.11  .12    .13     (members: default route to GCS via CH)
```

- **Tier-2 (backhaul):** one link, CH↔GCS. Its quality drives election.
- **Tier-1 (intra-cluster):** CH↔member links as **separate UDP flows** over the
  shared 802.11n channel (single radio ⇒ half-duplex relay — §9 Q7).
- **Addressing:** MANET = `10.42.0.0/24` (drones `.11/.12/.13`, as today). GCS on a
  **separate external subnet** `10.99.0.0/24` reachable **only via the elected CH
  (the gateway)**. This is the classic MANET-Internet gateway pattern and is what
  forces member→CH→GCS at L3.

---

## 3. Component architecture (MAPE-K control loop)

The system is a closed **Monitor → Analyze → Plan → Execute** loop over shared
**Knowledge**. Each stage is a separate, independently testable module. Nothing
new goes into NS-3 except one extra node and one extra metric.

```
 MONITOR (per drone, local)     ANALYZE (per drone)      PLAN (distributed)        EXECUTE (per drone)
 ───────────────────────────    ────────────────────     ──────────────────        ────────────────────
 own SINR to GCS (γ_i)       ─▶ smoothed quality q_i  ─▶ election + hysteresis  ─▶ olsrd HNA on/off (gateway)
 neighbour adverts (q_j,…)      + neighbour table        + epoch/split-brain        + role = CH / MEMBER
                                                         (§4 protocol)              members: default route via CH
```

| Module | Runs in | Responsibility |
|---|---|---|
| NS-3 scenario (extended) | NS-3 process | Add **4th node** = GCS, bridged to existing `tap-gcs`. Still a pure L2 bridge. Export **drone↔GCS** RSSI/SINR on `/ns3_link_rssi`. |
| Obstacle plugin + position feed | Gazebo | Feed the **GCS position** so buildings between GCS and a drone attenuate the backhaul. GCS is static. |
| `olsrd` | each namespace (uav1/2/3ns, gcsns) | Real OLSR MANET routing over the NS-3 channel; disseminates gateway (HNA) routes. |
| **`ch_agent`** (new) | each drone namespace | Distributed election: ingest own q_i + neighbour adverts, run the state machine (§4), toggle its local olsrd gateway advertisement. |
| Traffic + logger | namespaces / host | Member→GCS flows; records CH timeline, routes, PDR, handover loss. |

---

## 4. Distributed cluster-head election protocol (the core)

### 4.1 Backhaul quality metric
Each drone *i* observes its **own** link to the GCS (locally — from GCS beacons /
its own `/ns3_link_rssi` entry) as SINR `γ_i[k]` (dB). Raw SINR fades, so smooth
it with the same EMA the loss model already uses:

```
   q_i[k] = α · γ_i[k] + (1 − α) · q_i[k−1]          α ∈ (0,1], default 0.3
```

Optionally map to achievable backhaul rate `C_i = B·log₂(1+10^{q_i/10})` and elect
on `C_i` — more defensible than raw dBm (§9 Q3). `q_i` is **locally scoped**: a
drone knows its own quality and learns others' only through their advertisements
⇒ genuinely distributed.

### 4.2 Advertisement messages
Every `T_adv` (default 0.5 s) each drone broadcasts to its 1-hop cluster:

```
   ADVERT { id, q_i, role∈{MEMBER,CH}, ch_id, epoch, t }
```

These ride the always-on ad-hoc broadcast (they must survive the same lossy
channel they control — realistic). A node keeps a neighbour table of the latest
ADVERT per id, aged out after `T_timeout` (default 3·T_adv).

### 4.3 Election rule (per node, per round)
Let `N = {i} ∪ {live neighbours}`. Candidate `best = argmax_{j∈N} q_j`, ties broken
by **lowest id** (deterministic). Apply **hysteresis** against the current CH `c`:

```
   if best == c:                         keep c
   elif (q_best − q_c ≥ Δ) sustained ≥ T_dwell:   CH ← best ; epoch ← epoch+1
   else:                                 keep c            # inside the margin → no change
   role(i) = CH if CH==i else MEMBER (associate to CH)
```

- **Δ** (margin, default 3 dB / 20 % rate) and **T_dwell** (default 2 s) are the
  anti-flap guarantees (§9 Q1).
- **q_min** viability floor: if no drone has `q ≥ q_min`, cluster has *no* backhaul
  (report "GCS unreachable" rather than elect a useless CH).

### 4.4 Role state machine
```
        ┌─────────┐  best==self & margin/dwell met      ┌──────────────┐
        │ MEMBER  │ ──────────────────────────────────▶ │ CLUSTER_HEAD │
        │(assoc CH)│ ◀────────────────────────────────── │ (gateway on) │
        └─────────┘  another node wins by ≥Δ for T_dwell └──────────────┘
             ▲   │ CH timeout (no ADVERT for T_timeout)          │
             │   └───────────────▶ re-election (self-candidate)  │
             └───────────────────────────────────────────────────┘
   Transient HANDOVER_IN/OUT wrap the switch for make-before-break (§5.3).
```

### 4.5 Consistency guarantees (what evaluators probe)
- **Convergence:** 1-hop cluster ⇒ after one ADVERT round every node sees all
  `q_j`; distributed argmax with a common tie-break converges to the **global**
  best in O(1) rounds when the gap exceeds Δ. (Diameter-bounded in general.)
- **No-flap:** a switch requires a Δ-margin sustained for T_dwell ⇒ within any
  window where `|q_j − q_c| < Δ` there is **zero** switching. Under a stationary
  channel with a unique argmax whose gap > Δ: exactly one switch, then stable.
- **Split-brain:** if two nodes claim CH, ADVERT `epoch` + deterministic
  `(q, id)` tie-break resolves to one within bounded rounds; the loser demotes and
  adopts the higher epoch.
- **Partition / CH loss:** CH silence for `T_timeout` triggers re-election; an
  isolated drone self-elects (degenerate 1-node cluster) and recovers on rejoin.

---

## 5. L3 routing integration (OLSR + gateway handover)

### 5.1 Why OLSR (proactive) here
- **HNA (Host & Network Association)** lets a node advertise reachability to an
  **external network** — exactly the CH-as-gateway-to-GCS role.
- **Proactive** ⇒ routes already converged ⇒ **fast handover** (no on-demand
  route discovery latency). AODV (reactive) is the documented alternative but
  pays route-setup latency on every CH change (§9 Q6).

### 5.2 Coupling election → routing
The **only** thing the CH election changes at L3 is the gateway advertisement:

```
   role == CH      →  olsrd advertises  HNA(10.99.0.0/24)   ( "I can reach the GCS" )
   role == MEMBER  →  olsrd withdraws that HNA
```

Members' olsrd receive the HNA and install a route to `10.99.0.0/24` **via the
current CH**. The CH forwards (and NATs if required) between the MANET and the
GCS. Because the GCS lives on a separate subnet **only advertised by the CH**,
member traffic to the GCS is structurally forced through the CH — even on a
shared medium.

### 5.3 Handover: make-before-break
On decision `c → j`:
1. `j` enters HANDOVER_IN, **starts** advertising HNA (both c and j briefly
   advertise).
2. OLSR reconverges; members' routes to `10.99/24` shift to `j` (shorter/served).
3. `c` enters HANDOVER_OUT, **withdraws** HNA after a hold time, demotes to MEMBER.

Overlap ⇒ minimal loss. **Measured:** convergence time (decision → all members
routed via j) and packets lost in the window.

---

## 6. NS-3 & stack integration (what actually changes)

| Change | File / place | Note |
|---|---|---|
| **4th node = GCS** | `three_uav_tapbridge_obstacle_loss.cc` | `nodes.Create(4)`; bridge node 3 to the **existing** `tap-gcs` (already made by `setup_netns_tap.sh`). Still `UseLocal` L2 bridge. |
| GCS position | position feed + obstacle plugin | GCS static (e.g. field edge). Add it to `/uav_world_positions` (id 3) so the raycaster attenuates drone↔GCS links through buildings. |
| Export GCS-link metric | `PublishStats` | Already loops all pairs → GCS links appear automatically once node 3 exists. Optionally publish **per-node** GCS SINR so each `ch_agent` consumes only its own (keeps it distributed). |
| MANET routing | namespaces | Install `olsrd` in uav1/2/3ns (+gcsns as plain host on 10.99). NS-3 unchanged as the channel. |
| `ch_agent` | new, per drone namespace | §4 protocol; toggles olsrd HNA. |
| Addressing | `setup_netns_tap.sh` | Add GCS external subnet `10.99.0.0/24`; CH does forwarding/NAT MANET↔GCS. |

**Key point:** NS-3 stays a pure L2 lossy bridge. All routing/clustering is real
Linux networking over that bridge — consistent with the existing design and
maximally realistic.

---

## 7. Data plane & addressing plan

| Entity | MANET addr (10.42.0.0/24) | External (10.99.0.0/24) | Role |
|---|---|---|---|
| drone 1 | 10.42.0.11 | — | member/CH |
| drone 2 | 10.42.0.12 | — | member/CH |
| drone 3 | 10.42.0.13 | — | member/CH |
| CH (whichever) | its .1x | gateway to 10.99/24 (HNA) | relay |
| GCS | — | 10.99.0.1 | endpoint (no OLSR on MANET) |

Traffic: `member(10.42.0.1x) → CH(10.42.0.1y) → GCS(10.99.0.1)`. Return path
symmetric via the CH's advertised gateway route.

---

## 8. Evaluation plan (design it in now, not later)

**Baselines to beat** (same seeds, same channel):
1. **Static CH** — a fixed drone is always CH.
2. **Random CH** — periodic random rotation.
3. **Geometric** — drone physically nearest the GCS (ignores obstacles/fading).
4. **Proposed** — SINR-driven distributed election with hysteresis.

**Metrics:**
| Metric | Definition | Why |
|---|---|---|
| CH-selection accuracy | fraction of time `CH == argmax_i q_i` (ground truth from NS-3) | correctness of the decision |
| Flap rate | CH changes / minute | stability (sweep vs Δ, T_dwell) |
| Handover convergence | decision → all members routed via new CH | routing agility |
| Handover loss | packets dropped during the switch | make-before-break quality |
| E2E to GCS | PDR, latency, throughput per member | does clustering actually deliver |
| Relay penalty | medium utilisation / throughput vs single-hop | honesty about half-duplex relay |
| Robustness | reselection time after a building blocks CH↔GCS | reacts to the channel |

**Reproducibility:** fix NS-3 `RngRun`/seeds (the gap flagged in `architecture.md`
R7), run **N seeds**, report **mean ± 95 % CI**. Log CH timeline, routing tables,
per-flow counters.

**Sensitivity study:** sweep `α, Δ, T_dwell, q_min` → show the flap/latency
trade-off and pick an operating point with justification.

---

## 9. Defensive Q&A (anticipated evaluator questions)

**Q1. What stops the CH oscillating?** Hysteresis: a challenger must beat the
incumbent by ≥ Δ sustained for ≥ T_dwell. Inside the margin there is provably zero
switching (§4.5). We report flap rate vs Δ.

**Q2. Distributed — so no single point of failure?** Correct: every drone runs
the same election; if the CH dies, `T_timeout` triggers re-election. There is no
central controller.

**Q3. How is "best connection" defined rigorously?** Smoothed SINR `q_i`
(optionally mapped to Shannon/MCS capacity), locally observed — not raw
instantaneous dBm. Sensitivity to α reported.

**Q4. Is the clustering real or cosmetic?** Real OLSR routing tables + real
traffic; we show member→CH→GCS routes and 2-hop PDR/throughput, and that
withdrawing the CH's HNA actually removes the route.

**Q5. Split-brain / two CHs?** ADVERT `epoch` + deterministic `(q,id)` tie-break;
resolves to one CH in bounded rounds (§4.5).

**Q6. Why OLSR not AODV?** Proactive routes + built-in HNA gateway advertisement
⇒ fast handover and a natural gateway abstraction. AODV is provided as an
alternative and its route-setup latency is measured for comparison.

**Q7. Single radio, shared medium — is relaying realistic?** Yes: single-radio
half-duplex relay roughly halves relayed throughput; we **report** that penalty
rather than hide it, and note multi-radio/directional backhaul as an extension.

**Q8. Why should members not just reach the GCS directly?** Grounding assumption
(§1.2): the GCS is distant/obstructed so only the best drone has a viable
backhaul; where a weak direct path exists, L3 gateway routing forces traffic
through the CH. We can demonstrate direct-link SINR below threshold for members.

**Q9. Reproducibility?** Seeded RNG, N runs, confidence intervals, logged
configs.

---

## 10. Assumptions, limitations, extensions

**Assumptions:** single cluster of a few drones; static GCS on an external
subnet reachable only via the gateway; single shared 802.11n radio; loose time
sync (async-tolerant via periodic re-advertisement).

**Limitations:** one cluster (no inter-cluster routing yet); half-duplex relay
throughput penalty; the SINR metric, though locally scoped, is computed by NS-3
(a modelling simplification of real beacon measurement).

**Extensions:** multi-cluster with gateway federation; multi-objective election
(backhaul quality **and** intra-cluster coverage / residual energy, HEED-style);
multi-radio or directional backhaul; predictive (Kalman/ML) link-quality for
pre-emptive handover.

---

## 11. Implementation roadmap (phased, each phase independently testable)

| Phase | Deliverable | Exit test |
|---|---|---|
| **P0** | NS-3 4th node (GCS) on `tap-gcs`; GCS in position feed + obstacle model; drone↔GCS SINR on `/ns3_link_rssi` | `/ns3_link_rssi` shows 6 links; a building between GCS and a drone drops that link |
| **P1** | `olsrd` in all namespaces; GCS on 10.99/24 | plain OLSR connectivity drone↔drone; routes converge |
| **P2** | `ch_agent` election (role + logging, **no** gateway yet) | agents converge to `CH == argmax q_i`; flap rate ≈ 0 under stable channel |
| **P3** | Gateway coupling: CH toggles HNA; members route to GCS via CH | `ping/iperf3` member→GCS flows through the CH (traceroute shows 2 hops) |
| **P4** | Make-before-break handover | forced CH switch → measured convergence + minimal loss |
| **P5** | Evaluation harness + baselines + seeded stats | plots: proposed vs static/random/geometric on PDR/flap/handover |

---

*Companion docs: [`architecture.md`](architecture.md) (base stack),
[`SETUP.md`](SETUP.md) (install/run). This file specifies the clustering
extension only; implementation lands under `scripts/ch_agent*`, an extended
`ns3/three_uav_tapbridge_obstacle_loss.cc`, and `scripts/setup_netns_tap.sh`.*

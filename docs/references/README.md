# References

Papers, theses and documentation this project **reasons from**.

One row per source. Each row says what the source was used to justify. This is a record
of reasoning, not a reading list — if a paper was read but not used to support a
decision, it does not belong here.

When the report or a publication is written, every sentence of the form *"this follows
established practice"* should already have its source in this table.

---

## Co-simulation and hardware-in-the-loop

| # | Source | Used to justify |
|---|---|---|
| R1 | [Emulation Overview — ns-3 Model Library](https://www.nsnam.org/docs/models/html/emulation-overview.html) | TapBridge is the documented way to attach real hardware to an emulated network. This is the mechanism by which the Pi occupies aircraft node 2. |
| R2 | [HOWTO make ns-3 interact with the real world](https://www.nsnam.org/wiki/HOWTO_make_ns-3_interact_with_the_real_world) | Practical setup of TAP devices and bridging; the pattern `scripts/netns/launch_single_uav_netns.sh` follows. |
| R3 | [CORNET: A Co-Simulation Middleware for Robot Networks](https://ece.iisc.ac.in/~parimal/papers/2020/comsnets.pdf) | Prior art for pairing ns-3 with Gazebo/ROS. Uses infrastructure-mode Wi-Fi. |
| R4 | [FlyNetSim: Synchronized UAV Network Simulator based on ns-3 and ArduPilot](https://ar5iv.labs.arxiv.org/html/1808.04967) | Prior art for synchronising ns-3 with ArduPilot — closest to our SITL arrangement. |
| R5 | [Hardware Implementation of FANET Using FANS (ICNS3 2025)](https://dl.acm.org/doi/10.1145/3747204.3747223) | **Nearest published work.** Also puts a real embedded board in the loop (Jetson Nano + Pixhawk 4). Names buffer bloat, socket buffering and clock drift as characteristic HITL problems — the same faults this project hit. Quantifies the sim-to-real gap: 0.40 ms simulated vs 104.35 ms on hardware. Local copy: `docs/FANs.pdf`. |
| R6 | [Performance Evaluation of ns-3 Real-Time Emulation](https://www.researchgate.net/publication/390270733_Performance_Evaluation_of_ns-3_Real-Time_Emulation) | ns-3 real-time mode loses accuracy at high traffic volume. Justifies keeping the 110–481 Mbps camera stream **off** ns-3 — required for the emulation to be trustworthy, not only for physical fidelity. |

## Edge inference on constrained hardware

| # | Source | Used to justify |
|---|---|---|
| R7 | [UAV detection with YOLO on a standalone Raspberry Pi 5](https://ceur-ws.org/Vol-3970/PAPER1.pdf) | Independent confirmation that a Pi cannot meet real-time detection needs. Our ~1,015 ms on a Pi 4B sits where this predicts. |
| R8 | [YOLOv8 / RT-DETR energy efficiency on edge devices](https://www.nature.com/articles/s41598-026-46453-6) | CPU-only inference of large models on a Pi 5 is impractical due to multi-second per-frame latency. Supports reporting slowness as a *finding* rather than a defect. |
| R9 | [Communication–Computation Trade-Off in Resource-Constrained Edge Inference](https://arxiv.org/pdf/2006.02166) | End-to-end latency rises 0.123 s → 2.317 s as bandwidth falls 1 Mbps → 50 Kbps. The curve the ground arm of this experiment traces. |

## Environment construction

| # | Source | Used to justify |
|---|---|---|
| R10 | *(to add)* Georeferenced terrain into Gazebo via BlenderGIS — the SSRN preprint on Antisana wetland UAV simulation | The GIS → SRTM → mesh → Gazebo pipeline used for the campus world. Also records two STL limitations (manual rescaling, no texture coordinates) that our COLLADA export avoids. **Add the DOI/URL — currently only the abstract ID 4778454 is known.** |

---

## To add

Sources still needed, with what each would support:

- **Clock synchronisation for distributed measurement** — chrony/PTP accuracy over Ethernet, to justify the method once cross-host end-to-end latency is reported. R5 calls this the critical unsolved problem but does not solve it.
- **DDS / Fast DDS large-sample transport** — fragmentation, socket buffer sizing, QoS effects on throughput. Would let the `net.core.wmem_max` finding be framed against documented behaviour rather than only our own measurement.
- **VLAN or link separation in robotic testbeds** — no source found so far. If none exists, the sensor/radio split is a contribution and should be claimed as one.

# Edge Processing — Raspberry Pi 4B Hardware-in-the-Loop

Branch: `ground-vs-edge-processing-RPi`

Running the vision detection **onboard the drone**, on a real Raspberry Pi 4B, so only
tiny detection results — not video — cross the simulated wireless link. For the base
simulation setup see [README.md](README.md); for the full design rationale, measurements
and open problems see [docs/HITL_INTEGRATION_PLAN.md](docs/HITL_INTEGRATION_PLAN.md).
The command sheet for an actual run is [docs/COMMANDS.txt](docs/COMMANDS.txt).

> The board is a **Pi 4B**, not a Pi 5. That is what allows the exact same stack as the
> host — Ubuntu 22.04 + ROS 2 Humble natively, with an identical DDS implementation.

---

## 1. What edge processing means here

A surveillance drone sees people with its camera. In **edge processing** the drone does
the work itself: YOLO runs on the Pi and produces a small detection result (class,
confidence, bounding box), and **only that result** is sent to the ground station.

| | crosses the radio link | inference runs | status |
|---|---|---|---|
| **Edge** | **72–433 B** per detection | on the Pi 4B, 236 ms | ✅ working |
| **Ground** | ~100 KB JPEG per frame | on the host, ~44 ms | ⬜ not yet run |

The payload is not a constant — it grows with the number of people found: ~72 B of JSON
envelope plus ~72 B per detection. Against the delivered 0.74 MB frame that is ~5,200×
smaller for a single detection. The heavy video never crosses the radio — it stays on the
drone.

---

## 2. The rig — as built

The Pi 4B is **UAV2's onboard computer**. Its autopilot (SITL) still runs on the host;
only its vision runs on the Pi, and the Pi does not command the drone.

```
   HOST                                              PI 4B  (the UAV2 slot)
   ────                                              ─────────────────────
   Gazebo ─── eth-cam  VLAN 10 ═══ cable ═══ eth0.10 ──► detector (YOLO onboard)
              10.0.0.1                       10.0.0.2         │
              ① SENSOR LINK — unimpaired                      │  72-433 B
                                                              │  detections
   gcsns ◄── tap-gcs ◄── ns-3 ◄── tap-uav2 ◄── br-uav2 ◄──────┘
   10.42.0.10             ▲       eth-rf VLAN 42      eth0.42
   gcs_receiver     loss · latency · fading           10.42.0.12
                    ② WIRELESS LINK — IMPAIRED

   (host only:)  SITL ◄── FDM over veth 172.31.1.1/.2 ──► Gazebo    never impaired
```

The Pi has one Ethernet port, so both links share one cable and are separated with
**802.1Q VLANs**. Routing enforces the split with no firewall rules: Gazebo can only reach
the Pi at `10.0.0.2`, `gcsns` can only reach it at `10.42.0.12`.

| Link | Subnet | Carries | Impaired by ns-3? |
|---|---|---|---|
| ① Sensor | `10.0.0.x` (VLAN 10) | Gazebo camera → Pi | No |
| ② Wireless | `10.42.0.x` (VLAN 42) | Pi → GCS: detections only | **Yes** |

---

## 3. Why the camera crosses a cable, unimpaired

The camera is simulated in Gazebo on the host, so its frames have to physically reach the
Pi. On a real drone the camera is bolted to the Pi by a **CSI ribbon cable** inside one
airframe — the Ethernet cable stands in for that. Degrading it would be physically
meaningless, so it is deliberately kept off ns-3 and is **not** part of what is measured.

A bridge would flood the camera stream into `tap-uav2` and through ns-3 as well, which is
why the split is done with VLANs rather than by putting the NIC into `br-uav2`.

---

## 4. The nodes (`ros2/uav_vision/`)

| Node | Where | Role | In the HITL edge path? |
|---|---|---|---|
| `detector` | Pi | YOLO on the frame → `/detections/uavN` (72–433 B) | ✅ yes |
| `gcs_receiver` | GCS (`gcsns`) | receives the detections across ns-3 | ✅ yes |
| `metrics_logger` | GCS | per-frame CSV: size, latency, `navsat_age` | optional |
| `camera_relay` | drone | JPEG-compress + republish for **ground** mode | ground mode only |

In edge mode the detector subscribes **directly** to `/uav1/camera/image_raw`; throttling
happens at the source (`<update_rate>` in `models/iris_1_netns/model.sdf`), because a
relay-side throttle applies after receipt and would not reduce link bandwidth.

The nodes deliberately avoid `cv_bridge` (NumPy 2.x incompatibility) and do their own
Image↔numpy conversion.

---

## 5. Running it

Four scripts, each verifying its stage before the next begins. Run them in order; leave
steps 1 and 2 running in their own terminals.

```bash
./scripts/netns/rpi_init.sh                 # 0  board network, config, buffers, clock
./scripts/netns/sitl_init.sh --gui --view   # 1  host pipeline, ends at PIPELINE READY
./scripts/netns/detector_start.sh           # 2  Pi detectors + gcs_receivers
./scripts/netns/run_missions.sh             # 3  fly
```

They are separate on purpose: initialisation must be fully verified before a mission arms
anything, and each stage can then be debugged on its own. Ctrl+C in step 2 also stops the
detectors on the boards; Ctrl+C in step 1 tears down the whole pipeline.

Check these first — each has caused a silent failure before:

```bash
[Pi]   ip -4 addr show | grep -c "10.0.0.2/24"     # must print 1, not 2
[Pi]   chronyc tracking | grep "System time"       # clock offset, a few us
[Host] sysctl net.core.rmem_max net.core.wmem_max  # both 536870912
[Host] ping -c 3 10.0.0.2                          # camera link is up from boot
```

`10.0.0.1` must exist **before** Gazebo starts — FastDDS reads its interface list once, at
participant creation. If `eth-cam` is missing: `sudo nmcli connection up eth-cam`.

**Success:** `gcs_receiver` logs `[GCS] detection #.... | size=75B` — those bytes crossed
the simulated radio. Watch the logs with `tail -f /tmp/hitl_detector.log` and
`/tmp/hitl_gcs.log`; view boxes on the live camera with `scripts/detection_viewer.py`.

---

## 6. Measured

Delivered configuration: camera 640×384 at 5 Hz, FOV 0.6 rad, pitch 45°; YOLO11n OpenVINO
on the Pi, `conf=0.4`, class 0 only.

| Metric | Value |
|---|---|
| Sensor link latency (60 pings) | min 1.0 / **avg 1.4** / max 1.8 ms, jitter 0.12 ms |
| Radio link latency (30 pings) | min 10.1 / **avg 74.5** / max 158 ms, jitter 38.4 ms |
| Radio vs sensor path | **53× latency**, jitter 2 orders of magnitude larger |
| Camera delivered to the Pi | 5.000 Hz vs 5 Hz source, σ = 4.5 ms, 0 drops |
| Inference — OpenVINO | **236 ms** (fastest correct back-end) |
| Inference — NCNN 384×640 | 270 ms standalone, 277 ms in pipeline (3.47 fps) |
| Inference — PyTorch 640 | 1,027 ms |
| Clock offset, Pi to host | **2–60 µs** (chrony over the sensor link) |
| Detection payload | **72 B** empty, **143 B** one person, **433 B** five — vs a 0.74 MB frame |
| Detection quality | 1–4 of 5 subjects per frame when in view, conf 0.42–0.59 |

Inference went from 1,343 ms to 236 ms — a factor of 5.7 — entirely through configuration:
removing the annotated return stream, matching the camera to the model input, and switching
PyTorch → NCNN → OpenVINO. Published **Pi 5** back-end rankings do not transfer to a Pi 4B;
the Cortex-A72 is ARMv8.0-A and lacks the dot-product and fp16 instructions those back-ends
lean on.

> **Trap:** never pass `imgsz` to an exported model — its input shape is fixed at export
> time. OpenVINO raises on the mismatch; NCNN does not, and returns a stable wrong answer
> from the second call onward. Both detectors now drop `imgsz` for anything but a `.pt`.

---

## 7. Key files

| Path | What |
|---|---|
| `scripts/netns/rpi_init.sh` | verify both Pi boards before anything starts |
| `scripts/netns/sitl_init.sh` | cold-start the host pipeline; ends at PIPELINE READY |
| `scripts/netns/detector_start.sh` | Pi detectors over SSH + `gcs_receiver` in `gcsns` |
| `scripts/netns/run_missions.sh` | fly `two_drone_mission.py` |
| `scripts/netns/launch_single_uav_netns.sh` | 8-stage host bring-up; STEP 1c adds the VLANs + `br-uav2` |
| `scripts/netns/kill_all_netns.sh` | teardown (also kills gzserver) |
| `scripts/yolo_detect_node.py` | standalone detector + timing; runs on the Pi |
| `scripts/bench_backends.py` | back-end benchmark (rejects any whose detection count varies) |
| `scripts/detection_viewer.py` | draw the received boxes on the live camera, host-side |
| `scripts/pi_setup.sh` | one-shot Pi 4B installer (incl. the CPU-only torch fix) |
| `ros2/uav_vision/` | `detector`, `gcs_receiver`, `metrics_logger`, `camera_relay` |
| `config/fastdds_hitl_eth.xml` | DDS profile for HITL — interface whitelist, UDP-only |
| `config/fastdds_udp_only.xml` | UDP-only profile for host-only runs (bypasses shared memory) |
| `models/iris_1_netns/model.sdf` | camera: 640×384, 45° pitch, 0.6 rad FOV, 5 Hz |
| `worlds/small_city_single_uav_netns.world` | city world, lockstep physics, 5 `person_standing` |
| `Makefile.edge` | superseded multi-terminal host-only bring-up, kept as a fallback |
| `docs/HITL_INTEGRATION_PLAN.md` | full plan, phase status, every gotcha |
| `docs/PI_SETUP.md` | Pi flashing + setup guide |
| `docs/COMMANDS.txt` | the command sheet for a run |

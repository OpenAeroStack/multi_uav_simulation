# Current network and port mapping

This document describes the active `scripts/launch_city_dds.sh` topology.
“Configured” means the value was verified in the launcher, parameter file,
active UAV SDF, topology script, or ArduPilot source. “Runtime-confirmed”
means a live socket/interface was observed with `ss` or `ip`.

Audit date: 2026-07-25.

## Runtime status during this audit

The unified simulation was not running. `ip netns list` returned no
namespaces; no relevant listeners appeared in `ss`; the TAP devices and
bridges did not exist; and no Gazebo, SITL, agent, NS-3, or bridge process/log
was present. Therefore the configuration is fully audited below, but live
ports, routes, bridge membership, and counters remain explicitly unconfirmed.
No port was changed merely to make the numbering look regular.

## Complete port table

| Plane | UAV | Listener | Protocol | Sender / destination | Configured source | Runtime |
|---|---:|---|---|---|---|---|
| DDS/XRCE | 1 | micro-ROS agent in `gcsns`, `10.42.0.10:2019` | UDP | UAV1 AP_DDS client → `10.42.0.10:2019` | `params/uav1_dds.parm`; launcher agent port 2019 | Not running |
| DDS/XRCE | 2 | micro-ROS agent in `gcsns`, `10.42.0.10:2020` | UDP | UAV2 AP_DDS client → `10.42.0.10:2020` | `params/uav2_dds.parm`; launcher agent port 2020 | Not running |
| DDS/XRCE | 3 | micro-ROS agent in `gcsns`, `10.42.0.10:2021` | UDP | UAV3 AP_DDS client → `10.42.0.10:2021` | `params/uav3_dds.parm`; launcher agent port 2021 | Not running |
| MAVLink | 1 | SITL in `uav1`, `0.0.0.0:5760` (reachable as `10.42.0.11:5760`) | TCP | `drone_bridge_uav1` in `gcsns` connects to `10.42.0.11:5760` | launcher `--serial0=tcp:0.0.0.0:5760` and bridge destination | Not running |
| MAVLink | 2 | SITL in `uav2`, `0.0.0.0:5770` (reachable as `10.42.0.12:5770`) | TCP | `drone_bridge_uav2` in `gcsns` connects to `10.42.0.12:5770` | launcher `--serial0=tcp:0.0.0.0:5770` and bridge destination | Not running |
| MAVLink | 3 | SITL in `uav3`, `0.0.0.0:5780` (reachable as `10.42.0.13:5780`) | TCP | `drone_bridge_uav3` in `gcsns` connects to `10.42.0.13:5780` | launcher `--serial0=tcp:0.0.0.0:5780` and bridge destination | Not running |
| Gazebo servo input | 1 | Gazebo plugin at `172.31.1.1:9002` | UDP | SITL sends servo commands from its Gazebo socket to `172.31.1.1:9002` | `iris_1/model.sdf`: `listen_addr`, `fdm_port_in`; ArduPilot instance 0 `SIM_OUT_PORT` | Not running |
| SITL FDM input | 1 | SITL in `uav1`, `0.0.0.0:9003` (address `172.31.1.2`) | UDP | Gazebo sends FDM state to `172.31.1.2:9003` | `iris_1/model.sdf`: `fdm_addr`, `fdm_port_out`; ArduPilot instance 0 `SIM_IN_PORT` | Not running |
| Gazebo servo input | 2 | Gazebo plugin at `172.31.2.1:9012` | UDP | SITL sends servo commands to `172.31.2.1:9012` | `iris_2/model.sdf`; ArduPilot instance 1 adds 10 | Not running |
| SITL FDM input | 2 | SITL in `uav2`, `0.0.0.0:9013` (address `172.31.2.2`) | UDP | Gazebo sends FDM state to `172.31.2.2:9013` | `iris_2/model.sdf`; ArduPilot instance 1 adds 10 | Not running |
| Gazebo servo input | 3 | Gazebo plugin at `172.31.3.1:9022` | UDP | SITL sends servo commands to `172.31.3.1:9022` | `iris_3/model.sdf`; ArduPilot instance 2 adds 20 | Not running |
| SITL FDM input | 3 | SITL in `uav3`, `0.0.0.0:9023` (address `172.31.3.2`) | UDP | Gazebo sends FDM state to `172.31.3.2:9023` | `iris_3/model.sdf`; ArduPilot instance 2 adds 20 | Not running |
| Gazebo master | — | root namespace, TCP `11345` | TCP | Gazebo clients/plugins connect locally | Gazebo Classic default; launcher explicitly checks TCP 11345 | Not running |

### Meaning of the 900x fields

The names are from the Gazebo plugin’s perspective:

- `fdm_port_in` is the UDP port the Gazebo plugin **binds and listens on** for
  incoming SITL servo commands: 9002, 9012, 9022.
- `fdm_port_out` is the UDP **destination port** to which Gazebo sends FDM
  state: the SITL listeners at 9003, 9013, 9023.

The ArduPilot side uses complementary terminology and behavior. Its
`SIM_IN_PORT` defaults to 9003 and is bound by SITL; its `SIM_OUT_PORT`
defaults to 9002 and is the Gazebo destination. `--instance` adds 10 per
instance. The launcher uses instances 0, 1, and 2, exactly matching all three
active SDFs.

## Interface table

| Location | Interface | Address | Peer / attachment | Purpose |
|---|---|---|---|---|
| `gcsns` | `wifi0` | `10.42.0.10/24` | `veth-gcs-host` → `br-gcs` → `tap-gcs` | DDS agents and mission/bridges across NS-3 |
| `uav1` | `wifi0` | `10.42.0.11/24` | `veth-uav1-host` → `br-uav1` → `tap-uav1` | UAV1 DDS and MAVLink across NS-3 |
| `uav2` | `wifi0` | `10.42.0.12/24` | `veth-uav2-host` → `br-uav2` → `tap-uav2` | UAV2 DDS and MAVLink across NS-3 |
| `uav3` | `wifi0` | `10.42.0.13/24` | `veth-uav3-host` → `br-uav3` → `tap-uav3` | UAV3 DDS and MAVLink across NS-3 |
| root | `sim-uav1-host` | `172.31.1.1/30` | `uav1/sim0` | Gazebo physics endpoint for UAV1 |
| `uav1` | `sim0` | `172.31.1.2/30` | root `sim-uav1-host` | SITL physics endpoint for UAV1 |
| root | `sim-uav2-host` | `172.31.2.1/30` | `uav2/sim0` | Gazebo physics endpoint for UAV2 |
| `uav2` | `sim0` | `172.31.2.2/30` | root `sim-uav2-host` | SITL physics endpoint for UAV2 |
| root | `sim-uav3-host` | `172.31.3.1/30` | `uav3/sim0` | Gazebo physics endpoint for UAV3 |
| `uav3` | `sim0` | `172.31.3.2/30` | root `sim-uav3-host` | SITL physics endpoint for UAV3 |

The root-side wireless veths, bridges, and TAPs intentionally have no IP
addresses. Each bridge has exactly two ports: its endpoint veth and TAP.
Management links are isolated /30 point-to-point networks and are not attached
to the wireless bridges.

## Namespace table

| Namespace | Wireless identity | Management identity | Processes / listeners |
|---|---|---|---|
| root | No `10.42.0.0/24` address or route | `172.31.1.1`, `.2.1`, `.3.1` | Gazebo, NS-3; UDP 9002/9012/9022; TCP 11345 |
| `gcsns` | `10.42.0.10` | None | micro-ROS agents UDP 2019–2021; drone bridges and mission clients |
| `uav1` | `10.42.0.11` | `172.31.1.2` | SITL TCP 5760; SITL UDP 9003 |
| `uav2` | `10.42.0.12` | `172.31.2.2` | SITL TCP 5770; SITL UDP 9013 |
| `uav3` | `10.42.0.13` | `172.31.3.2` | SITL TCP 5780; SITL UDP 9023 |

No namespace has a default route in this topology. Wireless destinations are
on-link within `10.42.0.0/24`; each management /30 is also directly connected.

## Data-flow directions

```text
DDS (UDP, must cross NS-3)
uavN AP_DDS client
  -> uavN/wifi0 (10.42.0.1N)
  -> endpoint bridge/TAP -> NS-3 Wi-Fi -> tap-gcs/bridge
  -> gcsns/wifi0 (10.42.0.10) -> agent UDP 2018+N

MAVLink (TCP, must cross NS-3)
gcsns drone_bridge
  -> gcsns/wifi0 -> tap-gcs -> NS-3 Wi-Fi -> tap-uavN
  -> uavN/wifi0 (10.42.0.1N) -> SITL TCP 5760/5770/5780

Gazebo physics (UDP, must not cross NS-3)
uavN SITL UDP 9003/9013/9023
  -> uavN/sim0 (172.31.N.2)
  -> point-to-point management veth
  -> Gazebo UDP 9002/9012/9022 at 172.31.N.1

Gazebo FDM return (UDP, must not cross NS-3)
Gazebo
  -> 172.31.N.2:9003/9013/9023 over the same management veth
  -> uavN SITL FDM listener
```

## Live verification commands

Run while the unified launcher is active:

```bash
sudo ss -H -lntup |
  grep -E ':(11345|9002|9012|9022)\b'

for ns in gcsns uav1 uav2 uav3; do
  sudo ip netns exec "$ns" ip -brief address
  sudo ip netns exec "$ns" ip route
  sudo ip netns exec "$ns" ss -H -lntup
done

bridge link

for tap in tap-gcs tap-uav1 tap-uav2 tap-uav3; do
  ip -s link show "$tap"
done
```

Expected listener distinction:

- `gcsns`: UDP 2019/2020/2021.
- `uav1/uav2/uav3`: TCP 5760/5770/5780 and UDP 9003/9013/9023.
- root: UDP 9002/9012/9022 and TCP 11345.
- UDP destination ports do not necessarily appear as listeners in the sending
  namespace. Use `ss` to identify bound listeners and configuration or packet
  capture to establish destinations.

During a live traffic test, RX/TX packet and byte counters on all four TAPs
must increase for DDS/MAVLink traffic. Physics traffic must instead increase
the `sim-uavN-host` / `uavN/sim0` counters.

## Corrections and unresolved runtime evidence

No configuration conflict or mismatch was found, so no ports, addresses,
routes, or topology were changed.

Still requiring a live unified run for confirmation:

- UDP agent listeners 2019–2021 in `gcsns`.
- TCP SITL listeners 5760/5770/5780 in UAV namespaces.
- Root Gazebo UDP listeners 9002/9012/9022.
- UAV SITL UDP listeners 9003/9013/9023.
- Gazebo master TCP listener 11345.
- Namespace routes, exact bridge membership, and live TAP/management counters.

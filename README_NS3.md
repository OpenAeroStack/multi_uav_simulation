# README — Three UAV WiFi Ad-hoc Network with NS-3, ROS2, and TapBridge

# Overview

This project implements a real-time UAV network simulation using NS-3 integrated with external systems such as ROS 2 and Gazebo through UDP communication and Linux TAP interfaces.

The architecture simulates:

- Three UAVs communicating over a wireless ad-hoc WiFi network
- Dynamic UAV mobility updates from an external simulator
- Real-time propagation penalties for obstacle-aware communication
- OLSR-based multi-hop routing
- ROS2 integration through TapBridge virtual interfaces
- Real-time synchronization using the NS-3 realtime scheduler

---

# System Architecture

## High-Level Architecture

```text
                 +---------------------------+
                 |        ROS2 / Gazebo      |
                 |---------------------------|
                 | UAV Pose Publisher        |
                 | Obstacle Analysis         |
                 | Link Penalty Generator    |
                 +-------------+-------------+
                               |
                               | UDP Port 5555
                               |
                -----------------------------------
                               |
                               v

+-------------------------------------------------------------+
|                     NS-3 Realtime Simulator                 |
|-------------------------------------------------------------|
|                                                             |
|  +-------------+      WiFi Adhoc Network     +-------------+|
|  |    UAV 1    |<--------------------------->|    UAV 2    ||
|  +-------------+                             +-------------+|
|         ^                                           ^      |
|         |                                           |      |
|         +--------------------+----------------------+      |
|                              |                             |
|                              v                             |
|                       +-------------+                      |
|                       |    UAV 3    |                      |
|                       +-------------+                      |
|                                                             |
|         OLSR Routing + Dynamic Propagation Loss             |
+-------------------------------------------------------------+

        |                |                 |
        |                |                 |
        v                v                 v

   tap-uav1         tap-uav2         tap-uav3

        |                |                 |
        +------ Linux TAP Interfaces ------+

Main Components
1. UAV Nodes

The simulation creates:

g_uavNodes.Create(3);

Three wireless UAV nodes are simulated inside NS-3.

Each UAV contains:

WiFi device
Internet stack
OLSR routing
Mobility model
Real-time position updates
2. Proxy Nodes
proxyNodes.Create(3);

Each UAV is paired with a proxy node through a CSMA link.

Purpose
Connect NS-3 nodes to Linux TAP interfaces
Allow ROS2 applications to communicate using standard Linux networking
Architecture per UAV
ROS2 App
   |
tap-uavX
   |
Proxy Node ---- CSMA ---- UAV Node
3. WiFi Ad-hoc Network
Standard
wifi.SetStandard(WIFI_STANDARD_80211g);

The UAVs communicate using:

IEEE 802.11g
Ad-hoc MAC mode
No access point required
Data Rate
ErpOfdmRate54Mbps

Configured rates:

Type	Rate
Data	54 Mbps
Control	24 Mbps
4. Routing Protocol
OLSR (Optimized Link State Routing)
OlsrHelper olsr;

OLSR enables:

Dynamic route discovery
Multi-hop forwarding
Automatic topology adaptation

Useful when UAVs move and routes change continuously.

5. Mobility System
Mobility Model
ConstantPositionMobilityModel

Although initialized statically, UAV positions are updated dynamically via UDP messages from ROS2/Gazebo.

External Pose Updates

UDP messages:

POSE <uavIndex> <x> <y> <z>

Example:

POSE 1 20.5 15.2 3.0
Workflow
Gazebo/ROS2
    ↓
UDP Listener Thread
    ↓
Simulator::ScheduleWithContext()
    ↓
NS-3 MobilityModel::SetPosition()

This ensures thread-safe mobility updates inside the NS-3 simulation loop.

6. Dynamic Obstacle Loss Model
Custom Propagation Model
class UdpObstacleLossModel : public PropagationLossModel

This model injects externally computed communication penalties.

Penalty Messages

UDP format:

PENALTY <txNode> <rxNode> <loss>

Example:

PENALTY 0 1 18.5

Meaning:

Communication from node 0 → node 1 experiences an additional 18.5 dB loss
Propagation Chain
LogDistancePropagationLossModel
              ↓
UdpObstacleLossModel

Total received power:

RxPower = TxPower
          - LogDistanceLoss
          - ObstaclePenalty

This allows realistic modeling of:

Buildings
Trees
Walls
RF shadowing
Dynamic environmental blockage
7. Real-Time Execution

The simulation runs using:

RealtimeSimulatorImpl

This synchronizes NS-3 simulation time with wall-clock time.

Benefits
Live ROS2 interaction
Hardware-in-the-loop compatibility
Real-time networking experiments
8. TAP Bridge Integration

Each proxy node is attached to a Linux TAP device.

Configured TAP interfaces:

tap-uav1
tap-uav2
tap-uav3

Mode used:

ConfigureLocal

This allows external applications to exchange packets with NS-3 as if the simulated UAVs were real network devices.

9. Network Topology
Wireless Network
10.250.0.0/24

Used for UAV-to-UAV communication.

CSMA/TAP Links
UAV	Subnet
UAV1	10.1.1.0/24
UAV2	10.1.2.0/24
UAV3	10.1.3.0/24
10. Flow Monitoring

The script optionally enables:

FlowMonitorHelper

Generated output:

three_uav_wifi_ros_flowmon.xml
Metrics Collected
Throughput
Packet loss
Delay
Jitter
Flow statistics

Useful for performance analysis.

UDP Communication Interface
UDP Port
5555

The NS-3 simulator listens for external updates on this port.

Supported Message Types
Position Update
POSE <uav> <x> <y> <z>

Example:

POSE 2 40.0 15.0 5.0
Link Penalty Update
PENALTY <tx> <rx> <penalty>

Example:

PENALTY 1 2 12.0
Threading Architecture

A dedicated background thread handles UDP reception:

std::thread listenerThread(UdpListenerThread);
Responsibilities
Receive external updates
Parse messages
Update shared link penalties
Schedule mobility updates safely

Synchronization uses:

std::mutex g_penaltyMutex;
Key Design Features
Real-Time ROS2 Integration

Enables live interaction between:

ROS2 nodes
Gazebo simulations
NS-3 networking
Dynamic RF Environment

Obstacle-aware propagation allows:

Environment-sensitive communication
Adaptive routing behavior
Realistic UAV networking
Multi-Hop UAV Communication

OLSR automatically handles:

Route establishment
Topology changes
Link failures

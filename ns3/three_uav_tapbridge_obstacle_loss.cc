/*
 * Updated from 'three_uav_tapbridge_rt.cc'
 *
 * Real-time NS-3 scenario for multi_uav_sim.
 * Bridges three network namespaces (uav1ns/uav2ns/uav3ns) into a simulated
 * 802.11n ad-hoc channel with Nakagami fading + log-distance path loss.
 *
 * Communication with Gazebo/ROS2 is now done via a native rclcpp::Node
 * embedded directly in this NS-3 process -- no ZMQ, no separate Python
 * bridge process. Subscribes to /uav_world_positions and
 * /link_obstacle_loss, publishes /ns3_link_rssi.
 *
 * NS-3 version: 3.38.1  Ubuntu: 22.04  ROS2: Humble
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/propagation-module.h"
#include "ns3/internet-module.h"
#include "ns3/stats-module.h"

#include "dynamic_obstacle_loss_model.hh"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <thread>
#include <atomic>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThreeUavTapBridge");

static std::atomic<bool> g_rosRunning{true};

// Per-link metrics log for offline model validation (opened in main() if
// --csvPath is given). Written on the NS-3 main thread from PublishStats().
static std::ofstream g_csv;

// ───────────────────────────────────────────────────────────────────────
// Ns3RosNode: a single rclcpp::Node living inside the NS-3 process.
// It owns:
//   - a subscriber to /uav_world_positions  (drives ConstantVelocityMobilityModel)
//   - a subscriber to /link_obstacle_loss   (drives DynamicObstacleLossModel)
//   - a publisher  to /ns3_link_rssi        (replaces the old ZMQ PUB)
//
// All callbacks below run on rclcpp's executor thread (spun in main()),
// NOT on the NS-3 simulation thread. Just like the ZMQ listener thread
// before it, every callback hands its work to NS-3 via
// Simulator::ScheduleWithContext(), which is documented as thread-safe
// under RealtimeSimulatorImpl. This is the same threading pattern as
// before -- only the transport changed.
// ───────────────────────────────────────────────────────────────────────
class Ns3RosNode : public rclcpp::Node
{
public:
  Ns3RosNode(Ptr<DynamicObstacleLossModel> obstacleLoss, NodeContainer nodes)
  : rclcpp::Node("ns3_bridge_node"),
    m_obstacleLoss(obstacleLoss),
    m_nodes(nodes)
  {
    m_posSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/uav_world_positions", 10,
      std::bind(&Ns3RosNode::OnPositions, this, std::placeholders::_1));

    m_obsSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/link_obstacle_loss", 10,
      std::bind(&Ns3RosNode::OnObstacleLoss, this, std::placeholders::_1));

    m_rssiPub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/ns3_link_rssi", 10);

    // ADDED: headline SNR metric, so the demo can show per-link SNR live
    // (rqt/rosbag) without changing the existing RSSI layout above.
    m_snrPub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/ns3_link_snr", 10);
  }

  // Called by PublishStats() on the NS-3 main thread -- this is fine
  // because rclcpp publishers are themselves thread-safe to call publish() on.
  void PublishRssi(const std::vector<float> & flatData)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = flatData;
    m_rssiPub->publish(msg);
  }

  // Flat layout [node_a, node_b, snr_dB, ...] -- same convention as RSSI.
  void PublishSnr(const std::vector<float> & flatData)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = flatData;
    m_snrPub->publish(msg);
  }

private:
  // /uav_world_positions payload convention (same as the Gazebo plugin
  // from earlier in this guide): [id, x, y, z, id, x, y, z, ...]
  void OnPositions(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const auto & data = msg->data;
    for (size_t i = 0; i + 3 < data.size(); i += 4)
    {
      uint32_t uid = static_cast<uint32_t>(data[i]);
      double x = data[i+1], y = data[i+2], z = data[i+3];
      if (uid >= m_nodes.GetN()) continue;

      auto nodes = m_nodes;  // capture by value for the lambda
      Simulator::ScheduleWithContext(uid, Seconds(0),
        MakeEvent([nodes, uid, x, y, z]() {
          auto mob = nodes.Get(uid)->GetObject<ConstantVelocityMobilityModel>();
          if (mob) {
            mob->SetPosition(Vector(x, y, z));
          }
        }));
    }
  }

  // /link_obstacle_loss payload convention: [i, j, loss_dB, i, j, loss_dB, ...]
  void OnObstacleLoss(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const auto & data = msg->data;
    for (size_t i = 0; i + 2 < data.size(); i += 3)
    {
      uint32_t a = static_cast<uint32_t>(data[i]);
      uint32_t b = static_cast<uint32_t>(data[i+1]);
      double lossDb = data[i+2];

      auto obstacleLoss = m_obstacleLoss;
      Simulator::ScheduleWithContext(a, Seconds(0),
        MakeEvent([obstacleLoss, a, b, lossDb]() {
          obstacleLoss->SetObstacleLoss(a, b, lossDb);
        }));
    }
  }

  Ptr<DynamicObstacleLossModel> m_obstacleLoss;
  NodeContainer m_nodes;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_posSub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_obsSub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_rssiPub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_snrPub;
};

// ───────────────────────────────────────────────────────────────────────
// Per-link metrics export + model-validation logging.
//
// For each UAV pair we compute and record the full loss decomposition so the
// composite result can be traced back to each model:
//
//   pathloss_rx  = Tx - L_pathloss                 (log-distance model alone)
//   obstacle_dB  = smoothed shadowing              (from /link_obstacle_loss)
//   faded_rx     = Tx - L_pathloss - obstacle_dB + fading   (full chain, 1 draw)
//   fading_dB    = faded_rx - (pathloss_rx - obstacle_dB)   (the fading term)
//   snr_dB       = faded_rx - noise_floor          (headline metric)
//
// The fading draw here is an independent realization from what data packets
// see -- it samples the same Nakagami distribution, which is exactly what the
// distribution-matching validation plot needs.
// ───────────────────────────────────────────────────────────────────────
void PublishStats(Ptr<DynamicObstacleLossModel> obstacleLoss,
                   Ptr<PropagationLossModel> pathLossOnly,
                   double noiseFloorDbm,
                   NodeContainer nodes,
                   std::shared_ptr<Ns3RosNode> rosNode)
{
    std::vector<float> rssiFlat;
    std::vector<float> snrFlat;
    double now = Simulator::Now().GetSeconds();

    for (uint32_t i = 0; i < nodes.GetN(); i++)
    {
        for (uint32_t j = i + 1; j < nodes.GetN(); j++)
        {
            Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel>();
            Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel>();
            if (!mobI || !mobJ) continue;

            double txPowerDbm = 20.0;

            // Deterministic path-loss-only rx (log-distance model on its own).
            double pathLossRx = pathLossOnly->CalcRxPower(txPowerDbm, mobI, mobJ);
            // Full chain: path loss - obstacle shadowing + one fading draw.
            double fadedRx = obstacleLoss->CalcRxPower(txPowerDbm, mobI, mobJ);

            uint32_t idI = nodes.Get(i)->GetId();
            uint32_t idJ = nodes.Get(j)->GetId();

            // Deterministic link state (no fading draw) for the decomposition.
            auto info = obstacleLoss->GetLinkLossInfo(idI, idJ);
            double deterministicRx = pathLossRx - info.obstacleLossDb;
            double fadingDeltaDb   = fadedRx - deterministicRx;
            double snrDb           = fadedRx - noiseFloorDbm;
            double distance        = mobI->GetDistanceFrom(mobJ);

            // Flat layouts: [node_a, node_b, value, ...]
            rssiFlat.push_back(static_cast<float>(idI));
            rssiFlat.push_back(static_cast<float>(idJ));
            rssiFlat.push_back(static_cast<float>(fadedRx));

            snrFlat.push_back(static_cast<float>(idI));
            snrFlat.push_back(static_cast<float>(idJ));
            snrFlat.push_back(static_cast<float>(snrDb));

            if (g_csv.is_open())
            {
                g_csv << now << ',' << idI << ',' << idJ << ','
                      << distance << ',' << pathLossRx << ','
                      << info.obstacleLossDb << ',' << fadedRx << ','
                      << fadingDeltaDb << ',' << snrDb << ','
                      << (info.blocked ? 1 : 0) << ',' << info.fadingM << ','
                      << (info.known ? 1 : 0) << '\n';
            }
        }
    }
    if (g_csv.is_open()) g_csv.flush();

    rosNode->PublishRssi(rssiFlat);
    rosNode->PublishSnr(snrFlat);

    Simulator::Schedule(Seconds(0.5), &PublishStats, obstacleLoss, pathLossOnly,
                        noiseFloorDbm, nodes, rosNode);
}

int main(int argc, char *argv[])
{
    // ── Command-line overrides ──────────────────────────────────────────────
    std::string tapBase    = "tap-uav";
    double      simTime    = 0;
    double      distance   = 50.0;
    std::string csvPath    = "";      // empty -> no CSV logging
    // Noise floor for SNR: kTB over the 802.11n 20 MHz channel plus a ~7 dB
    // receiver noise figure => -174 + 10log10(20e6) + 7 ≈ -94 dBm.
    double      noiseFloor = -94.0;

    CommandLine cmd;
    cmd.AddValue("tapBase",    "TAP device name prefix",        tapBase);
    cmd.AddValue("simTime",    "Sim duration (0=unlimited)",    simTime);
    cmd.AddValue("distance",   "Initial UAV separation metres", distance);
    cmd.AddValue("csvPath",    "Per-link metrics CSV path (empty=off)", csvPath);
    cmd.AddValue("noiseFloor", "Receiver noise floor dBm for SNR",      noiseFloor);
    cmd.Parse(argc, argv);

    // ── Open the validation CSV (header row: one line per link per sample) ──
    if (!csvPath.empty())
    {
        g_csv.open(csvPath);
        if (g_csv.is_open())
        {
            g_csv << "t_sim,node_a,node_b,distance_m,pathloss_rx_dbm,"
                     "obstacle_loss_db,faded_rx_dbm,fading_delta_db,snr_db,"
                     "blocked,fading_m,known\n";
        }
    }

    // ── Global config ───────────────────────────────────────────────────────
    GlobalValue::Bind("SimulatorImplementationType",
                      StringValue("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    // ── 3 UAV nodes ────────────────────────────────────────────────────────
    NodeContainer nodes;
    nodes.Create(3);

    // ── Channel: Obstacle loss (incl. LoS-aware fading) -> LogDistance ──────
    Ptr<DynamicObstacleLossModel> obstacleLoss =
        CreateObject<DynamicObstacleLossModel>();

    Ptr<LogDistancePropagationLossModel> logDist =
        CreateObject<LogDistancePropagationLossModel>();
    // REMOVED: exponent 2.7 models urban ground-level propagation; these are
    // UAV-to-UAV air-to-air links, which are close to free space (n = 2.0):
    // logDist->SetAttribute("Exponent",         DoubleValue(2.7));
    // ADDED:
    logDist->SetAttribute("Exponent",         DoubleValue(2.0));
    logDist->SetAttribute("ReferenceDistance", DoubleValue(1.0));
    logDist->SetAttribute("ReferenceLoss",     DoubleValue(46.67));

    // REMOVED: static Nakagami with distance-selected m (m = 1.0 past 80 m
    // assumed NLoS Rayleigh scattering from distance alone, double-penalising
    // links that Gazebo already flagged as blocked). Fading now lives inside
    // DynamicObstacleLossModel, with m driven by the ray-caster's LoS state
    // (attributes MLos / MNlos / EmaAlpha / Block~ / ClearThresholdDb):
    // Ptr<NakagamiPropagationLossModel> nakagami =
    //     CreateObject<NakagamiPropagationLossModel>();
    // nakagami->SetAttribute("m0", DoubleValue(1.5));
    // nakagami->SetAttribute("m1", DoubleValue(1.0));
    // nakagami->SetAttribute("m2", DoubleValue(1.0));
    // nakagami->SetAttribute("Distance1", DoubleValue(80.0));
    // nakagami->SetAttribute("Distance2", DoubleValue(200.0));

    obstacleLoss->SetNext(logDist);
    // REMOVED: logDist->SetNext(nakagami);

    Ptr<ConstantSpeedPropagationDelayModel> delay =
        CreateObject<ConstantSpeedPropagationDelayModel>();

    Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
    channel->SetPropagationLossModel(obstacleLoss);
    channel->SetPropagationDelayModel(delay);

    // ── WiFi PHY (802.11n, 5 GHz) ──────────────────────────────────────────
    YansWifiPhyHelper phy;
    phy.SetChannel(channel);
    phy.Set("TxPowerStart", DoubleValue(20.0));
    phy.Set("TxPowerEnd",   DoubleValue(20.0));
    phy.Set("RxSensitivity", DoubleValue(-82.0));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    // ── Internet stack + addressing ─────────────────────────────────────────
    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.42.0.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    // ── Initial 3D positions (flat formation) ──────────────────────────────
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> posAlloc =
        CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0.0,        0.0, 10.0));
    posAlloc->Add(Vector(distance,   0.0, 10.0));
    posAlloc->Add(Vector(distance/2, distance * 0.866, 10.0));
    mobility.SetPositionAllocator(posAlloc);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(nodes);

    // ── TapBridge: connect each NS-3 node to its Linux TAP device ──────────
    TapBridgeHelper tapBridge;
    tapBridge.SetAttribute("Mode", StringValue("UseLocal"));

    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        std::string tapName = tapBase + std::to_string(i + 1);
        tapBridge.SetAttribute("DeviceName", StringValue(tapName));
        tapBridge.Install(nodes.Get(i), devices.Get(i));
    }

    // ── ROS2 bridge setup (replaces ZMQ entirely) ───────────────────────────
    rclcpp::init(argc, argv);
    auto rosNode = std::make_shared<Ns3RosNode>(obstacleLoss, nodes);

    // rclcpp::spin() blocks forever, so it must run on its own thread,
    // exactly the same role the old ZmqListenerThread played.
    std::thread rosThread([rosNode]() {
        rclcpp::spin(rosNode);
    });
    // Joined (not detached) after rclcpp::shutdown() below, so the spin thread
    // fully unwinds before Simulator::Destroy() tears down shared state.
    // (detach() would race teardown against a still-running spin thread; join
    // costs nothing at runtime since both run concurrently until shutdown.)

    Simulator::Schedule(Seconds(0.5), &PublishStats,
                        obstacleLoss, Ptr<PropagationLossModel>(logDist),
                        noiseFloor, nodes, rosNode);

    // ── Run ────────────────────────────────────────────────────────────────
    Simulator::Stop(simTime > 0
                    ? Seconds(simTime)
                    : Seconds(3600.0 * 24));

    AsciiTraceHelper ascii;
    phy.EnableAsciiAll(ascii.CreateFileStream("/tmp/ns3_wifi_trace.tr"));
    Simulator::Run();

    g_rosRunning = false;
    rclcpp::shutdown();   // makes rclcpp::spin() return

    rosThread.join();     // wait for the spin thread to finish before teardown

    if (g_csv.is_open()) g_csv.close();

    Simulator::Destroy();
    return 0;
}
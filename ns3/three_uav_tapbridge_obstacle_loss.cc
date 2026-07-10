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

#include "dynamic_obstacle_loss_model.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <thread>
#include <atomic>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThreeUavTapBridge");

static std::atomic<bool> g_rosRunning{true};

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
  }

  // Called by PublishStats() on the NS-3 main thread -- this is fine
  // because rclcpp publishers are themselves thread-safe to call publish() on.
  void PublishRssi(const std::vector<float> & flatData)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = flatData;
    m_rssiPub->publish(msg);
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
};

// ───────────────────────────────────────────────────────────────────────
// RSSI export (same logic as before, now publishes via Ns3RosNode
// instead of zmq_send)
// ───────────────────────────────────────────────────────────────────────
void PublishStats(Ptr<PropagationLossModel> lossChainHead,
                   NodeContainer nodes,
                   std::shared_ptr<Ns3RosNode> rosNode)
{
    std::vector<float> flat;
    for (uint32_t i = 0; i < nodes.GetN(); i++)
    {
        for (uint32_t j = i + 1; j < nodes.GetN(); j++)
        {
            Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel>();
            Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel>();
            if (!mobI || !mobJ) continue;

            double txPowerDbm = 20.0;
            double rxPower = lossChainHead->CalcRxPower(txPowerDbm, mobI, mobJ);

            uint32_t idI = nodes.Get(i)->GetId();
            uint32_t idJ = nodes.Get(j)->GetId();

            // Flat layout: [node_a, node_b, rx_power_dbm, ...]
            flat.push_back(static_cast<float>(idI));
            flat.push_back(static_cast<float>(idJ));
            flat.push_back(static_cast<float>(rxPower));
        }
    }
    rosNode->PublishRssi(flat);

    Simulator::Schedule(Seconds(0.5), &PublishStats, lossChainHead, nodes, rosNode);
}

int main(int argc, char *argv[])
{
    // ── Command-line overrides ──────────────────────────────────────────────
    std::string tapBase  = "tap-uav";
    double      simTime  = 0;
    double      distance = 50.0;

    CommandLine cmd;
    cmd.AddValue("tapBase",  "TAP device name prefix",        tapBase);
    cmd.AddValue("simTime",  "Sim duration (0=unlimited)",    simTime);
    cmd.AddValue("distance", "Initial UAV separation metres", distance);
    cmd.Parse(argc, argv);

    // ── Global config ───────────────────────────────────────────────────────
    GlobalValue::Bind("SimulatorImplementationType",
                      StringValue("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    // ── 3 UAV nodes ────────────────────────────────────────────────────────
    NodeContainer nodes;
    nodes.Create(3);

    // ── Channel: Obstacle loss -> LogDistance + Nakagami fading ─────────────
    Ptr<DynamicObstacleLossModel> obstacleLoss =
        CreateObject<DynamicObstacleLossModel>();

    Ptr<LogDistancePropagationLossModel> logDist =
        CreateObject<LogDistancePropagationLossModel>();
    logDist->SetAttribute("Exponent",         DoubleValue(2.7));
    logDist->SetAttribute("ReferenceDistance", DoubleValue(1.0));
    logDist->SetAttribute("ReferenceLoss",     DoubleValue(46.67));

    Ptr<NakagamiPropagationLossModel> nakagami =
        CreateObject<NakagamiPropagationLossModel>();
    nakagami->SetAttribute("m0", DoubleValue(1.5));
    nakagami->SetAttribute("m1", DoubleValue(1.0));
    nakagami->SetAttribute("m2", DoubleValue(1.0));
    nakagami->SetAttribute("Distance1", DoubleValue(80.0));
    nakagami->SetAttribute("Distance2", DoubleValue(200.0));

    obstacleLoss->SetNext(logDist);
    logDist->SetNext(nakagami);

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
    rosThread.detach();

    Simulator::Schedule(Seconds(0.5), &PublishStats,
                        Ptr<PropagationLossModel>(obstacleLoss), nodes, rosNode);

    // ── Run ────────────────────────────────────────────────────────────────
    Simulator::Stop(simTime > 0
                    ? Seconds(simTime)
                    : Seconds(3600.0 * 24));

    AsciiTraceHelper ascii;
    phy.EnableAsciiAll(ascii.CreateFileStream("/tmp/ns3_wifi_trace.tr"));
    Simulator::Run();

    g_rosRunning = false;
    rclcpp::shutdown();

    Simulator::Destroy();
    return 0;
}
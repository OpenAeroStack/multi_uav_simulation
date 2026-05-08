#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/propagation-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/netanim-module.h"

#include <array>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThreeUavTapBridgeRt");

namespace
{

static void
RestoreNodeColor(AnimationInterface* anim, uint32_t nodeId, uint8_t r, uint8_t g, uint8_t b)
{
  if (!anim)
    {
      return;
    }
  anim->UpdateNodeColor(nodeId, r, g, b);
}

static void
FlashNodeOnPhyRxEnd(AnimationInterface* anim,
                    uint32_t nodeId,
                    uint8_t origR,
                    uint8_t origG,
                    uint8_t origB,
                    Ptr<const Packet> packet)
{
  (void)packet;
  if (!anim)
    {
      return;
    }

  // Flash amber on receive, then restore original color.
  anim->UpdateNodeColor(nodeId, 255, 200, 0);
  Simulator::Schedule(Seconds(0.1), &RestoreNodeColor, anim, nodeId, origR, origG, origB);
}

static void
LogNodePositions(NodeContainer nodes)
{
  const double now = Simulator::Now().GetSeconds();
  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
      Ptr<MobilityModel> mm = nodes.Get(i)->GetObject<MobilityModel>();
      if (!mm)
        {
          continue;
        }
      const Vector p = mm->GetPosition();
      NS_LOG_UNCOND("t=" << now << "s UAV" << (i + 1) << " pos=(" << p.x << "," << p.y << "," << p.z << ")");
    }

  Simulator::Schedule(Seconds(1.0), &LogNodePositions, nodes);
}

class FixedPropagationDelayModel : public PropagationDelayModel
{
public:
  static TypeId GetTypeId()
  {
    static TypeId tid =
      TypeId("ns3::FixedPropagationDelayModel")
        .SetParent<PropagationDelayModel>()
        .SetGroupName("Propagation")
        .AddConstructor<FixedPropagationDelayModel>()
        .AddAttribute("Delay",
                      "Fixed propagation delay applied to all links.",
                      TimeValue(Seconds(0)),
                      MakeTimeAccessor(&FixedPropagationDelayModel::m_delay),
                      MakeTimeChecker());
    return tid;
  }

  FixedPropagationDelayModel() = default;
  ~FixedPropagationDelayModel() override = default;

  Time GetDelay(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override
  {
    (void)a;
    (void)b;
    return m_delay;
  }

protected:
  int64_t DoAssignStreams(int64_t stream) override
  {
    (void)stream;
    return 0;
  }

private:
  Time m_delay{Seconds(0)};
};

NS_OBJECT_ENSURE_REGISTERED(FixedPropagationDelayModel);

} // namespace

int main(int argc, char* argv[])
{
  std::array<std::string, 3> tapNames = {"tap-uav1", "tap-uav2", "tap-uav3"};
  double delayMs = 20.0;
  double lossRate = 0.0;
  std::string dataRate = "50Mbps";
  double simDurationSec = 0.0;
  double uavSpeed = 5.0;
  bool enableFlowMonitor = true;
  std::string flowmonXml = "three_uav_flowmon.xml";
  std::string animFile = "three_uav_anim.xml";
  std::string tapMode = "UseLocal";

  CommandLine cmd(__FILE__);
  cmd.AddValue("tap1", "TAP name for UAV1", tapNames[0]);
  cmd.AddValue("tap2", "TAP name for UAV2", tapNames[1]);
  cmd.AddValue("tap3", "TAP name for UAV3", tapNames[2]);
  cmd.AddValue("delayMs", "WiFi propagation delay in milliseconds (fixed)", delayMs);
  cmd.AddValue("lossRate", "Per-device receive packet loss [0.0-1.0]", lossRate);
  cmd.AddValue("dataRate", "(Unused) legacy parameter retained for compatibility", dataRate);
  cmd.AddValue("simDurationSec", "Stop simulation after N seconds (0 = run forever)", simDurationSec);
  cmd.AddValue("uavSpeed", "UAV waypoint speed in m/s (used when simDurationSec > 0)", uavSpeed);
  cmd.AddValue("enableFlowMonitor", "Enable FlowMonitor statistics collection", enableFlowMonitor);
  cmd.AddValue("flowmonXml", "FlowMonitor XML output path", flowmonXml);
  cmd.AddValue("animFile", "NetAnim XML output path", animFile);
  cmd.AddValue("tapMode",
               "TapBridge mode: UseLocal (default, works with WiFi) or UseBridge (requires SupportsSendFrom)",
               tapMode);
  cmd.Parse(argc, argv);

  GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

  NodeContainer nodes;
  nodes.Create(3);

  // 3D mobility for Friis propagation loss.
  // If simDurationSec==0 (run forever), use constant positions as a safe fallback.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator>();
  const Vector uav1Start(0.0, 0.0, 20.0);
  const Vector uav2Start(50.0, 0.0, 20.0);
  const Vector uav3Start(-50.0, 0.0, 20.0);
  positions->Add(uav1Start);
  positions->Add(uav2Start);
  positions->Add(uav3Start);
  mobility.SetPositionAllocator(positions);

  const bool useWaypoints = (simDurationSec > 0.0);
  if (useWaypoints)
    {
      mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    }
  else
    {
      mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    }
  mobility.Install(nodes);

  if (useWaypoints)
    {
      if (uavSpeed <= 0.0)
        {
          NS_LOG_UNCOND("Invalid uavSpeed=" << uavSpeed << " (must be > 0); using 5.0 m/s");
          uavSpeed = 5.0;
        }

      // Simple back-and-forth along +Y/-Y with 50m amplitude.
      const double legDistanceM = 50.0;
      const double legTimeSec = legDistanceM / uavSpeed;

      std::array<Vector, 3> startPos = {uav1Start, uav2Start, uav3Start};
      std::array<Vector, 3> endPos = {
        Vector(uav1Start.x, uav1Start.y + legDistanceM, uav1Start.z),
        Vector(uav2Start.x, uav2Start.y + legDistanceM, uav2Start.z),
        Vector(uav3Start.x, uav3Start.y + legDistanceM, uav3Start.z)};

      for (uint32_t i = 0; i < nodes.GetN(); ++i)
        {
          Ptr<WaypointMobilityModel> wmm = nodes.Get(i)->GetObject<WaypointMobilityModel>();
          if (!wmm)
            {
              continue;
            }

          double t = 0.0;
          bool toEnd = true;
          wmm->AddWaypoint(Waypoint(Seconds(t), startPos[i]));
          while (t + legTimeSec <= simDurationSec)
            {
              t += legTimeSec;
              wmm->AddWaypoint(Waypoint(Seconds(t), toEnd ? endPos[i] : startPos[i]));
              toEnd = !toEnd;
            }
        }
    }

  // 802.11n ad-hoc WiFi with Friis loss and a fixed propagation delay mapped from --delayMs.
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211n);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                               "DataMode",
                               StringValue("HtMcs7"),
                               "ControlMode",
                               StringValue("HtMcs0"));

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  channel.SetPropagationDelay("ns3::FixedPropagationDelayModel",
                              "Delay",
                              TimeValue(MilliSeconds(delayMs)));
  channel.AddPropagationLoss("ns3::FriisPropagationLossModel");

  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());

  WifiMacHelper mac;
  mac.SetType("ns3::AdhocWifiMac");

  NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

  if (lossRate > 0.0)
    {
      for (uint32_t i = 0; i < devices.GetN(); ++i)
        {
          Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
          em->SetAttribute("ErrorRate", DoubleValue(lossRate));
          devices.Get(i)->SetAttribute("ReceiveErrorModel", PointerValue(em));
        }
    }

  InternetStackHelper internet;
  internet.Install(nodes);

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;
  if (enableFlowMonitor)
    {
      monitor = flowmon.InstallAll();
    }

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.250.0.0", "255.255.255.0");
  Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

  for (uint32_t i = 0; i < 3; ++i)
    {
      TapBridgeHelper tap;
      tap.SetAttribute("Mode", StringValue(tapMode));
      tap.SetAttribute("DeviceName", StringValue(tapNames[i]));
      tap.Install(nodes.Get(i), devices.Get(i));

      NS_LOG_UNCOND("UAV" << i + 1 << " TAP=" << tapNames[i]
                           << " ns3-ip=" << ifaces.GetAddress(i));
    }

  if (simDurationSec > 0.0)
    {
      Simulator::Stop(Seconds(simDurationSec));
    }

  NS_LOG_UNCOND("Starting NS-3 real-time TapBridge simulation");
  NS_LOG_UNCOND("Config: delayMs=" << delayMs << " lossRate=" << lossRate
                                    << " dataRate=" << dataRate);

  // Log 1Hz position updates.
  Simulator::Schedule(Seconds(1.0), &LogNodePositions, nodes);

  // NetAnim configuration (must be constructed before Simulator::Run()).
  AnimationInterface anim(animFile);
  anim.EnablePacketMetadata(true);

  // Node labels: include TAP name and MAVLink TCP port.
  const std::array<uint16_t, 3> mavlinkPorts = {5760, 5770, 5780};
  anim.UpdateNodeDescription(nodes.Get(0), "UAV1 " + tapNames[0] + " tcp:" + std::to_string(mavlinkPorts[0]));
  anim.UpdateNodeDescription(nodes.Get(1), "UAV2 " + tapNames[1] + " tcp:" + std::to_string(mavlinkPorts[1]));
  anim.UpdateNodeDescription(nodes.Get(2), "UAV3 " + tapNames[2] + " tcp:" + std::to_string(mavlinkPorts[2]));

  // Node base colors and sizes.
  const std::array<std::array<uint8_t, 3>, 3> baseColors = {{{255, 0, 0}, {0, 255, 0}, {0, 0, 255}}};
  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
      anim.UpdateNodeColor(nodes.Get(i), baseColors[i][0], baseColors[i][1], baseColors[i][2]);
      anim.UpdateNodeSize(nodes.Get(i), 5.0, 5.0);
    }

  // Link labels (for visualization): mark links with configured dataRate.
  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
      for (uint32_t j = i + 1; j < nodes.GetN(); ++j)
        {
          anim.UpdateLinkDescription(nodes.Get(i), nodes.Get(j), dataRate);
          anim.UpdateLinkDescription(nodes.Get(j), nodes.Get(i), dataRate);
        }
    }

  // Flash nodes on WiFi packet receipt using the PHY RxEnd trace.
  for (uint32_t i = 0; i < devices.GetN() && i < nodes.GetN(); ++i)
    {
      Ptr<WifiNetDevice> wnd = DynamicCast<WifiNetDevice>(devices.Get(i));
      if (!wnd || !wnd->GetPhy())
        {
          continue;
        }
      wnd->GetPhy()->TraceConnectWithoutContext(
        "PhyRxEnd",
        MakeBoundCallback(&FlashNodeOnPhyRxEnd,
                          &anim,
                          nodes.Get(i)->GetId(),
                          baseColors[i][0],
                          baseColors[i][1],
                          baseColors[i][2]));
    }

  Simulator::Run();

  if (enableFlowMonitor && monitor)
    {
      monitor->CheckForLostPackets();

      const auto stats = monitor->GetFlowStats();
      uint64_t totalTxPackets = 0;
      uint64_t totalRxPackets = 0;
      uint64_t totalLostPackets = 0;
      uint64_t totalRxBytes = 0;
      Time delaySum = Seconds(0);
      Time jitterSum = Seconds(0);

      for (const auto& entry : stats)
        {
          const FlowMonitor::FlowStats& s = entry.second;
          totalTxPackets += s.txPackets;
          totalRxPackets += s.rxPackets;
          totalLostPackets += s.lostPackets;
          totalRxBytes += s.rxBytes;
          delaySum += s.delaySum;
          jitterSum += s.jitterSum;
        }

      const double durationSec = (simDurationSec > 0.0) ? simDurationSec : Simulator::Now().GetSeconds();
      const double throughputMbps = (durationSec > 0.0)
                                      ? (static_cast<double>(totalRxBytes) * 8.0) / (durationSec * 1e6)
                                      : 0.0;
      const double avgDelayMs = (totalRxPackets > 0)
                                  ? (delaySum.GetSeconds() * 1000.0) / static_cast<double>(totalRxPackets)
                                  : 0.0;
      const double avgJitterMs = (totalRxPackets > 1)
                                   ? (jitterSum.GetSeconds() * 1000.0) / static_cast<double>(totalRxPackets - 1)
                                   : 0.0;
      const double lossPct = (totalTxPackets > 0)
                               ? (100.0 * static_cast<double>(totalLostPackets) / static_cast<double>(totalTxPackets))
                               : 0.0;

      NS_LOG_UNCOND("FlowMonitor summary:");
      NS_LOG_UNCOND("  txPackets=" << totalTxPackets << " rxPackets=" << totalRxPackets
                                     << " lostPackets=" << totalLostPackets);
      NS_LOG_UNCOND("  throughputMbps=" << throughputMbps
                                         << " avgDelayMs=" << avgDelayMs
                                         << " avgJitterMs=" << avgJitterMs
                                         << " lossPct=" << lossPct);

      monitor->SerializeToXmlFile(flowmonXml, true, true);
      NS_LOG_UNCOND("FlowMonitor XML written to: " << flowmonXml);
    }

  Simulator::Destroy();
  return 0;
}

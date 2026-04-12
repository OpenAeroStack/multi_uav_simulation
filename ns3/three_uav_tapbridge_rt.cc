#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/csma-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"

#include <array>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThreeUavTapBridgeRt");

int main(int argc, char* argv[])
{
  std::array<std::string, 3> tapNames = {"tap-uav1", "tap-uav2", "tap-uav3"};
  double delayMs = 20.0;
  double lossRate = 0.0;
  std::string dataRate = "50Mbps";
  double simDurationSec = 0.0;
  bool enableFlowMonitor = true;
  std::string flowmonXml = "three_uav_flowmon.xml";

  CommandLine cmd(__FILE__);
  cmd.AddValue("tap1", "TAP name for UAV1", tapNames[0]);
  cmd.AddValue("tap2", "TAP name for UAV2", tapNames[1]);
  cmd.AddValue("tap3", "TAP name for UAV3", tapNames[2]);
  cmd.AddValue("delayMs", "CSMA channel delay in milliseconds", delayMs);
  cmd.AddValue("lossRate", "Per-device receive packet loss [0.0-1.0]", lossRate);
  cmd.AddValue("dataRate", "CSMA data rate", dataRate);
  cmd.AddValue("simDurationSec", "Stop simulation after N seconds (0 = run forever)", simDurationSec);
  cmd.AddValue("enableFlowMonitor", "Enable FlowMonitor statistics collection", enableFlowMonitor);
  cmd.AddValue("flowmonXml", "FlowMonitor XML output path", flowmonXml);
  cmd.Parse(argc, argv);

  GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

  NodeContainer nodes;
  nodes.Create(3);

  CsmaHelper csma;
  csma.SetChannelAttribute("DataRate", StringValue(dataRate));
  csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(delayMs)));

  NetDeviceContainer devices = csma.Install(nodes);

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
      tap.SetAttribute("Mode", StringValue("UseBridge"));
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

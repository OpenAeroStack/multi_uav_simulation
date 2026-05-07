#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/csma-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/error-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-module.h"

#include <thread>
#include <mutex>
#include <map>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThreeUavWifiTapBridgeRt");

// --- 1. SHARED GLOBALS ---
std::map<std::pair<uint32_t, uint32_t>, double> g_linkPenalties;
std::mutex g_penaltyMutex;
bool g_simulationRunning = true;

// NEW: Make the UAV NodeContainer global so the background thread can access it
NodeContainer g_uavNodes; 


// --- 2. THE BACKGROUND UDP LISTENER THREAD ---
void UdpListenerThread() 
{
  int sockfd;
  struct sockaddr_in servaddr;
  
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      NS_LOG_ERROR("Socket creation failed");
      return;
  }
  
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(5555);
  
  if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
      NS_LOG_ERROR("Socket bind failed");
      return;
  }

  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  char buffer[1024];
  while (g_simulationRunning) {
      int n = recvfrom(sockfd, (char *)buffer, 1024, 0, NULL, NULL);
      if (n > 0) {
          buffer[n] = '\0';
          std::string msg(buffer);
          std::stringstream ss(msg);
          std::string msgType;
          
          // Read the first word to see if it's a PENALTY or POSE update
          ss >> msgType;

          if (msgType == "PENALTY") {
              uint32_t tx, rx;
              double penalty;
              if (ss >> tx >> rx >> penalty) {
                  std::lock_guard<std::mutex> lock(g_penaltyMutex);
                  g_linkPenalties[{tx, rx}] = penalty;
              }
          } 
          // NEW: Handle incoming location updates
          else if (msgType == "POSE") {
              uint32_t uavIndex;
              double x, y, z;
              if (ss >> uavIndex >> x >> y >> z) {
                  // Ensure index is valid (0, 1, or 2 for our 3 UAVs)
                  if (uavIndex < g_uavNodes.GetN()) {
                      Ptr<Node> targetNode = g_uavNodes.Get(uavIndex); 
                      Ptr<MobilityModel> mob = targetNode->GetObject<MobilityModel>();
                      
                      if (mob) {
                          // NEW: Safely schedule the position update in the NS-3 simulation loop
                          Simulator::ScheduleWithContext(targetNode->GetId(), Seconds(0.0), 
                              &MobilityModel::SetPosition, mob, Vector(x, y, z));
                      }
                  }
              }
          }
      }
  }
  close(sockfd);
}

// --- 3. CUSTOM OBSTACLE LOSS MODEL ---
class UdpObstacleLossModel : public PropagationLossModel
{
public:
  static TypeId GetTypeId (void) {
    static TypeId tid = TypeId ("ns3::UdpObstacleLossModel")
      .SetParent<PropagationLossModel> ()
      .SetGroupName ("Wifi")
      .AddConstructor<UdpObstacleLossModel> ();
    return tid;
  }
  UdpObstacleLossModel() {}

private:
  virtual double DoCalcRxPower (double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override {
    uint32_t txId = a->GetObject<Node>()->GetId();
    uint32_t rxId = b->GetObject<Node>()->GetId();

    double penalty = 0.0;
    std::pair<uint32_t, uint32_t> link = {txId, rxId};

    {
        std::lock_guard<std::mutex> lock(g_penaltyMutex);
        if (g_linkPenalties.find(link) != g_linkPenalties.end()) {
            penalty = g_linkPenalties[link];
        }
    }
    return txPowerDbm - penalty;
  }
  virtual int64_t DoAssignStreams (int64_t stream) override { return 0; }
};


// --- 4. MAIN SCRIPT ---
int main(int argc, char* argv[])
{
  std::thread listenerThread(UdpListenerThread);

  std::array<std::string, 3> tapNames = {"tap-uav1", "tap-uav2", "tap-uav3"};
  double simDurationSec = 0.0;
  bool enableFlowMonitor = true;
  std::string flowmonXml = "three_uav_wifi_ros_flowmon.xml";

  CommandLine cmd(__FILE__);
  cmd.Parse(argc, argv);

  GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

  NodeContainer proxyNodes; 
  proxyNodes.Create(3);
  
  // NEW: Initialize the global UAV node container instead of a local one
  g_uavNodes.Create(3);

  YansWifiChannelHelper channelHelper;
  channelHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  Ptr<YansWifiChannel> channel = channelHelper.Create();

  Ptr<LogDistancePropagationLossModel> logLoss = CreateObject<LogDistancePropagationLossModel>();
  Ptr<UdpObstacleLossModel> udpLoss = CreateObject<UdpObstacleLossModel>(); 
  
  logLoss->SetNext(udpLoss);
  channel->SetPropagationLossModel(logLoss);

  YansWifiPhyHelper phy;
  phy.SetChannel(channel);

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211g);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                               "DataMode", StringValue("ErpOfdmRate54Mbps"),
                               "ControlMode", StringValue("ErpOfdmRate24Mbps"));

  WifiMacHelper mac;
  mac.SetType("ns3::AdhocWifiMac");
  // Use the global node container here
  NetDeviceContainer uavWifiDevices = wifi.Install(phy, mac, g_uavNodes);

  MobilityHelper mobility;
  // Starting positions don't matter as much now, as Gazebo will overwrite them immediately
  mobility.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY", DoubleValue(0.0), "DeltaX", DoubleValue(10.0), "DeltaY", DoubleValue(10.0), "GridWidth", UintegerValue(3), "LayoutType", StringValue("RowFirst"));
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(g_uavNodes);

  OlsrHelper olsr;
  InternetStackHelper internet;
  internet.SetRoutingHelper(olsr); 
  internet.Install(proxyNodes);
  internet.Install(g_uavNodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.250.0.0", "255.255.255.0");
  Ipv4InterfaceContainer uavWifiIfaces = ipv4.Assign(uavWifiDevices);

  CsmaHelper csmaTether;
  csmaTether.SetChannelAttribute("DataRate", StringValue("1000Mbps")); 
  csmaTether.SetChannelAttribute("Delay", TimeValue(NanoSeconds(1)));

  for (uint32_t i = 0; i < 3; ++i) {
      NodeContainer linkNodes(proxyNodes.Get(i), g_uavNodes.Get(i));
      NetDeviceContainer linkDevs = csmaTether.Install(linkNodes);

      std::ostringstream subnet;
      subnet << "10.1." << i + 1 << ".0";
      ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
      Ipv4InterfaceContainer linkIfaces = ipv4.Assign(linkDevs);

      TapBridgeHelper tap;
      tap.SetAttribute("Mode", StringValue("ConfigureLocal")); 
      tap.SetAttribute("DeviceName", StringValue(tapNames[i]));
      tap.Install(proxyNodes.Get(i), linkDevs.Get(0));
  }

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;
  if (enableFlowMonitor) monitor = flowmon.InstallAll();

  if (simDurationSec > 0.0) Simulator::Stop(Seconds(simDurationSec));

  NS_LOG_UNCOND("Starting NS-3 Real-time Simulation (Listening on UDP 5555)...");
  Simulator::Run();

  if (enableFlowMonitor && monitor) {
      monitor->CheckForLostPackets();
      monitor->SerializeToXmlFile(flowmonXml, true, true);
  }

  Simulator::Destroy();

  g_simulationRunning = false;
  if (listenerThread.joinable()) {
      listenerThread.join();
  }

  return 0;
}
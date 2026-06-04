/*
 * three_uav_tapbridge_rt.cc
 *
 * Real-time NS-3 scenario for multi_uav_sim.
 * Bridges three network namespaces (uav1ns/uav2ns/uav3ns) into a simulated
 * 802.11n ad-hoc channel with Nakagami fading + log-distance path loss.
 *
 * NS-3 version: 3.38.1  Ubuntu: 22.04
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/propagation-module.h"
#include "ns3/internet-module.h"
#include "ns3/stats-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThreeUavTapBridge");

void PublishStats(Ptr<YansWifiChannel> channel, NodeContainer nodes, void* zmqPub)
{
    // For each pair, get the loss model output
    for (uint32_t i = 0; i < nodes.GetN(); i++) {
        for (uint32_t j = i+1; j < nodes.GetN(); j++) {
            Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel>();
            Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel>();
            
            double txPowerDbm = 20.0;
            // Walk the loss model chain
            double rxPower = logDist->CalcRxPower(txPowerDbm, mobI, mobJ);
            
            // Publish: "RSSI i j rxPower"
            std::string msg = "RSSI " + std::to_string(i+1) + " " + 
                              std::to_string(j+1) + " " + std::to_string(rxPower);
            zmq_send(zmqPub, msg.c_str(), msg.size(), 0);
        }
    }
    Simulator::Schedule(Seconds(0.5), &PublishStats, channel, nodes, zmqPub);
}

int main(int argc, char *argv[])
{
    // ── Command-line overrides ──────────────────────────────────────────────
    std::string tapBase  = "tap-uav";   // tap-uav1, tap-uav2, tap-uav3
    double      simTime  = 0;           // 0 = run until Ctrl-C
    double      distance = 50.0;        // initial UAV separation (metres)

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

    // ── Channel: LogDistance + Nakagami fading ──────────────────────────────
    // LogDistance gives large-scale path loss; Nakagami adds small-scale
    // multipath fading (m=1 → Rayleigh, m=3 → mild Rician-like).
    Ptr<LogDistancePropagationLossModel> logDist =
        CreateObject<LogDistancePropagationLossModel>();
    logDist->SetAttribute("Exponent",         DoubleValue(2.7));
    logDist->SetAttribute("ReferenceDistance", DoubleValue(1.0));
    logDist->SetAttribute("ReferenceLoss",     DoubleValue(46.67));

    Ptr<NakagamiPropagationLossModel> nakagami =
        CreateObject<NakagamiPropagationLossModel>();
    // m0: distances < d1  (m=1.5 → moderate fading)
    nakagami->SetAttribute("m0", DoubleValue(1.5));
    // m1: d1 <= distance < d2  (m=1 → Rayleigh, worst-case urban)
    nakagami->SetAttribute("m1", DoubleValue(1.0));
    // m2: distances >= d2  (m=1 → Rayleigh, long-range)
    nakagami->SetAttribute("m2", DoubleValue(1.0));
    nakagami->SetAttribute("Distance1", DoubleValue(80.0));
    nakagami->SetAttribute("Distance2", DoubleValue(200.0));

    // Chain: signal passes through logDist first, then Nakagami fades it
    logDist->SetNext(nakagami);

    Ptr<ConstantSpeedPropagationDelayModel> delay =
        CreateObject<ConstantSpeedPropagationDelayModel>();

    Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
    channel->SetPropagationLossModel(logDist);
    channel->SetPropagationDelayModel(delay);

    // ── WiFi PHY (802.11n, 5 GHz) ──────────────────────────────────────────
    YansWifiPhyHelper phy;
    phy.SetChannel(channel);
    phy.Set("TxPowerStart", DoubleValue(20.0));   // dBm
    phy.Set("TxPowerEnd",   DoubleValue(20.0));
    phy.Set("RxSensitivity", DoubleValue(-82.0)); // dBm

    // ── WiFi MAC — ad-hoc ──────────────────────────────────────────────────
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
    posAlloc->Add(Vector(0.0,        0.0, 10.0));  // UAV1
    posAlloc->Add(Vector(distance,   0.0, 10.0));  // UAV2
    posAlloc->Add(Vector(distance/2, distance * 0.866, 10.0)); // UAV3 (equilateral)
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

    // ── Run ────────────────────────────────────────────────────────────────
    Simulator::Stop(simTime > 0
                    ? Seconds(simTime)
                    : Seconds(3600.0 * 24)); // 24h ceiling if "unlimited"
    
    AsciiTraceHelper ascii;
    phy.EnableAsciiAll(ascii.CreateFileStream("/tmp/ns3_wifi_trace.tr"));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
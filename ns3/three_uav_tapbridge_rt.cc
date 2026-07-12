// =============================================================================
//  three_uav_tapbridge_rt.cc
//
//  Three-UAV TapBridge simulation with realistic channel modelling:
//    - Log-distance path loss     (primary path loss)
//    - Nakagami-m fading overlay  (multipath / LOS richness)
//    - Log-normal shadowing       (environment clutter)
//    - ConstantRate OfdmRate54Mbps (fixed rate, crash-safe)
//    - Gauss-Markov mobility      (realistic UAV flight dynamics)
//    - Per-link SNR logging       (for verification / plotting)
//    - Extended FlowMonitor stats (throughput, delay, jitter, loss)
//
//  Gazebo / ArduPilot / micro-ROS integration is unchanged – the TapBridge
//  and IP addressing are identical to the original script.
//
//  Build:
//    cp three_uav_tapbridge_rt.cc <ns3-root>/scratch/
//    cd <ns3-root>
//    ./ns3 build
//
//  Run (basic):
//    ./ns3 run "scratch/three_uav_tapbridge_rt --simDurationSec=60"
//
//  Run (full options):
//    ./ns3 run "scratch/three_uav_tapbridge_rt 
//      --simDurationSec=120 
//      --delayMs=20 
//      --lossRate=0.05 
//      --nakagamiM=1.5 
//      --shadowingStdDb=4.0 
//      --txPowerDbm=20 
//      --uavAltitude=20 
//      --enableFlowMonitor=true 
//      --snrLogFile=snr_log.csv"
//
// =============================================================================

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
#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThreeUavRealistic");

// ─────────────────────────────────────────────────────────────────────────────
//  Custom fixed-delay model (kept from original)
// ─────────────────────────────────────────────────────────────────────────────
namespace
{

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
        .AddAttribute("Delay", "Fixed propagation delay.",
                      TimeValue(Seconds(0)),
                      MakeTimeAccessor(&FixedPropagationDelayModel::m_delay),
                      MakeTimeChecker());
    return tid;
  }
  FixedPropagationDelayModel() = default;
  ~FixedPropagationDelayModel() override = default;
  Time GetDelay(Ptr<MobilityModel>, Ptr<MobilityModel>) const override { return m_delay; }
protected:
  int64_t DoAssignStreams(int64_t s) override { return 0; }
private:
  Time m_delay{Seconds(0)};
};
NS_OBJECT_ENSURE_REGISTERED(FixedPropagationDelayModel);

// ─────────────────────────────────────────────────────────────────────────────
//  SNR / RSSI logger
//  Hooks into WifiPhy MonitorSnifferRx trace to record per-packet SNR.
//  Output: CSV with columns  time_s, rx_node, tx_addr, snr_db, rssi_dbm
// ─────────────────────────────────────────────────────────────────────────────
static std::ofstream g_snrFile;

static void
MonitorSnifferCallback(std::string context,
                       Ptr<const Packet> packet,
                       uint16_t channelFreqMhz,
                       WifiTxVector txVector,
                       MpduInfo aMpdu,
                       SignalNoiseDbm signalNoise,
                       uint16_t staId)
{
  if (!g_snrFile.is_open())
    return;

  // context is "/NodeList/N/DeviceList/0/$ns3::WifiNetDevice/Phy/MonitorSnifferRx"
  // extract node index
  uint32_t nodeId = 0;
  std::size_t pos = context.find("/NodeList/");
  if (pos != std::string::npos)
    {
      std::sscanf(context.c_str() + pos + 10, "%u", &nodeId);
    }

  const double snrDb = signalNoise.signal - signalNoise.noise;
  g_snrFile << std::fixed << std::setprecision(4)
            << Simulator::Now().GetSeconds() << ","
            << nodeId << ","
            << signalNoise.signal << ","
            << signalNoise.noise  << ","
            << snrDb              << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
//  Periodic position logger
// ─────────────────────────────────────────────────────────────────────────────
static void
LogNodePositions(NodeContainer nodes)
{
  const double now = Simulator::Now().GetSeconds();
  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
      Ptr<MobilityModel> mm = nodes.Get(i)->GetObject<MobilityModel>();
      if (!mm) continue;
      const Vector p = mm->GetPosition();
      // Also compute inter-UAV distances for channel verification
      if (i == 0 && nodes.GetN() >= 2)
        {
          Ptr<MobilityModel> mm2 = nodes.Get(1)->GetObject<MobilityModel>();
          Ptr<MobilityModel> mm3 = nodes.Get(2)->GetObject<MobilityModel>();
          if (mm2 && mm3)
            {
              const double d12 = mm->GetDistanceFrom(mm2);
              const double d13 = mm->GetDistanceFrom(mm3);
              const double d23 = mm2->GetDistanceFrom(mm3);
              NS_LOG_UNCOND("t=" << now
                << "s  d(UAV1-UAV2)=" << std::fixed << std::setprecision(1) << d12
                << "m  d(UAV1-UAV3)=" << d13
                << "m  d(UAV2-UAV3)=" << d23 << "m");
            }
        }
      NS_LOG_UNCOND("  UAV" << (i+1)
        << " pos=(" << p.x << "," << p.y << "," << p.z << ")");
    }
  Simulator::Schedule(Seconds(1.0), &LogNodePositions, nodes);
}

// ─────────────────────────────────────────────────────────────────────────────
//  NetAnim helpers (unchanged from original)
// ─────────────────────────────────────────────────────────────────────────────
static void
RestoreNodeColor(AnimationInterface* anim, uint32_t nodeId,
                 uint8_t r, uint8_t g, uint8_t b)
{
  if (anim) anim->UpdateNodeColor(nodeId, r, g, b);
}

static void
FlashNodeOnPhyRxEnd(AnimationInterface* anim,
                    uint32_t nodeId,
                    uint8_t origR, uint8_t origG, uint8_t origB,
                    Ptr<const Packet> packet)
{
  (void)packet;
  if (!anim) return;
  anim->UpdateNodeColor(nodeId, 255, 200, 0);
  Simulator::Schedule(Seconds(0.1), &RestoreNodeColor,
                      anim, nodeId, origR, origG, origB);
}

} // anonymous namespace

// =============================================================================
//  main()
// =============================================================================
int main(int argc, char* argv[])
{
  // ── Command-line parameters ──────────────────────────────────────────────
  std::array<std::string, 4> tapNames = {"tap-gcs", "tap-uav1", "tap-uav2", "tap-uav3"};
  double delayMs          = 20.0;   // propagation delay (ms)
  double lossRate         = 0.0;    // extra burst loss on top of channel models
  double simDurationSec   = 60.0;   // 0 = run forever (real-time)
  double uavSpeed         = 5.0;    // m/s for waypoint legs
  double uavAltitude      = 20.0;   // m AGL — drives Two-ray model
  double nakagamiM        = 1.5;    // Nakagami-m: 1.0=Rayleigh, >1 = more LOS
  double shadowingStdDb   = 4.0;    // log-normal shadow standard deviation (dB)
  double txPowerDbm       = 20.0;   // Tx power in dBm
  bool   enableGaussMarkov = true;  // Gauss-Markov (true) vs Waypoint (false)
  bool   enableFlowMonitor = true;
  bool   enableSnrLog      = true;
  std::string flowmonXml  = "three_uav_flowmon.xml";
  std::string animFile    = "three_uav_anim.xml";
  std::string snrLogFile  = "snr_log.csv";

  CommandLine cmd(__FILE__);
  cmd.AddValue("tap0",            "TAP name for GCS",                     tapNames[0]);
  cmd.AddValue("tap1",            "TAP name for UAV1",                    tapNames[1]);
  cmd.AddValue("tap2",            "TAP name for UAV2",                    tapNames[2]);
  cmd.AddValue("tap3",            "TAP name for UAV3",                    tapNames[3]);
  cmd.AddValue("delayMs",         "Propagation delay (ms)",               delayMs);
  cmd.AddValue("lossRate",        "Extra burst error rate [0-1]",         lossRate);
  cmd.AddValue("simDurationSec",  "Sim duration (0=forever)",             simDurationSec);
  cmd.AddValue("uavSpeed",        "UAV speed m/s",                        uavSpeed);
  cmd.AddValue("uavAltitude",     "UAV altitude m (Two-ray height)",      uavAltitude);
  cmd.AddValue("nakagamiM",       "Nakagami-m fading factor (>=0.5)",     nakagamiM);
  cmd.AddValue("shadowingStdDb",  "Log-normal shadow std dev (dB)",       shadowingStdDb);
  cmd.AddValue("txPowerDbm",      "Transmit power in dBm",                txPowerDbm);
  cmd.AddValue("enableGaussMarkov","Use Gauss-Markov mobility model",     enableGaussMarkov);
  cmd.AddValue("enableFlowMonitor","Enable FlowMonitor",                  enableFlowMonitor);
  cmd.AddValue("enableSnrLog",    "Log SNR per packet to CSV",            enableSnrLog);
  cmd.AddValue("flowmonXml",      "FlowMonitor XML output",               flowmonXml);
  cmd.AddValue("animFile",        "NetAnim XML output",                   animFile);
  cmd.AddValue("snrLogFile",      "SNR CSV output path",                  snrLogFile);
  cmd.Parse(argc, argv);

  // Clamp Nakagami-m to valid range
  if (nakagamiM < 0.5) { NS_LOG_WARN("nakagamiM clamped to 0.5"); nakagamiM = 0.5; }

  // ── Real-time simulator + checksum ──────────────────────────────────────
  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

  // ── Nodes ────────────────────────────────────────────────────────────────
  // Node 0 = GCS (ground, stationary)   IP: 10.42.0.10   TAP: tap-gcs
  // Node 1 = UAV1                        IP: 10.42.0.11   TAP: tap-uav1
  // Node 2 = UAV2                        IP: 10.42.0.12   TAP: tap-uav2
  // Node 3 = UAV3                        IP: 10.42.0.13   TAP: tap-uav3
  NodeContainer nodes;
  nodes.Create(4);

  // ── Mobility ─────────────────────────────────────────────────────────────
  //  GCS (node 0): ConstantPositionMobilityModel at ground (0, 0, 0).
  //  UAVs (nodes 1-3): Gauss-Markov or Waypoint as before.

  // Install stationary mobility on GCS node first
  MobilityHelper gcsFixed;
  Ptr<ListPositionAllocator> gcsPos = CreateObject<ListPositionAllocator>();
  gcsPos->Add(Vector(0.0, 0.0, 0.0));  // ground level
  gcsFixed.SetPositionAllocator(gcsPos);
  gcsFixed.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  gcsFixed.Install(nodes.Get(0));

  // UAV nodes container (nodes 1-3)
  NodeContainer uavNodes;
  uavNodes.Add(nodes.Get(1));
  uavNodes.Add(nodes.Get(2));
  uavNodes.Add(nodes.Get(3));

  // ── Mobility: Gauss-Markov or Waypoint (UAVs only) ───────────────────────
  //
  //  Gauss-Markov is the recommended model for UAVs: it produces smooth,
  //  correlated trajectories that mimic ArduPilot flight dynamics far better
  //  than abrupt waypoint transitions. Alpha=0.85 gives moderate persistence.
  //
  MobilityHelper mobility;

  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator>();
  positions->Add(Vector(  0.0, 0.0, uavAltitude));  // UAV1
  positions->Add(Vector( 50.0, 0.0, uavAltitude));  // UAV2
  positions->Add(Vector(-50.0, 0.0, uavAltitude));  // UAV3
  mobility.SetPositionAllocator(positions);

  if (enableGaussMarkov)
    {
      // Bounding box 400 x 400 x 10 m centred at origin, at UAV altitude
      const double halfXY = 200.0;
      const double altMin = std::max(1.0, uavAltitude - 5.0);
      const double altMax = uavAltitude + 5.0;

      mobility.SetMobilityModel(
        "ns3::GaussMarkovMobilityModel",
        "Bounds",    BoxValue(Box(-halfXY, halfXY, -halfXY, halfXY, altMin, altMax)),
        "TimeStep",  TimeValue(Seconds(0.5)),   // update interval
        "Alpha",     DoubleValue(0.85),          // velocity correlation (0=random, 1=straight)
        "MeanVelocity",
          StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(uavSpeed*0.8)
                      + "|Max=" + std::to_string(uavSpeed*1.2) + "]"),
        "MeanDirection",
          StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185]"),
        "MeanPitch",
          StringValue("ns3::UniformRandomVariable[Min=-0.05|Max=0.05]"),
        "NormalVelocity",
          StringValue("ns3::NormalRandomVariable[Mean=0|Variance=0.5|Bound=1]"),
        "NormalDirection",
          StringValue("ns3::NormalRandomVariable[Mean=0|Variance=0.2|Bound=0.5]"),
        "NormalPitch",
          StringValue("ns3::NormalRandomVariable[Mean=0|Variance=0.01|Bound=0.02]"));
    }
  else
    {
      // Fallback: simple waypoint back-and-forth (original behaviour)
      mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    }

  mobility.Install(uavNodes);

  // Add waypoints for UAV nodes when NOT using Gauss-Markov
  if (!enableGaussMarkov && simDurationSec > 0.0)
    {
      if (uavSpeed <= 0.0) uavSpeed = 5.0;
      const double legDist = 50.0;
      const double legTime = legDist / uavSpeed;
      std::array<Vector,3> sp = {
        Vector(  0.0, 0.0, uavAltitude),
        Vector( 50.0, 0.0, uavAltitude),
        Vector(-50.0, 0.0, uavAltitude)};
      std::array<Vector,3> ep = {
        Vector(  0.0, legDist, uavAltitude),
        Vector( 50.0, legDist, uavAltitude),
        Vector(-50.0, legDist, uavAltitude)};
      for (uint32_t i = 0; i < uavNodes.GetN(); ++i)
        {
          Ptr<WaypointMobilityModel> wmm =
            uavNodes.Get(i)->GetObject<WaypointMobilityModel>();
          if (!wmm) continue;
          double t = 0.0; bool toEnd = true;
          wmm->AddWaypoint(Waypoint(Seconds(t), sp[i]));
          while (t + legTime <= simDurationSec)
            {
              t += legTime;
              wmm->AddWaypoint(Waypoint(Seconds(t), toEnd ? ep[i] : sp[i]));
              toEnd = !toEnd;
            }
        }
    }

  // ── WiFi: 802.11a ad-hoc ─────────────────────────────────────────────────
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211a);

  // ConstantRateWifiManager with OfdmRate54Mbps:
  //   Fixed rate avoids the assertion failures that IdealWifiManager and
  //   MinstrelWifiManager can trigger on ns-3.36-3.38 when used with
  //   802.11a ad-hoc mode.  54 Mbps is the max OFDM rate for 802.11a.
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                               "DataMode",    StringValue("OfdmRate6Mbps"),
                               "ControlMode", StringValue("OfdmRate6Mbps"));

  // ── Channel: layered propagation models ─────────────────────────────────
  //
  //  Layer 1 (delay):   FixedPropagationDelayModel
  //  Layer 2 (loss):    LogDistancePropagationLossModel   — primary path loss
  //  Layer 3 (fading):  NakagamiPropagationLossModel      — fast fading
  //  Layer 4 (shadow):  RandomPropagationLossModel         — slow shadowing
  //
  //  ns-3 chains loss models: LogDistance feeds into Nakagami which feeds
  //  into the shadow model.  This avoids the assertion failures that
  //  TwoRayGroundPropagationLossModel can trigger with certain mobility
  //  setups in ns-3.36-3.38.

  YansWifiChannelHelper channel;

  // Delay
  channel.SetPropagationDelay("ns3::FixedPropagationDelayModel",
                              "Delay", TimeValue(MilliSeconds(delayMs)));

  // ── Layer 2: Log-distance path loss ─────────────────────────────────────
  //  Standard log-distance model with exponent 3.0 (between free-space
  //  exponent 2.0 and urban 4.0).  Reference distance 1 m at 5.18 GHz
  //  gives a reference loss of ~47.3 dB (Friis at 1 m).  This model is
  //  reliable across all ns-3 versions and does not require height params.
  channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                             "Exponent", DoubleValue(2.0),
                             "ReferenceLoss", DoubleValue(47.3));

  // ── Layer 3: Nakagami-m fast fading ─────────────────────────────────────
  //  Chained after LogDistance so the total loss = LogDistance + Nakagami.
  //  m0: fading factor for distance  0 – 80 m  (near, possible LOS)
  //  m1: fading factor for distance 80 – 200 m (mid, mixed)
  //  m2: fading factor for distance > 200 m    (far, NLOS dominant)
  //
  //  m=1.0 → pure Rayleigh (fully scattered, no dominant LOS component)
  //  m=1.5 → slight LOS dominance (typical low-altitude UAV outdoor)
  //  m=2.0 → moderate LOS (clear sky, sparse environment)
  //  m=0.5 → worse than Rayleigh (dense urban / heavy NLOS)
  channel.AddPropagationLoss("ns3::NakagamiPropagationLossModel",
                             "m0", DoubleValue(nakagamiM),        // near zone
                             "m1", DoubleValue(std::max(0.5, nakagamiM - 0.3)), // mid zone
                             "m2", DoubleValue(std::max(0.5, nakagamiM - 0.5)), // far zone
                             "Distance1", DoubleValue(80.0),
                             "Distance2", DoubleValue(200.0));

  // ── Layer 4: Log-normal shadowing ───────────────────────────────────────
  //  Models slow-varying obstruction (trees, buildings, terrain).
  //  Uses a zero-mean Normal RV with std dev = shadowingStdDb.
  //  Typical values: 2-4 dB open sky, 4-8 dB suburban, 8-12 dB urban.
  if (shadowingStdDb > 0.0)
    {
      channel.AddPropagationLoss(
        "ns3::RandomPropagationLossModel",
        "Variable",
        StringValue("ns3::NormalRandomVariable[Mean=0|Variance="
                    + std::to_string(shadowingStdDb * shadowingStdDb) + "]"));
    }

  // ── PHY layer ────────────────────────────────────────────────────────────
  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());

  // Tx power: typical UAV datalink radio is 20-23 dBm (100-200 mW)
  phy.Set("TxPowerStart", DoubleValue(txPowerDbm));
  phy.Set("TxPowerEnd",   DoubleValue(txPowerDbm));

  // Rx sensitivity: default -82 dBm for 802.11a, good enough for testing
  // Uncomment to tighten: phy.Set("RxSensitivity", DoubleValue(-75.0));

  // Enable monitoring sniffer (required for SNR logging)
  phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

  WifiMacHelper mac;
  mac.SetType("ns3::AdhocWifiMac");

  NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

  // ── Optional extra burst error model ────────────────────────────────────
  //  Sits on top of the channel models to inject additional random loss
  //  (e.g., to simulate hardware packet drops or OS buffer overflows).
  if (lossRate > 0.0)
    {
      for (uint32_t i = 0; i < devices.GetN(); ++i)
        {
          Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
          em->SetAttribute("ErrorRate", DoubleValue(lossRate));
          devices.Get(i)->SetAttribute("ReceiveErrorModel", PointerValue(em));
        }
      NS_LOG_UNCOND("Extra burst error model applied: lossRate=" << lossRate);
    }

  // ── Internet stack ───────────────────────────────────────────────────────
  InternetStackHelper internet;
  internet.Install(nodes);

  // ── FlowMonitor ──────────────────────────────────────────────────────────
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;
  if (enableFlowMonitor) monitor = flowmon.InstallAll();

  // ── IP addressing ─────────────────────────────────────────────────────────
  //  Node 0 (GCS)  = 10.42.0.10
  //  Node 1 (UAV1) = 10.42.0.11
  //  Node 2 (UAV2) = 10.42.0.12
  //  Node 3 (UAV3) = 10.42.0.13
  //  Start at .10 so GCS gets the first address.
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.42.0.0", "255.255.255.0", "0.0.0.10"); // start at .10 (GCS)
  Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

  // ── TapBridge ─────────────────────────────────────────────────────────────
  for (uint32_t i = 0; i < 4; ++i)
    {
      TapBridgeHelper tap;
      tap.SetAttribute("Mode",       StringValue("UseLocal"));
      tap.SetAttribute("DeviceName", StringValue(tapNames[i]));
      tap.Install(nodes.Get(i), devices.Get(i));
      NS_LOG_UNCOND((i == 0 ? std::string("GCS") : "UAV" + std::to_string(i))
        << "  TAP=" << tapNames[i]
        << "  ns3-ip=" << ifaces.GetAddress(i));
    }

  // ── SNR / RSSI CSV logger ─────────────────────────────────────────────────
  if (enableSnrLog)
    {
      g_snrFile.open(snrLogFile);
      if (g_snrFile.is_open())
        {
          g_snrFile << "time_s,rx_node,signal_dbm,noise_dbm,snr_db\n";
          // Connect to every node's PHY sniffer trace (GCS + 3 UAVs)
          for (uint32_t i = 0; i < devices.GetN(); ++i)
            {
              Ptr<WifiNetDevice> wnd = DynamicCast<WifiNetDevice>(devices.Get(i));
              if (wnd && wnd->GetPhy())
                {
                  std::string ctx = "/NodeList/" + std::to_string(i)
                                    + "/DeviceList/0/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";
                  Config::Connect(ctx, MakeCallback(&MonitorSnifferCallback));
                }
            }
          NS_LOG_UNCOND("SNR log → " << snrLogFile);
        }
      else
        {
          NS_LOG_WARN("Could not open SNR log file: " << snrLogFile);
        }
    }

  // ── Simulator stop ───────────────────────────────────────────────────────
  if (simDurationSec > 0.0)
    Simulator::Stop(Seconds(simDurationSec));

  // ── NetAnim ───────────────────────────────────────────────────────────────
  // Only enable detailed packet metadata for bounded/short test runs.
  // For live missions (simDurationSec=0) this would grow unbounded.
  AnimationInterface anim(animFile);
  if (simDurationSec > 0.0 && simDurationSec <= 300.0)
    {
      anim.EnablePacketMetadata(true);
    }
  else
    {
      NS_LOG_UNCOND("NetAnim packet metadata disabled (unbounded/long run) "
                    "to avoid unbounded memory/file growth.");
    }

  // GCS node (index 0) — white
  anim.UpdateNodeDescription(nodes.Get(0), "GCS " + tapNames[0]);
  anim.UpdateNodeColor(nodes.Get(0), 255, 255, 255);
  anim.UpdateNodeSize(nodes.Get(0), 7.0, 7.0);

  // UAV nodes (indices 1-3)
  const std::array<uint16_t, 3>              mavPorts = {5760, 5770, 5780};
  const std::array<std::array<uint8_t,3>,3>  clr      = {{{255,0,0},{0,200,0},{0,80,255}}};
  for (uint32_t i = 0; i < 3; ++i)
    {
      anim.UpdateNodeDescription(nodes.Get(i+1),
        "UAV" + std::to_string(i+1) + " " + tapNames[i+1]
        + " tcp:" + std::to_string(mavPorts[i]));
      anim.UpdateNodeColor(nodes.Get(i+1), clr[i][0], clr[i][1], clr[i][2]);
      anim.UpdateNodeSize(nodes.Get(i+1), 5.0, 5.0);
    }

  // GCS ↔ UAV links
  for (uint32_t j = 1; j <= 3; ++j)
    {
      std::string label = "GCS-UAV" + std::to_string(j);
      anim.UpdateLinkDescription(nodes.Get(0), nodes.Get(j), label);
      anim.UpdateLinkDescription(nodes.Get(j), nodes.Get(0), label);
    }
  // UAV ↔ UAV links
  for (uint32_t i = 1; i <= 3; ++i)
    for (uint32_t j = i+1; j <= 3; ++j)
      {
        std::string label = "NakM=" + std::to_string(nakagamiM).substr(0,3)
                          + " shadow=" + std::to_string((int)shadowingStdDb) + "dB";
        anim.UpdateLinkDescription(nodes.Get(i), nodes.Get(j), label);
        anim.UpdateLinkDescription(nodes.Get(j), nodes.Get(i), label);
      }

  // PHY flash on receive — GCS + 3 UAVs
  for (uint32_t i = 0; i < devices.GetN() && i < 4; ++i)
    {
      Ptr<WifiNetDevice> wnd = DynamicCast<WifiNetDevice>(devices.Get(i));
      if (wnd && wnd->GetPhy())
        {
          // GCS = white flash; UAVs = their own colour
          uint8_t r = 255, g = 255, b = 255; // default: GCS white
          if (i >= 1 && i <= 3)
            { r = clr[i-1][0]; g = clr[i-1][1]; b = clr[i-1][2]; }
          wnd->GetPhy()->TraceConnectWithoutContext(
            "PhyRxEnd",
            MakeBoundCallback(&FlashNodeOnPhyRxEnd, &anim,
                              nodes.Get(i)->GetId(), r, g, b));
        }
    }

  // ── Periodic position / distance log ─────────────────────────────────────
  Simulator::Schedule(Seconds(1.0), &LogNodePositions, nodes);

  // ── Run ──────────────────────────────────────────────────────────────────
  NS_LOG_UNCOND("=== Starting realistic UAV simulation ===");
  NS_LOG_UNCOND("  Channel: LogDist(n=2.0) + Nakagami(m=" << nakagamiM
    << ") + Shadow(σ=" << shadowingStdDb << "dB)");
  NS_LOG_UNCOND("  Rate: OfdmRate6Mbps (ConstantRate)");
  NS_LOG_UNCOND("  TxPower=" << txPowerDbm << "dBm"
    << "  Delay=" << delayMs << "ms"
    << "  Mobility=" << (enableGaussMarkov ? "Gauss-Markov" : "Waypoint"));

  Simulator::Run();

  // ── FlowMonitor summary ──────────────────────────────────────────────────
  if (enableFlowMonitor && monitor)
    {
      monitor->CheckForLostPackets();

      Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

      uint64_t totTx=0, totRx=0, totLost=0, totBytes=0;
      Time delaySum=Seconds(0), jitterSum=Seconds(0);

      const auto& stats = monitor->GetFlowStats();
      for (const auto& entry : stats)
        {
          const auto& s = entry.second;
          totTx    += s.txPackets;
          totRx    += s.rxPackets;
          totLost  += s.lostPackets;
          totBytes += s.rxBytes;
          delaySum  += s.delaySum;
          jitterSum += s.jitterSum;

          if (classifier)
            {
              auto t = classifier->FindFlow(entry.first);
              const double avgDly = s.rxPackets>0
                ? s.delaySum.GetSeconds()*1e3/s.rxPackets : 0;
              NS_LOG_UNCOND("  Flow " << entry.first
                << " " << t.sourceAddress << "->" << t.destinationAddress
                << " tx=" << s.txPackets << " rx=" << s.rxPackets
                << " lost=" << s.lostPackets
                << " avgDelay=" << std::fixed << std::setprecision(2) << avgDly << "ms");
            }
        }

      const double dur = simDurationSec > 0 ? simDurationSec
                                             : Simulator::Now().GetSeconds();
      const double tput = dur>0 ? totBytes*8.0/(dur*1e6) : 0;
      const double avgDlyMs = totRx>0 ? delaySum.GetSeconds()*1e3/totRx : 0;
      const double avgJitMs = totRx>1 ? jitterSum.GetSeconds()*1e3/(totRx-1) : 0;
      const double lossPct  = totTx>0 ? 100.0*totLost/totTx : 0;

      NS_LOG_UNCOND("\n=== FlowMonitor summary ===");
      NS_LOG_UNCOND("  Throughput : " << std::fixed << std::setprecision(3) << tput << " Mbps");
      NS_LOG_UNCOND("  Avg delay  : " << avgDlyMs << " ms");
      NS_LOG_UNCOND("  Avg jitter : " << avgJitMs << " ms");
      NS_LOG_UNCOND("  Loss       : " << lossPct  << " %");
      NS_LOG_UNCOND("  tx/rx/lost : " << totTx << "/" << totRx << "/" << totLost);

      monitor->SerializeToXmlFile(flowmonXml, true, true);
      NS_LOG_UNCOND("FlowMonitor XML → " << flowmonXml);
    }

  if (g_snrFile.is_open()) g_snrFile.close();

  NS_LOG_UNCOND("NetAnim XML → " << animFile);
  Simulator::Destroy();
  return 0;
}
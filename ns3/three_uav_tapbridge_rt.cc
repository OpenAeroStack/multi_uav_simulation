// =============================================================================
//  three_uav_tapbridge_rt.cc
//
//  Three-UAV TapBridge simulation with realistic channel modelling:
//    - Log-distance path loss     (primary path loss)
//    - Nakagami-m fading overlay  (multipath / LOS richness)
//    - Optional independent Gaussian loss variation
//    - ConstantRate OfdmRate54Mbps (fixed rate, crash-safe)
//    - Fixed initial positions    (pending live Gazebo synchronization)
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
//    ./ns3 run "scratch/three_uav_tapbridge_rt \
//      --simDurationSec=120 \
//      --delayMs=20 \
//      --lossRate=0.05 \
//      --nakagamiM=1.5 \
//      --shadowingStdDb=4.0 \
//      --txPowerDbm=20 \
//      --uavAltitude=20 \
//      --enableFlowMonitor=true \
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
#include <arpa/inet.h>
#include <cctype>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThreeUavRealistic");

// ─────────────────────────────────────────────────────────────────────────────
namespace
{

// ─────────────────────────────────────────────────────────────────────────────
//  SNR / RSSI logger
//  Hooks into WifiPhy MonitorSnifferRx trace to record per-packet SNR.
//  Output: CSV with columns  time_s, rx_node, tx_addr, snr_db, rssi_dbm
// ─────────────────────────────────────────────────────────────────────────────
static std::ofstream g_snrFile;
static std::ofstream g_frameMetricsFile;
static std::ofstream g_throughputFile;

struct WifiCounters
{
  uint64_t txFrames{0}, rxFrames{0}, txBytes{0}, rxBytes{0};
  uint64_t macTxDrops{0}, macRxDrops{0}, phyTxDrops{0}, phyRxDrops{0};
  uint64_t intervalTxFrames{0}, intervalRxFrames{0};
  uint64_t intervalTxBytes{0}, intervalRxBytes{0};
  uint64_t intervalMacTxDrops{0}, intervalMacRxDrops{0};
  uint64_t intervalPhyTxDrops{0}, intervalPhyRxDrops{0};
};
static std::array<WifiCounters, 4> g_wifiCounters;

static std::string
NodeLabel(uint32_t nodeId)
{
  return nodeId == 0 ? "GCS" : "UAV" + std::to_string(nodeId);
}

static void
WriteFrameEvent(uint32_t nodeId, const char* event, uint32_t bytes,
                const std::string& rssi = "", const std::string& snr = "")
{
  if (g_frameMetricsFile.is_open())
    g_frameMetricsFile << std::fixed << std::setprecision(6)
      << Simulator::Now().GetSeconds() << "," << nodeId << "," << NodeLabel(nodeId)
      << "," << event << "," << bytes << "," << rssi << "," << snr << "\n";
}

static void
MacTxCallback(uint32_t nodeId, Ptr<const Packet> packet)
{
  WriteFrameEvent(nodeId, "MAC_TX_OFFERED", packet->GetSize());
}

static void
MonitorSnifferTxCallback(uint32_t nodeId,
                         Ptr<const Packet> packet,
                         uint16_t channelFreqMhz,
                         WifiTxVector txVector,
                         MpduInfo aMpdu,
                         uint16_t staId)
{
  auto& c = g_wifiCounters.at(nodeId);
  ++c.txFrames; ++c.intervalTxFrames;
  c.txBytes += packet->GetSize(); c.intervalTxBytes += packet->GetSize();
  WriteFrameEvent(nodeId, "PHY_TX_FRAME", packet->GetSize());
}

static void
MacRxCallback(uint32_t nodeId, Ptr<const Packet> packet)
{
  auto& c = g_wifiCounters.at(nodeId);
  ++c.rxFrames; ++c.intervalRxFrames;
  c.rxBytes += packet->GetSize(); c.intervalRxBytes += packet->GetSize();
  WriteFrameEvent(nodeId, "MAC_RX_OK", packet->GetSize());
}

static void
MacTxDropCallback(uint32_t nodeId, Ptr<const Packet> packet)
{
  auto& c = g_wifiCounters.at(nodeId);
  ++c.macTxDrops; ++c.intervalMacTxDrops;
  WriteFrameEvent(nodeId, "MAC_TX_DROP", packet->GetSize());
}

static void
MacRxDropCallback(uint32_t nodeId, Ptr<const Packet> packet)
{
  auto& c = g_wifiCounters.at(nodeId);
  ++c.macRxDrops; ++c.intervalMacRxDrops;
  WriteFrameEvent(nodeId, "MAC_RX_DROP", packet->GetSize());
}

static void
PhyTxDropCallback(uint32_t nodeId, Ptr<const Packet> packet)
{
  auto& c = g_wifiCounters.at(nodeId);
  ++c.phyTxDrops; ++c.intervalPhyTxDrops;
  WriteFrameEvent(nodeId, "PHY_TX_DROP", packet->GetSize());
}

static void
PhyRxDropCallback(uint32_t nodeId, Ptr<const Packet> packet)
{
  auto& c = g_wifiCounters.at(nodeId);
  ++c.phyRxDrops; ++c.intervalPhyRxDrops;
  WriteFrameEvent(nodeId, "PHY_RX_DROP", packet->GetSize());
}

static void
MonitorSnifferCallback(uint32_t nodeId,
                       Ptr<const Packet> packet,
                       uint16_t channelFreqMhz,
                       WifiTxVector txVector,
                       MpduInfo aMpdu,
                       SignalNoiseDbm signalNoise,
                       uint16_t staId)
{
  const double snrDb = signalNoise.signal - signalNoise.noise;
  if (g_snrFile.is_open())
    g_snrFile << std::fixed << std::setprecision(4)
              << Simulator::Now().GetSeconds() << ","
              << nodeId << "," << NodeLabel(nodeId) << ","
              << signalNoise.signal << ","
              << signalNoise.noise  << ","
              << snrDb              << "\n";
  WriteFrameEvent(nodeId, "PHY_RX_SIGNAL", packet->GetSize(),
                  std::to_string(signalNoise.signal), std::to_string(snrDb));
}

static void
WriteThroughputSample(Time interval)
{
  if (g_throughputFile.is_open())
    {
      const double seconds = interval.GetSeconds();
      for (uint32_t i = 0; i < g_wifiCounters.size(); ++i)
        {
          auto& c = g_wifiCounters[i];
          g_throughputFile << std::fixed << std::setprecision(6)
            << Simulator::Now().GetSeconds() << "," << seconds << "," << i << ","
            << NodeLabel(i) << "," << c.intervalTxFrames << "," << c.intervalRxFrames
            << "," << c.intervalTxBytes << "," << c.intervalRxBytes
            << "," << c.intervalMacTxDrops << "," << c.intervalMacRxDrops
            << "," << c.intervalPhyTxDrops << "," << c.intervalPhyRxDrops
            << "," << (seconds > 0 ? c.intervalTxBytes * 8.0 / seconds / 1e6 : 0)
            << "," << (seconds > 0 ? c.intervalRxBytes * 8.0 / seconds / 1e6 : 0)
            << "," << c.txFrames << "," << c.rxFrames
            << "," << c.txBytes << "," << c.rxBytes << "\n";
          c.intervalTxFrames = c.intervalRxFrames = 0;
          c.intervalTxBytes = c.intervalRxBytes = 0;
          c.intervalMacTxDrops = c.intervalMacRxDrops = 0;
          c.intervalPhyTxDrops = c.intervalPhyRxDrops = 0;
        }
      g_throughputFile.flush();
    }
  Simulator::Schedule(interval, &WriteThroughputSample, interval);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Periodic position logger
// ─────────────────────────────────────────────────────────────────────────────
static void
LogNodePositions(NodeContainer nodes)
{
  const double now = Simulator::Now().GetSeconds();

  // Node 0 is the GCS.  Inter-UAV distances must use UAV nodes 1, 2, and 3.
  if (nodes.GetN() >= 4)
    {
      Ptr<MobilityModel> uav1 = nodes.Get(1)->GetObject<MobilityModel>();
      Ptr<MobilityModel> uav2 = nodes.Get(2)->GetObject<MobilityModel>();
      Ptr<MobilityModel> uav3 = nodes.Get(3)->GetObject<MobilityModel>();
      if (uav1 && uav2 && uav3)
        {
          const double d12 = uav1->GetDistanceFrom(uav2);
          const double d13 = uav1->GetDistanceFrom(uav3);
          const double d23 = uav2->GetDistanceFrom(uav3);
          NS_LOG_UNCOND("t=" << now
            << "s  d(UAV1-UAV2)=" << std::fixed << std::setprecision(1) << d12
            << "m  d(UAV1-UAV3)=" << d13
            << "m  d(UAV2-UAV3)=" << d23 << "m");
        }
    }

  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
      Ptr<MobilityModel> mm = nodes.Get(i)->GetObject<MobilityModel>();
      if (!mm) continue;
      const Vector p = mm->GetPosition();
      const std::string label = (i == 0) ? "GCS" : "UAV" + std::to_string(i);
      NS_LOG_UNCOND("  " << label
        << " pos=(" << p.x << "," << p.y << "," << p.z << ")");
    }
  Simulator::Schedule(Seconds(1.0), &LogNodePositions, nodes);
}

// ─────────────────────────────────────────────────────────────────────────────
// Host UDP position receiver. The socket is nonblocking and is polled only by
// scheduled simulator events, so no external thread touches ns-3 objects.
// ─────────────────────────────────────────────────────────────────────────────
struct ExternalPositionReceiver
{
  int fd{-1};
  NodeContainer nodes;
  Time pollInterval{MilliSeconds(100)};
  Time staleAfter{Seconds(1)};
  Time startedAt{Seconds(0)};
  Time lastUpdate{Seconds(0)};
  uint64_t lastSequence{0};
  bool hasUpdate{false};
  bool staleLogged{false};
};

static bool
ParseUnsigned(const std::string& text, uint64_t& value)
{
  try
    {
      std::size_t used = 0;
      value = std::stoull(text, &used);
      while (used < text.size() && std::isspace(static_cast<unsigned char>(text[used])))
        ++used;
      return used == text.size();
    }
  catch (...)
    {
      return false;
    }
}

static bool
ParseCoordinate(const std::string& text, double& value)
{
  try
    {
      std::size_t used = 0;
      value = std::stod(text, &used);
      while (used < text.size() && std::isspace(static_cast<unsigned char>(text[used])))
        ++used;
      return used == text.size() && std::isfinite(value);
    }
  catch (...)
    {
      return false;
    }
}

static bool
ParsePositionSnapshot(const std::string& message,
                      uint64_t& sequence,
                      std::array<Vector, 4>& positions)
{
  std::vector<std::string> fields;
  std::istringstream stream(message);
  std::string field;
  while (std::getline(stream, field, ','))
    fields.push_back(field);

  // NS3POS1,sequence,timestamp_ms,(id,x,y,z) x 4
  if (fields.size() != 19 || fields[0] != "NS3POS1")
    return false;

  uint64_t timestampMs = 0;
  if (!ParseUnsigned(fields[1], sequence) || !ParseUnsigned(fields[2], timestampMs))
    return false;

  std::array<bool, 4> seen = {false, false, false, false};
  for (std::size_t offset = 3; offset < fields.size(); offset += 4)
    {
      uint64_t entityId = 0;
      double x = 0, y = 0, z = 0;
      if (!ParseUnsigned(fields[offset], entityId) || entityId > 3 || seen[entityId] ||
          !ParseCoordinate(fields[offset + 1], x) ||
          !ParseCoordinate(fields[offset + 2], y) ||
          !ParseCoordinate(fields[offset + 3], z))
        return false;
      positions[entityId] = Vector(x, y, z);
      seen[entityId] = true;
    }
  return seen[0] && seen[1] && seen[2] && seen[3];
}

static void
PollExternalPositions(ExternalPositionReceiver* receiver)
{
  char buffer[1024];
  bool applied = false;

  while (true)
    {
      const ssize_t bytes = recv(receiver->fd, buffer, sizeof(buffer), 0);
      if (bytes < 0)
        {
          if (errno == EAGAIN || errno == EWOULDBLOCK)
            break;
          NS_LOG_WARN("External mobility recv failed: " << std::strerror(errno));
          break;
        }
      if (bytes == 0)
        break;

      uint64_t sequence = 0;
      std::array<Vector, 4> positions;
      if (!ParsePositionSnapshot(std::string(buffer, static_cast<std::size_t>(bytes)),
                                 sequence, positions))
        continue;
      if (receiver->hasUpdate && sequence <= receiver->lastSequence)
        continue;

      bool modelsValid = true;
      for (uint32_t i = 0; i < 4; ++i)
        {
          Ptr<ConstantPositionMobilityModel> model =
            receiver->nodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
          if (!model)
            {
              modelsValid = false;
              break;
            }
        }
      if (!modelsValid)
        continue;

      for (uint32_t i = 0; i < 4; ++i)
        receiver->nodes.Get(i)->GetObject<ConstantPositionMobilityModel>()
          ->SetPosition(positions[i]);
      receiver->lastSequence = sequence;
      receiver->lastUpdate = Simulator::Now();
      receiver->hasUpdate = true;
      applied = true;
    }

  if (applied && receiver->staleLogged)
    {
      NS_LOG_UNCOND("External mobility updates recovered.");
      receiver->staleLogged = false;
    }

  const Time reference = receiver->hasUpdate ? receiver->lastUpdate : receiver->startedAt;
  if (Simulator::Now() - reference >= receiver->staleAfter && !receiver->staleLogged)
    {
      if (receiver->hasUpdate)
        NS_LOG_WARN("External mobility data is stale; retaining last valid positions.");
      else
        NS_LOG_WARN("No external mobility data received; retaining fixed fallback positions.");
      receiver->staleLogged = true;
    }

  Simulator::Schedule(receiver->pollInterval, &PollExternalPositions, receiver);
}

static void
OpenExternalPositionReceiver(ExternalPositionReceiver* receiver,
                             const std::string& bindAddress,
                             uint32_t port)
{
  NS_ABORT_MSG_IF(port == 0 || port > 65535, "positionSyncPort must be 1..65535");
  receiver->fd = socket(AF_INET, SOCK_DGRAM, 0);
  NS_ABORT_MSG_IF(receiver->fd < 0, "Could not create external mobility UDP socket");

  const int flags = fcntl(receiver->fd, F_GETFL, 0);
  NS_ABORT_MSG_IF(flags < 0 || fcntl(receiver->fd, F_SETFL, flags | O_NONBLOCK) < 0,
                  "Could not make external mobility socket nonblocking");

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(port));
  NS_ABORT_MSG_IF(inet_pton(AF_INET, bindAddress.c_str(), &address.sin_addr) != 1,
                  "positionSyncAddress must be an IPv4 address");
  NS_ABORT_MSG_IF(bind(receiver->fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0,
                  "Could not bind external mobility UDP socket at "
                    << bindAddress << ":" << port << ": " << std::strerror(errno));
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
  double delayMs          = 20.0;   // deprecated compatibility option; ignored
  double lossRate         = 0.0;    // extra burst loss on top of channel models
  double simDurationSec   = 60.0;   // 0 = run forever (real-time)
  double uavSpeed         = 5.0;    // deprecated compatibility option; ignored
  double uavAltitude      = 20.0;   // deprecated compatibility option; ignored
  double gcsX = 0.0,   gcsY = 0.0, gcsZ = 0.0;
  double uav1X = 0.0,  uav1Y = 0.0, uav1Z = 60.0;
  double uav2X = 50.0, uav2Y = 0.0, uav2Z = 40.0;
  double uav3X = -50.0, uav3Y = 0.0, uav3Z = 50.0;
  double nakagamiM        = 1.5;    // Nakagami-m: 1.0=Rayleigh, >1 = more LOS
  double shadowingStdDb   = 4.0;    // independent Gaussian loss std deviation (dB)
  double txPowerDbm       = 20.0;   // Tx power in dBm
  bool   enableGaussMarkov = true;  // deprecated compatibility option; ignored
  bool   enableFlowMonitor = true;
  bool   enableSnrLog      = true;
  bool   enableWifiMetrics = true;
  bool   enableExternalMobilitySync = false;
  std::string positionSyncAddress = "127.0.0.1";
  uint32_t positionSyncPort = 5555;
  double positionSyncPollMs = 100.0;
  double positionSyncStaleMs = 1000.0;
  std::string flowmonXml  = "three_uav_flowmon.xml";
  std::string animFile    = "three_uav_anim.xml";
  std::string snrLogFile  = "/tmp/snr_log.csv";
  std::string frameMetricsFile = "/tmp/wifi_frame_metrics.csv";
  std::string throughputFile = "/tmp/wifi_throughput.csv";
  double throughputIntervalSec = 1.0;

  CommandLine cmd(__FILE__);
  cmd.AddValue("tap0",            "TAP name for GCS",                     tapNames[0]);
  cmd.AddValue("tap1",            "TAP name for UAV1",                    tapNames[1]);
  cmd.AddValue("tap2",            "TAP name for UAV2",                    tapNames[2]);
  cmd.AddValue("tap3",            "TAP name for UAV3",                    tapNames[3]);
  cmd.AddValue("delayMs",         "DEPRECATED and ignored; delay follows distance at light speed", delayMs);
  cmd.AddValue("lossRate",        "Extra burst error rate [0-1]",         lossRate);
  cmd.AddValue("simDurationSec",  "Sim duration (0=forever)",             simDurationSec);
  cmd.AddValue("uavSpeed",        "DEPRECATED and ignored in fixed-position mode", uavSpeed);
  cmd.AddValue("uavAltitude",     "DEPRECATED and ignored; use uav1Z/uav2Z/uav3Z", uavAltitude);
  cmd.AddValue("gcsX",            "Initial GCS X position (m)",            gcsX);
  cmd.AddValue("gcsY",            "Initial GCS Y position (m)",            gcsY);
  cmd.AddValue("gcsZ",            "Initial GCS Z position (m)",            gcsZ);
  cmd.AddValue("uav1X",           "Initial UAV1 X position (m)",           uav1X);
  cmd.AddValue("uav1Y",           "Initial UAV1 Y position (m)",           uav1Y);
  cmd.AddValue("uav1Z",           "Initial UAV1 Z position (m)",           uav1Z);
  cmd.AddValue("uav2X",           "Initial UAV2 X position (m)",           uav2X);
  cmd.AddValue("uav2Y",           "Initial UAV2 Y position (m)",           uav2Y);
  cmd.AddValue("uav2Z",           "Initial UAV2 Z position (m)",           uav2Z);
  cmd.AddValue("uav3X",           "Initial UAV3 X position (m)",           uav3X);
  cmd.AddValue("uav3Y",           "Initial UAV3 Y position (m)",           uav3Y);
  cmd.AddValue("uav3Z",           "Initial UAV3 Z position (m)",           uav3Z);
  cmd.AddValue("nakagamiM",       "Nakagami-m fading factor (>=0.5)",     nakagamiM);
  cmd.AddValue("shadowingStdDb",  "Independent Gaussian loss std dev (dB; 0 disables)", shadowingStdDb);
  cmd.AddValue("txPowerDbm",      "Transmit power in dBm",                txPowerDbm);
  cmd.AddValue("enableGaussMarkov","DEPRECATED and ignored in fixed-position mode", enableGaussMarkov);
  cmd.AddValue("enableFlowMonitor","Enable FlowMonitor",                  enableFlowMonitor);
  cmd.AddValue("enableSnrLog",    "Log SNR per packet to CSV",            enableSnrLog);
  cmd.AddValue("enableWifiMetrics", "Enable Wi-Fi MAC/PHY traffic metrics", enableWifiMetrics);
  cmd.AddValue("enableExternalMobilitySync", "Apply NS3POS1 host UDP position snapshots", enableExternalMobilitySync);
  cmd.AddValue("positionSyncAddress", "Host IPv4 address for position receiver", positionSyncAddress);
  cmd.AddValue("positionSyncPort", "Host UDP port for position receiver", positionSyncPort);
  cmd.AddValue("positionSyncPollMs", "Simulator-event socket polling interval (ms)", positionSyncPollMs);
  cmd.AddValue("positionSyncStaleMs", "Stale update warning threshold (ms)", positionSyncStaleMs);
  cmd.AddValue("flowmonXml",      "FlowMonitor XML output",               flowmonXml);
  cmd.AddValue("animFile",        "NetAnim XML output",                   animFile);
  cmd.AddValue("snrLogFile",      "SNR CSV output path",                  snrLogFile);
  cmd.AddValue("frameMetricsFile", "Wi-Fi frame event CSV output", frameMetricsFile);
  cmd.AddValue("throughputFile", "Wi-Fi interval throughput CSV output", throughputFile);
  cmd.AddValue("throughputIntervalSec", "Wi-Fi throughput sample interval", throughputIntervalSec);
  cmd.Parse(argc, argv);

  if (delayMs != 20.0)
    {
      NS_LOG_WARN("delayMs is deprecated and ignored; using ConstantSpeedPropagationDelayModel");
    }

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

  // ── Mobility: fixed positions pending live Gazebo synchronization ────────
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator>();
  positions->Add(Vector(gcsX,  gcsY,  gcsZ));
  positions->Add(Vector(uav1X, uav1Y, uav1Z));
  positions->Add(Vector(uav2X, uav2Y, uav2Z));
  positions->Add(Vector(uav3X, uav3Y, uav3Z));
  mobility.SetPositionAllocator(positions);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  NS_LOG_UNCOND("Initial mobility positions (fixed):");
  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
      const Vector p = nodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
      const std::string label = (i == 0) ? "GCS" : "UAV" + std::to_string(i);
      NS_LOG_UNCOND("  " << label << " pos=(" << p.x << "," << p.y << "," << p.z << ")");
    }

  ExternalPositionReceiver positionReceiver;
  if (enableExternalMobilitySync)
    {
      NS_ABORT_MSG_IF(positionSyncPollMs <= 0.0 || positionSyncStaleMs <= 0.0,
                      "Position sync poll and stale intervals must be positive");
      positionReceiver.nodes = nodes;
      positionReceiver.pollInterval = MilliSeconds(positionSyncPollMs);
      positionReceiver.staleAfter = MilliSeconds(positionSyncStaleMs);
      positionReceiver.startedAt = Simulator::Now();
      OpenExternalPositionReceiver(&positionReceiver, positionSyncAddress, positionSyncPort);
      Simulator::ScheduleNow(&PollExternalPositions, &positionReceiver);
      NS_LOG_UNCOND("External mobility sync listening on udp://"
                    << positionSyncAddress << ":" << positionSyncPort
                    << " poll=" << positionSyncPollMs
                    << "ms stale=" << positionSyncStaleMs << "ms");
    }
  else
    {
      NS_LOG_UNCOND("External mobility sync disabled; using fixed fallback positions.");
    }

  // ── WiFi: 802.11a ad-hoc ─────────────────────────────────────────────────
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211a);

  // ConstantRateWifiManager with OfdmRate54Mbps:
  //   Fixed rate avoids the assertion failures that IdealWifiManager and
  //   MinstrelWifiManager can trigger on ns-3.36-3.38 when used with
  //   802.11a ad-hoc mode.  54 Mbps is the max OFDM rate for 802.11a.
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                               "DataMode",    StringValue("OfdmRate54Mbps"),
                               "ControlMode", StringValue("OfdmRate6Mbps"));

  // ── Channel: layered propagation models ─────────────────────────────────
  //
  //  Layer 1 (delay):   ConstantSpeedPropagationDelayModel (distance / c)
  //  Layer 2 (loss):    LogDistancePropagationLossModel   — primary path loss
  //  Layer 3 (fading):  NakagamiPropagationLossModel      — fast fading
  //  Layer 4 (optional): RandomPropagationLossModel — independent variation
  //
  //  ns-3 chains loss models: LogDistance feeds into Nakagami, followed by
  //  optional independent random loss variation.  This avoids assertion failures that
  //  TwoRayGroundPropagationLossModel can trigger with certain mobility
  //  setups in ns-3.36-3.38.

  YansWifiChannelHelper channel;

  // Physical propagation delay: separation divided by propagation speed.
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

  // ── Layer 2: Log-distance path loss ─────────────────────────────────────
  //  Standard log-distance model with exponent 3.0 (between free-space
  //  exponent 2.0 and urban 4.0).  Reference distance 1 m at 5.18 GHz
  //  gives a reference loss of ~47.3 dB (Friis at 1 m).  This model is
  //  reliable across all ns-3 versions and does not require height params.
  channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                             "Exponent", DoubleValue(3.0),
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

  // ── Layer 4: Optional independent random loss variation ────────────────
  //  RandomPropagationLossModel draws independently whenever loss is
  //  evaluated.  It is not a spatially correlated shadowing model.  The
  //  legacy shadowingStdDb option is retained for command-line compatibility.
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

  // FlowMonitor sees flows traversing ns-3's simulated IPv4 probes. External
  // TapBridge frames may not traverse those probes, so FlowMonitor is retained
  // only as supplementary simulated-IP diagnostics; Wi-Fi traces below are the
  // authoritative metrics for emulated TAP traffic.
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
  // UseLocal requires exactly one Linux-side source MAC behind each TAP.  The
  // host topology provides one namespace wifi0 veth endpoint per dedicated
  // bridge/TAP pair; no TAP is shared by multiple Linux endpoints.
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

  // ── Wi-Fi MAC/PHY metrics for external TapBridge traffic ─────────────────
  if (enableSnrLog || enableWifiMetrics)
    {
      if (enableSnrLog)
        {
          g_snrFile.open(snrLogFile);
          if (g_snrFile.is_open())
            g_snrFile << "time_s,rx_node,node_label,rssi_dbm,noise_dbm,snr_db\n";
          NS_LOG_UNCOND("SNR log → " << snrLogFile);
        }
      if (enableWifiMetrics)
        {
          NS_ABORT_MSG_IF(throughputIntervalSec <= 0,
                          "throughputIntervalSec must be positive");
          g_frameMetricsFile.open(frameMetricsFile);
          g_throughputFile.open(throughputFile);
          NS_ABORT_MSG_IF(!g_frameMetricsFile.is_open() || !g_throughputFile.is_open(),
                          "Could not open Wi-Fi metrics CSV files");
          g_frameMetricsFile << "time_s,node_id,node_label,event,frame_bytes,rssi_dbm,snr_db\n";
          g_throughputFile
            << "time_s,interval_s,node_id,node_label,tx_frames,rx_frames,tx_bytes,rx_bytes,"
            << "mac_tx_drops,mac_rx_drops,phy_tx_drops,phy_rx_drops,tx_mbps,rx_mbps,"
            << "cumulative_tx_frames,cumulative_rx_frames,cumulative_tx_bytes,cumulative_rx_bytes\n";
          Simulator::Schedule(Seconds(throughputIntervalSec), &WriteThroughputSample,
                              Seconds(throughputIntervalSec));
          NS_LOG_UNCOND("Wi-Fi frame metrics → " << frameMetricsFile);
          NS_LOG_UNCOND("Wi-Fi throughput metrics → " << throughputFile);
        }

      for (uint32_t i = 0; i < devices.GetN(); ++i)
        {
          Ptr<WifiNetDevice> wnd = DynamicCast<WifiNetDevice>(devices.Get(i));
          if (!wnd || !wnd->GetPhy() || !wnd->GetMac())
            continue;
          wnd->GetPhy()->TraceConnectWithoutContext(
            "MonitorSnifferRx", MakeBoundCallback(&MonitorSnifferCallback, i));
          wnd->GetPhy()->TraceConnectWithoutContext(
            "MonitorSnifferTx", MakeBoundCallback(&MonitorSnifferTxCallback, i));
          if (enableWifiMetrics)
            {
              wnd->GetMac()->TraceConnectWithoutContext(
                "MacTx", MakeBoundCallback(&MacTxCallback, i));
              wnd->GetMac()->TraceConnectWithoutContext(
                "MacRx", MakeBoundCallback(&MacRxCallback, i));
              wnd->GetMac()->TraceConnectWithoutContext(
                "MacTxDrop", MakeBoundCallback(&MacTxDropCallback, i));
              wnd->GetMac()->TraceConnectWithoutContext(
                "MacRxDrop", MakeBoundCallback(&MacRxDropCallback, i));
              wnd->GetPhy()->TraceConnectWithoutContext(
                "PhyTxDrop", MakeBoundCallback(&PhyTxDropCallback, i));
              wnd->GetPhy()->TraceConnectWithoutContext(
                "PhyRxDrop", MakeBoundCallback(&PhyRxDropCallback, i));
            }
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
                          + " randomLossSigma=" + std::to_string((int)shadowingStdDb) + "dB";
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
  NS_LOG_UNCOND("  Channel: LogDist(n=3.0) + Nakagami(m=" << nakagamiM
    << ") + IndependentRandomLoss(σ=" << shadowingStdDb << "dB)");
  NS_LOG_UNCOND("  Rate: OfdmRate54Mbps (ConstantRate)");
  NS_LOG_UNCOND("  TxPower=" << txPowerDbm << "dBm"
    << "  Delay=distance/c"
    << "  Mobility=ConstantPosition");

  Simulator::Run();

  if (positionReceiver.fd >= 0)
    {
      close(positionReceiver.fd);
      positionReceiver.fd = -1;
    }

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

      NS_LOG_UNCOND("\n=== FlowMonitor summary (simulated IPv4 probes only; not authoritative for TapBridge traffic) ===");
      NS_LOG_UNCOND("  Throughput : " << std::fixed << std::setprecision(3) << tput << " Mbps");
      NS_LOG_UNCOND("  Avg delay  : " << avgDlyMs << " ms");
      NS_LOG_UNCOND("  Avg jitter : " << avgJitMs << " ms");
      NS_LOG_UNCOND("  Loss       : " << lossPct  << " %");
      NS_LOG_UNCOND("  tx/rx/lost : " << totTx << "/" << totRx << "/" << totLost);

      monitor->SerializeToXmlFile(flowmonXml, true, true);
      NS_LOG_UNCOND("FlowMonitor XML → " << flowmonXml);
    }

  if (g_snrFile.is_open()) g_snrFile.close();
  if (g_frameMetricsFile.is_open()) g_frameMetricsFile.close();
  if (g_throughputFile.is_open()) g_throughputFile.close();

  NS_LOG_UNCOND("NetAnim XML → " << animFile);
  Simulator::Destroy();
  return 0;
}

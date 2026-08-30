// =============================================================================
//  three_uav_tapbridge_integrated.cc
//
//  Integration of:
//    - three_uav_tapbridge_obstacle_loss.cc  (ROS 2 / Gazebo co-simulation)
//    - three_uav_tapbridge_rt_new.cc         (small_city_world-wimukthi branch)
//
//  Topology: 4 nodes. All four positions come from Gazebo over ROS -- the GCS
//  is a real model in the world, not a hard-coded coordinate in this file.
//      node 0 = GCS   (ground station)      tap-gcs   10.42.0.10
//      node 1 = UAV1                        tap-uav1  10.42.0.11
//      node 2 = UAV2                        tap-uav2  10.42.0.12
//      node 3 = UAV3                        tap-uav3  10.42.0.13
//
//  6 links are modelled: 3 GCS<->UAV + 3 UAV<->UAV. The GCS links are the ones
//  that matter most -- the station sits at ground level, so it is by far the
//  likeliest to be occluded by buildings.
//
//  Channel:  DynamicObstacleLossModel -> LogDistancePropagationLossModel
//            (fading lives INSIDE the obstacle model -- see the long note at
//             "CHANNEL CHAIN" below for why ns3::NakagamiPropagationLossModel
//             is deliberately absent)
//
//  PHY:      802.11a ad-hoc, ConstantRate OfdmRate6Mbps
//  Realtime: RealtimeSimulatorImpl. Sustained app throughput ceiling on this
//            host is ~1.4 Mbps -- keep iperf3 / video below that or you are
//            measuring the wall clock, not the channel.
//
//  ROS 2 topics (ids are NS-3 node ids: 0=GCS, 1-3=UAVs -- no offset anywhere):
//     sub  /uav_world_positions   Float32MultiArray [id,x,y,z, ...]
//     sub  /link_obstacle_loss    Float32MultiArray [i,j,loss_dB, ...]
//                                 6 links: 3 GCS<->UAV + 3 UAV<->UAV
//     pub  /ns3_link_rssi         Float32MultiArray [a,b,rssi_dBm, ...]
//     pub  /ns3_link_snr          Float32MultiArray [a,b,snr_dB, ...]
//
//  NS-3 3.38.1  /  Ubuntu 22.04  /  ROS 2 Humble
// =============================================================================

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/propagation-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/error-model.h"
#include "ns3/stats-module.h"
#include "ns3/netanim-module.h"

#include "dynamic_obstacle_loss_model.hh"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThreeUavIntegrated");

// ─────────────────────────────────────────────────────────────────────────────
//  Node-index convention
//
//  The GCS now exists in Gazebo too, so the ray-caster and the position
//  publisher were renumbered to match NS-3 exactly:
//
//      id 0 = GCS,  id 1..3 = UAV1..UAV3     (everywhere: ROS, Gazebo, NS-3)
//
//  Ids are therefore used AS-IS -- there is no mapping step at all.
//
//  REMOVED: a g_rosNodeOffset / --rosNodeOffset knob added to every incoming
//  id. It was load-bearing for exactly one intermediate state, when NS-3 had
//  the GCS but Gazebo did not and the UAVs still had to be shifted 0..2 ->
//  1..3. Once the GCS became a real Gazebo model that shift went away, leaving
//  a flag whose only remaining effect was to break a working setup: set it to
//  1 against the current feed and the GCS position lands on node 1, UAV1 on
//  node 2, UAV3 falls off the end, and node 0 is never fed at all. An offset
//  kept in sync by hand across three processes is a silent-failure trap; the
//  fix was to make the numbering agree, not to keep the arithmetic around.
//
//  Two defences remain:
//    1. The convention is printed at startup so it is visible in every log.
//    2. Out-of-range ids WARN instead of being silently skipped (the original
//       code did `if (uid >= nodes.GetN()) continue;` -- a typo on the Gazebo
//       side produced a perfectly quiet simulation with frozen UAVs).
//    3. CheckIntegration() reports any node or link the feed never covered.
// ─────────────────────────────────────────────────────────────────────────────

// Bit k set once node k has received at least one position update. Written
// from the NS-3 thread inside ApplyFeed(), read by CheckIntegration().
static std::atomic<uint32_t> g_posSeenMask{0};

// ─────────────────────────────────────────────────────────────────────────────
//  Cross-thread feed buffers
//
//  THREADING RULE: the rclcpp executor thread must never touch an ns-3
//  refcounted object. ns3::SimpleRefCount uses a PLAIN uint32_t counter --
//  Ref()/Unref() are `m_count++` / `m_count--`, not atomics. Copying a Ptr<>
//  (or a NodeContainer, which holds Ptr<Node>) on the ROS thread while the
//  NS-3 thread copies the same object races that counter; a lost increment
//  drops it to zero early, the object is deleted while still in use, and the
//  next CalcRxPower() dereferences freed memory.
//
//  That is not hypothetical. The previous version captured
//  Ptr<DynamicObstacleLossModel> and NodeContainer by value into lambdas
//  constructed on the ROS thread and handed to Simulator::ScheduleWithContext.
//  It SIGSEGV'd inside PropagationLossModel::CalcRxPower after ~34 s of an
//  all-links-blocked run (6 links x 10 Hz of obstacle reports = the highest
//  Ptr-copy rate of any scenario). The same pattern is present in the older
//  three_uav_tapbridge_obstacle_loss.cc, where the lower message rate simply
//  made it rare enough to look stable.
//
//  ScheduleWithContext being thread-safe does not help: what races is the
//  refcount of the objects CAPTURED in the event, not the event queue.
//
//  So the callbacks below only write plain data (ids and doubles) into these
//  buffers under a mutex, and ApplyFeed() -- an ordinary NS-3 event on the
//  simulation thread -- drains them and does all the ns-3 work.
// ─────────────────────────────────────────────────────────────────────────────
static std::mutex g_feedMutex;

// Positions COALESCE: a position is state, so only the newest matters and
// dropping older ones changes nothing.
static std::map<uint32_t, Vector> g_pendingPos;

// Obstacle reports QUEUE: the model applies an EMA over the sample sequence,
// so coalescing would silently change the smoothing. Every sample is kept and
// applied in arrival order. (a, b, loss_dB)
static std::vector<std::array<double, 3>> g_pendingLoss;

static std::atomic<bool> g_rosRunning{true};

// Per-link model-validation CSV (opened only if --csvPath is given).
// Written from the NS-3 main thread inside PublishStats().
static std::ofstream g_csv;

// Per-packet PHY-level SNR/RSSI CSV (opened only if --snrLogFile is given).
// Written from the MonitorSnifferRx trace callback.
static std::ofstream g_snrFile;

// ─────────────────────────────────────────────────────────────────────────────
//  PHY-level delivery counters
//
//  REPLACES FlowMonitor from rt_new.cc, which cannot work in this scenario:
//  TapBridge in UseLocal mode bridges the Linux netns to the WifiNetDevice at
//  layer 2. Traffic never traverses the NS-3 Ipv4L3Protocol, which is exactly
//  where FlowMonitor installs its probes -- InstallAll() would faithfully
//  report zero flows forever and look like a broken link.
//
//  These counters hook WifiPhy traces instead, which sit BELOW the bridge and
//  therefore see every frame that actually crosses the simulated air.
// ─────────────────────────────────────────────────────────────────────────────
struct PhyCounters
{
  uint64_t txPackets  = 0;
  uint64_t txBytes    = 0;
  uint64_t rxOkPackets = 0;
  uint64_t rxOkBytes   = 0;
  uint64_t rxDropped  = 0;
};
static std::vector<PhyCounters> g_phyStats;

static void PhyTxEndCb(uint32_t nodeId, Ptr<const Packet> p)
{
  if (nodeId >= g_phyStats.size()) return;
  g_phyStats[nodeId].txPackets++;
  g_phyStats[nodeId].txBytes += p->GetSize();
}

static void PhyRxEndCb(uint32_t nodeId, Ptr<const Packet> p)
{
  if (nodeId >= g_phyStats.size()) return;
  g_phyStats[nodeId].rxOkPackets++;
  g_phyStats[nodeId].rxOkBytes += p->GetSize();
}

static void PhyRxDropCb(uint32_t nodeId, Ptr<const Packet> p, WifiPhyRxfailureReason r)
{
  if (nodeId >= g_phyStats.size()) return;
  g_phyStats[nodeId].rxDropped++;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Per-packet SNR sniffer (KEPT from rt_new.cc)
//
//  Complementary to PublishStats(), not redundant:
//    PublishStats  -> samples the MODEL every 0.5 s on every pair, including
//                     pairs carrying no traffic. Independent fading draw.
//                     Answers "what does the channel model say?"
//    MonitorSniffer-> records what REAL packets experienced, only where
//                     traffic actually flowed.
//                     Answers "what did the radio deliver?"
//  Agreement between the two is the strongest evidence the model is correctly
//  wired into the PHY.
// ─────────────────────────────────────────────────────────────────────────────
static void
MonitorSnifferCallback(std::string context,
                       Ptr<const Packet> packet,
                       uint16_t channelFreqMhz,
                       WifiTxVector txVector,
                       MpduInfo aMpdu,
                       SignalNoiseDbm signalNoise,
                       uint16_t staId)
{
  if (!g_snrFile.is_open()) return;

  uint32_t nodeId = 0;
  std::size_t pos = context.find("/NodeList/");
  if (pos != std::string::npos)
    {
      std::sscanf(context.c_str() + pos + 10, "%u", &nodeId);
    }

  const double snrDb = signalNoise.signal - signalNoise.noise;
  g_snrFile << std::fixed << std::setprecision(4)
            << Simulator::Now().GetSeconds() << ','
            << nodeId                        << ','
            << channelFreqMhz                << ','
            << packet->GetSize()             << ','
            << signalNoise.signal            << ','
            << signalNoise.noise             << ','
            << snrDb                         << '\n';
}

// ─────────────────────────────────────────────────────────────────────────────
//  Ns3RosNode -- one rclcpp::Node living inside the NS-3 process.
//  (KEPT essentially verbatim from obstacle_loss.cc; only the node-index
//   mapping and the out-of-range warnings are new.)
//
//  All callbacks run on rclcpp's executor thread, NOT the NS-3 simulation
//  thread. Every callback hands its work over via ScheduleWithContext(), which
//  is documented thread-safe under RealtimeSimulatorImpl.
// ─────────────────────────────────────────────────────────────────────────────
class Ns3RosNode : public rclcpp::Node
{
public:
  // CHANGED: takes a plain node COUNT, not a NodeContainer and not a
  // Ptr<DynamicObstacleLossModel>. This class must not hold any ns-3
  // refcounted object -- see the threading rule above the feed buffers.
  explicit Ns3RosNode(uint32_t nNodes)
  : rclcpp::Node("ns3_bridge_node"),
    m_nNodes(nNodes)
  {
    m_posSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/uav_world_positions", 10,
      std::bind(&Ns3RosNode::OnPositions, this, std::placeholders::_1));

    m_obsSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/link_obstacle_loss", 10,
      std::bind(&Ns3RosNode::OnObstacleLoss, this, std::placeholders::_1));

    m_rssiPub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/ns3_link_rssi", 10);
    m_snrPub  = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/ns3_link_snr", 10);
  }

  // Called from the NS-3 main thread. Safe: rclcpp publishers are thread-safe.
  void PublishRssi(const std::vector<float> & d)
  { std_msgs::msg::Float32MultiArray m; m.data = d; m_rssiPub->publish(m); }

  void PublishSnr(const std::vector<float> & d)
  { std_msgs::msg::Float32MultiArray m; m.data = d; m_snrPub->publish(m); }

private:
  // [id, x, y, z, ...] with id = NS-3 node id: 0 = GCS, 1..3 = UAV1..UAV3.
  // The GCS arrives on this topic exactly like the UAVs -- world_pos_publisher
  // reads its model pose from Gazebo and adds the antenna height. Nothing here
  // special-cases node 0.
  void OnPositions(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const auto & data = msg->data;
    for (size_t i = 0; i + 3 < data.size(); i += 4)
      {
        uint32_t nsId = static_cast<uint32_t>(data[i]);   // used as-is
        double x = data[i+1], y = data[i+2], z = data[i+3];

        if (nsId >= m_nNodes)
          {
            // Loud on purpose -- see the note on the id convention above.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "Position for id=%u but only %u nodes exist. The publisher is "
              "using a different id convention (expected 0=GCS, 1..N=UAVs).",
              nsId, m_nNodes);
            continue;
          }

        // Buffer only. No ns-3 object is created, copied or destroyed here.
        std::lock_guard<std::mutex> lk(g_feedMutex);
        g_pendingPos[nsId] = Vector(x, y, z);
      }
  }

  // [i, j, loss_dB, ...] with i,j = NS-3 node ids (0 = GCS, 1..3 = UAVs).
  //
  // RESOLVED (was a KNOWN GAP here): the ray-caster used to cover UAV<->UAV
  // pairs only, so the three GCS<->UAV links never received a report and were
  // modelled as permanently clear line-of-sight -- the most optimistic
  // possible assumption applied to the link most likely to be blocked.
  // obstacle_raycast_plugin now loops over all n_nodes_ and publishes all 6
  // links. If a link still shows known=0 in the validation CSV, the reports
  // are not arriving -- see CheckIntegration(), which says so explicitly
  // rather than leaving it to be inferred from a zero column.
  void OnObstacleLoss(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const auto & data = msg->data;
    for (size_t i = 0; i + 2 < data.size(); i += 3)
      {
        uint32_t a = static_cast<uint32_t>(data[i]);      // used as-is
        uint32_t b = static_cast<uint32_t>(data[i+1]);
        double lossDb = data[i+2];

        if (a >= m_nNodes || b >= m_nNodes)
          {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "Obstacle loss for pair (%u,%u) but only %u nodes exist. The "
              "publisher is using a different id convention (expected 0=GCS, "
              "1..N=UAVs).", a, b, m_nNodes);
            continue;
          }

        // Buffer only -- queued, not coalesced, so the EMA sees every sample.
        std::lock_guard<std::mutex> lk(g_feedMutex);
        g_pendingLoss.push_back({static_cast<double>(a),
                                 static_cast<double>(b), lossDb});
      }
  }

  uint32_t m_nNodes;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_posSub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_obsSub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_rssiPub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_snrPub;
};

// Null in --standalone mode; PublishStats() guards on it.
static std::shared_ptr<Ns3RosNode> g_rosNode;

// ─────────────────────────────────────────────────────────────────────────────
//  ApplyFeed -- drains the ROS buffers on the NS-3 simulation thread.
//
//  This is where every ns-3 object touch happens, so all Ptr<> refcounting is
//  single-threaded and the non-atomic counter is safe. Runs as an ordinary
//  Simulator event; nothing is scheduled from the ROS thread any more.
//
//  Period must be at least as fast as the publisher (10 Hz) or g_pendingLoss
//  grows without bound. The default 50 Hz gives 5x headroom and costs a
//  mutex-protected swap of two small containers.
// ─────────────────────────────────────────────────────────────────────────────
static void ApplyFeed(NodeContainer nodes,
                      Ptr<DynamicObstacleLossModel> obstacleLoss,
                      double periodSec)
{
  std::map<uint32_t, Vector> pos;
  std::vector<std::array<double, 3>> loss;
  {
    std::lock_guard<std::mutex> lk(g_feedMutex);
    pos.swap(g_pendingPos);     // swap, not copy: hold the lock briefly
    loss.swap(g_pendingLoss);
  }

  for (const auto & kv : pos)
    {
      const uint32_t id = kv.first;
      if (id >= nodes.GetN()) continue;          // already warned on the ROS side
      auto mob = nodes.Get(id)->GetObject<ConstantVelocityMobilityModel>();
      if (mob)
        {
          mob->SetPosition(kv.second);
          g_posSeenMask.fetch_or(1u << id);      // for CheckIntegration()
        }
      else
        {
          // The original code silently dropped the update here. That is the
          // same class of quiet failure the id range check guards against, and
          // it became reachable when the GCS joined the feed: a node left on
          // ConstantPositionMobilityModel accepts no updates and would sit at
          // its CLI default forever while every link to it used the wrong
          // distance.
          NS_LOG_WARN("Position for node " << id << " ignored: its mobility "
            "model is not ConstantVelocityMobilityModel. (--standalone pins "
            "the GCS on purpose; in ROS mode it means the node was installed "
            "with the wrong model.)");
        }
    }

  for (const auto & l : loss)
    {
      obstacleLoss->SetObstacleLoss(static_cast<uint32_t>(l[0]),
                                    static_cast<uint32_t>(l[1]), l[2]);
    }

  Simulator::Schedule(Seconds(periodSec), &ApplyFeed, nodes, obstacleLoss,
                      periodSec);
}

// ─────────────────────────────────────────────────────────────────────────────
//  PublishStats -- per-link metric export + loss decomposition
//  (KEPT from obstacle_loss.cc, with txPower no longer hard-coded.)
//
//    pathloss_rx = Tx - L_pathloss                       (log-distance alone)
//    obstacle_dB = smoothed shadowing                    (from Gazebo)
//    faded_rx    = Tx - L_pathloss - obstacle_dB + fade  (full chain, 1 draw)
//    fading_dB   = faded_rx - (pathloss_rx - obstacle_dB)
//    snr_dB      = faded_rx - noise_floor
//
//  This decomposition is arithmetically valid ONLY because the chain contains
//  exactly one stochastic stage. Adding ns3::NakagamiPropagationLossModel
//  would silently turn fading_delta_db into the sum of two independent fading
//  processes and invalidate every validation plot built on this CSV.
// ─────────────────────────────────────────────────────────────────────────────
void PublishStats(Ptr<DynamicObstacleLossModel> obstacleLoss,
                  Ptr<PropagationLossModel> pathLossOnly,
                  double txPowerDbm,
                  double noiseFloorDbm,
                  double periodSec,
                  NodeContainer nodes)
{
  std::vector<float> rssiFlat, snrFlat;
  const double now = Simulator::Now().GetSeconds();

  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    for (uint32_t j = i + 1; j < nodes.GetN(); ++j)
      {
        Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel>();
        Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel>();
        if (!mobI || !mobJ) continue;

        // Deterministic path-loss-only rx (log-distance on its own).
        const double pathLossRx = pathLossOnly->CalcRxPower(txPowerDbm, mobI, mobJ);
        // Full chain: path loss - obstacle shadowing + one fading draw.
        const double fadedRx    = obstacleLoss->CalcRxPower(txPowerDbm, mobI, mobJ);

        const uint32_t idI = nodes.Get(i)->GetId();
        const uint32_t idJ = nodes.Get(j)->GetId();

        const auto info = obstacleLoss->GetLinkLossInfo(idI, idJ);
        const double deterministicRx = pathLossRx - info.obstacleLossDb;
        const double fadingDeltaDb   = fadedRx - deterministicRx;
        const double snrDb           = fadedRx - noiseFloorDbm;
        const double distance        = mobI->GetDistanceFrom(mobJ);

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

  if (g_csv.is_open()) g_csv.flush();

  if (g_rosNode)
    {
      g_rosNode->PublishRssi(rssiFlat);
      g_rosNode->PublishSnr(snrFlat);
    }

  Simulator::Schedule(Seconds(periodSec), &PublishStats, obstacleLoss,
                      pathLossOnly, txPowerDbm, noiseFloorDbm, periodSec, nodes);
}

// ─────────────────────────────────────────────────────────────────────────────
//  CheckIntegration -- retrying co-simulation sanity report.
//
//  ADDED with the GCS. Every way this integration breaks looks identical from
//  the outside: the simulation runs happily and produces plausible numbers
//  that are quietly wrong. Concretely, if node 0's position or the GCS link
//  reports never arrive, NS-3 leaves the station at its CLI default and treats
//  its links as clear line-of-sight -- which is exactly what a correctly
//  working unobstructed scenario looks like.
//
//  Failure causes this catches, all of which are one-line misconfigurations:
//    - world has no <model name="gcs">      (position publisher warns, but in
//                                            a different terminal)
//    - <gcs_enabled>false</gcs_enabled>     in the world's plugin block
//    - world_pos_publisher started with gcs_enabled:=false
//    - a publisher still on the legacy numbering (ids 0..2 = the UAVs)
//    - the ROS graph simply not connected (wrong ROS_DOMAIN_ID, etc.)
//
//  Starts at about 10 s and retries while Gazebo and its ROS publishers come
//  up.  Only the NS-3 simulation thread reads or applies the buffered feed.
// ─────────────────────────────────────────────────────────────────────────────
static void CheckIntegration(NodeContainer nodes,
                             Ptr<DynamicObstacleLossModel> obstacleLoss,
                             double finalTimeoutSec)
{
  const uint32_t mask = g_posSeenMask.load();
  std::ostringstream missingPos, missingLoss;

  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    if (!(mask & (1u << i)))
      missingPos << ' ' << (i == 0 ? "GCS(0)" : "UAV" + std::to_string(i)
                                                + "(" + std::to_string(i) + ")");

  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    for (uint32_t j = i + 1; j < nodes.GetN(); ++j)
      if (!obstacleLoss->GetLinkLossInfo(nodes.Get(i)->GetId(),
                                         nodes.Get(j)->GetId()).known)
        missingLoss << " (" << i << ',' << j << ')';

  if (missingPos.str().empty() && missingLoss.str().empty())
    {
      NS_LOG_UNCOND("[integration check] OK: positions for all "
                    << nodes.GetN() << " nodes and obstacle reports for all "
                    << (nodes.GetN() * (nodes.GetN() - 1) / 2)
                    << " links have been received.");
      return;
    }

  const double now = Simulator::Now().GetSeconds();
  if (now < finalTimeoutSec)
    {
      NS_LOG_UNCOND("[integration check] WAITING: missing node IDs:"
                    << (missingPos.str().empty() ? " none" : missingPos.str())
                    << "; missing link pairs:"
                    << (missingLoss.str().empty() ? " none" : missingLoss.str()));
      Simulator::Schedule(Seconds(2.0), &CheckIntegration, nodes, obstacleLoss,
                          finalTimeoutSec);
      return;
    }

  NS_LOG_UNCOND("[integration check] FAILED: timed out after "
                << finalTimeoutSec << " seconds.");
  NS_LOG_UNCOND("  missing node IDs:"
                << (missingPos.str().empty() ? " none" : missingPos.str()));
  NS_LOG_UNCOND("  missing link pairs:"
                << (missingLoss.str().empty() ? " none" : missingLoss.str()));
}

// ─────────────────────────────────────────────────────────────────────────────
//  Periodic position / inter-node distance log (KEPT from rt_new.cc, extended
//  to cover the GCS). Cheap, and invaluable when a link goes dead and you need
//  to know whether it was geometry or the channel.
// ─────────────────────────────────────────────────────────────────────────────
static void LogNodePositions(NodeContainer nodes, double periodSec)
{
  const double now = Simulator::Now().GetSeconds();
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1) << "t=" << now << "s ";
  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
      Ptr<MobilityModel> mm = nodes.Get(i)->GetObject<MobilityModel>();
      if (!mm) continue;
      const Vector p = mm->GetPosition();
      oss << (i == 0 ? "GCS" : "UAV" + std::to_string(i))
          << "=(" << p.x << ',' << p.y << ',' << p.z << ") ";
    }
  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    for (uint32_t j = i + 1; j < nodes.GetN(); ++j)
      {
        Ptr<MobilityModel> a = nodes.Get(i)->GetObject<MobilityModel>();
        Ptr<MobilityModel> b = nodes.Get(j)->GetObject<MobilityModel>();
        if (a && b) oss << " d" << i << j << '=' << a->GetDistanceFrom(b) << "m";
      }
  NS_LOG_UNCOND(oss.str());
  Simulator::Schedule(Seconds(periodSec), &LogNodePositions, nodes, periodSec);
}

// ─────────────────────────────────────────────────────────────────────────────
//  NetAnim helpers (KEPT from rt_new.cc, but NetAnim is now off by default --
//  see --enableNetAnim in main()).
// ─────────────────────────────────────────────────────────────────────────────
static void RestoreNodeColor(AnimationInterface* anim, uint32_t nodeId,
                             uint8_t r, uint8_t g, uint8_t b)
{
  if (anim) anim->UpdateNodeColor(nodeId, r, g, b);
}

static void FlashNodeOnPhyRxEnd(AnimationInterface* anim, uint32_t nodeId,
                                uint8_t oR, uint8_t oG, uint8_t oB,
                                Ptr<const Packet> packet)
{
  (void)packet;
  if (!anim) return;
  anim->UpdateNodeColor(nodeId, 255, 200, 0);
  Simulator::Schedule(Seconds(0.1), &RestoreNodeColor, anim, nodeId, oR, oG, oB);
}

// =============================================================================
//  main()
// =============================================================================
int main(int argc, char* argv[])
{
  // ── Command line ─────────────────────────────────────────────────────────
  // Number of radio nodes BESIDES the GCS -- not the number of aircraft.
  // Each aircraft needs TWO: its autopilot (SITL, in a namespace) and its
  // companion computer (a Pi). They cannot share one, because TapBridge in
  // UseLocal mode tracks a single MAC address per TAP. So two aircraft with
  // onboard vision is nRadios=4:
  //     node 0 GCS | node 1 SITL1 | node 2 Pi1 | node 3 SITL2 | node 4 Pi2
  uint32_t nRadios = 3;
  std::vector<std::string> tapNames;    // filled in after the command line is parsed

  double      simTime        = 0.0;    // 0 = run until killed (live mission)
  double      distance       = 50.0;   // initial UAV separation, m
  double      uavAltitude    = 20.0;   // m AGL
  double      txPowerDbm     = 20.0;   // typical UAV datalink radio
  double      rxSensitivity  = -82.0;  // 802.11a default
  // Noise floor for SNR: kTB over a 20 MHz 802.11a channel plus a ~7 dB
  // receiver noise figure => -174 + 10*log10(20e6) + 7 = -94 dBm.
  // (802.11a and 802.11n HT20 are both 20 MHz, so the number is unchanged
  //  from the obstacle_loss script -- only the justification differs.)
  double      noiseFloor     = -94.0;
  double      statsPeriod    = 0.5;    // s, ROS publish + validation CSV rate
  double      posLogPeriod   = 2.0;    // s, 0 = off

  // Obstacle model knobs -- previously reachable only by editing C++.
  double      mLos           = 3.0;
  double      mNlos          = 1.0;
  double      emaAlpha       = 0.3;
  double      blockThreshDb  = 3.0;
  double      clearThreshDb  = 1.0;

  bool        standalone     = false;  // true = no ROS, Gauss-Markov mobility
  double      uavSpeed       = 5.0;    // m/s, standalone mobility only

  // Initial GCS position. In ROS mode this is only a placeholder until the
  // first /uav_world_positions message arrives; it must nonetheless be sane
  // because path loss is computed from it during the first ~0.1 s, and it is
  // what the GCS stays at if the feed never arrives (CheckIntegration() says
  // so loudly at t=10 s rather than letting it pass as a working run).
  // In --standalone it is the permanent position.
  //
  // Defaults match <model name="gcs"> in multi_uav_plugin.world: pose (0,6,0)
  // plus the 2.9 m antenna height. small_city_base.world puts the station at
  // (-24,0,0) instead, so pass --gcsX=-24 --gcsY=0 when using that world.

  // Using small_city_base.world as the default
  double      gcsX           = -24.0;
  double      gcsY           = 0.0;
  double      gcsZ           = 0.0;

  double      delayMs        = 0.0;    // 0 = physical speed-of-light delay
  double      lossRate       = 0.0;    // extra burst loss, 0 = off
  bool        enableNetAnim  = false;
  // TapBridge needs root and pre-created TAP devices. Turning it off lets the
  // channel be exercised unprivileged (CI, model debugging) -- otherwise
  // --standalone still aborts in CreateTap() and is useless for that purpose.
  bool        enableTap      = true;
  uint32_t    rngRun         = 1;

  std::string csvPath        = "";
  std::string snrLogFile     = "";
  std::string animFile       = "three_uav_anim.xml";

  CommandLine cmd(__FILE__);
  cmd.AddValue("nRadios",        "Radio nodes besides the GCS (2 per aircraft: "
                                 "SITL + Pi). 2 aircraft = 4.",           nRadios);
  cmd.AddValue("simTime",        "Sim duration s (0=unlimited)",        simTime);
  cmd.AddValue("distance",       "Initial UAV separation m",            distance);
  cmd.AddValue("uavAltitude",    "Initial UAV altitude m",              uavAltitude);
  cmd.AddValue("txPowerDbm",     "Transmit power dBm",                  txPowerDbm);
  cmd.AddValue("rxSensitivity",  "PHY Rx sensitivity dBm",              rxSensitivity);
  cmd.AddValue("noiseFloor",     "Receiver noise floor dBm for SNR",    noiseFloor);
  cmd.AddValue("statsPeriod",    "ROS publish / CSV period s",          statsPeriod);
  cmd.AddValue("posLogPeriod",   "Position log period s (0=off)",       posLogPeriod);
  cmd.AddValue("mLos",           "Nakagami m, clear line-of-sight",     mLos);
  cmd.AddValue("mNlos",          "Nakagami m, obstacle-blocked",        mNlos);
  cmd.AddValue("emaAlpha",       "Obstacle-loss EMA factor (1=none)",   emaAlpha);
  cmd.AddValue("blockThreshDb",  "Loss above this -> NLoS fading",      blockThreshDb);
  cmd.AddValue("clearThreshDb",  "Loss below this -> LoS fading",       clearThreshDb);
  cmd.AddValue("gcsX",           "Initial GCS x (m)",                   gcsX);
  cmd.AddValue("gcsY",           "Initial GCS y (m)",                   gcsY);
  cmd.AddValue("gcsZ",           "Initial GCS antenna height z (m)",    gcsZ);
  cmd.AddValue("standalone",     "Run without ROS (Gauss-Markov mob.)", standalone);
  cmd.AddValue("uavSpeed",       "UAV speed m/s (standalone only)",     uavSpeed);
  cmd.AddValue("delayMs",        "Fixed extra prop delay ms (0=phys.)", delayMs);
  cmd.AddValue("lossRate",       "Extra burst error rate [0-1], 0=off", lossRate);
  cmd.AddValue("enableNetAnim",  "Write NetAnim XML",                   enableNetAnim);
  cmd.AddValue("enableTap",      "Install TapBridge (needs root+TAPs)", enableTap);
  cmd.AddValue("rngRun",         "RNG run number (reproducibility)",    rngRun);
  cmd.AddValue("csvPath",        "Per-link validation CSV (empty=off)", csvPath);
  cmd.AddValue("snrLogFile",     "Per-packet SNR CSV (empty=off)",      snrLogFile);
  cmd.AddValue("animFile",       "NetAnim XML output path",             animFile);
  cmd.Parse(argc, argv);

  // TAP names follow the node numbering: node 0 is the GCS, nodes 1..nRadios are
  // aircraft. Generated rather than passed in, so adding a node cannot leave
  // the names and the node count disagreeing.
  if (nRadios < 1) { NS_FATAL_ERROR("nRadios must be >= 1"); }
  tapNames.push_back("tap-gcs");
  for (uint32_t i = 1; i <= nRadios; ++i)
    { tapNames.push_back("tap-uav" + std::to_string(i)); }
  g_phyStats.resize(nRadios + 1);

  // ADDED: explicit RNG run control. Without it, two "identical" runs draw the
  // same fading sequence, which quietly understates variance in any averaged
  // result you put in a paper.
  RngSeedManager::SetRun(rngRun);

  // ── Sanity checks that would otherwise fail silently ─────────────────────
  if (mLos < 0.5 || mNlos < 0.5)
    { NS_FATAL_ERROR("Nakagami m must be >= 0.5 (m<0.5 is not a valid shape)"); }
  if (clearThreshDb >= blockThreshDb)
    { NS_FATAL_ERROR("clearThreshDb must be < blockThreshDb, otherwise the "
                     "LoS/NLoS hysteresis window is inverted and will flap"); }
  if (rxSensitivity < noiseFloor)
    { NS_LOG_WARN("rxSensitivity is below the stated noise floor -- SNR values "
                  "in the CSV will not correspond to the PHY's decode decisions"); }

  // ── Validation CSV header ────────────────────────────────────────────────
  if (!csvPath.empty())
    {
      g_csv.open(csvPath);
      if (g_csv.is_open())
        g_csv << "t_sim,node_a,node_b,distance_m,pathloss_rx_dbm,"
                 "obstacle_loss_db,faded_rx_dbm,fading_delta_db,snr_db,"
                 "blocked,fading_m,known\n";
      else
        NS_LOG_WARN("Could not open validation CSV: " << csvPath);
    }

  // ── Global config ────────────────────────────────────────────────────────
  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

  // ── Nodes: 0 = GCS, 1..nRadios = radios (SITL and Pi, 2 per aircraft) ────
  NodeContainer nodes;
  nodes.Create(nRadios + 1);

  NodeContainer uavNodes;
  for (uint32_t i = 1; i<= nRadios; ++i) uavNodes.Add(nodes.Get(i));

  // ── Mobility ─────────────────────────────────────────────────────────────
  //
  //  ROS mode (default): ALL FOUR nodes get ConstantVelocityMobilityModel and
  //  are driven entirely by /uav_world_positions -- including the GCS, which
  //  Gazebo now publishes as id 0.
  //
  //  CHANGED: the GCS used to be ConstantPositionMobilityModel with its
  //  coordinates hard-coded here. That is a second source of truth for where
  //  the ground station is, and if it drifted from the Gazebo world pose then
  //  NS-3's path loss and the ray-caster's occlusion would silently describe
  //  two different geometries. Taking the position from the same feed as
  //  everything else makes that impossible. (The OnPositions callback looks up
  //  ConstantVelocityMobilityModel specifically, so a ConstantPosition GCS
  //  would also have silently ignored every update.)
  //
  //  Gauss-Markov is NOT used in ROS mode -- both it and the ROS feed write
  //  position, so whichever fired last would win and the trajectory would
  //  become nondeterministic garbage.
  //
  //  --standalone: GCS pinned at the configured spot, UAVs on Gauss-Markov, so
  //  the channel can be exercised in CI or debugged without bringing up Gazebo.
  MobilityHelper gcsMob;
  Ptr<ListPositionAllocator> gcsPos = CreateObject<ListPositionAllocator>();
  gcsPos->Add(Vector(gcsX, gcsY, gcsZ));
  gcsMob.SetPositionAllocator(gcsPos);
  gcsMob.SetMobilityModel(standalone ? "ns3::ConstantPositionMobilityModel"
                                     : "ns3::ConstantVelocityMobilityModel");
  gcsMob.Install(nodes.Get(0));

  MobilityHelper uavMob;
  Ptr<ListPositionAllocator> uavPos = CreateObject<ListPositionAllocator>();
  
  // Evenly spaced on a circle so any node count gets distinct start positions.
  // Only used before the ROS feed arrives (and for the whole run in standalone).
  for (uint32_t i = 0; i < nRadios; ++i)
    {
      double a = 2.0 * M_PI * i / nRadios;
      uavPos->Add(Vector(distance * std::cos(a), distance * std::sin(a), uavAltitude));
    }

  uavMob.SetPositionAllocator(uavPos);

  if (standalone)
    {
      const double halfXY = 200.0;
      const double altMin = std::max(1.0, uavAltitude - 5.0);
      const double altMax = uavAltitude + 5.0;
      uavMob.SetMobilityModel(
        "ns3::GaussMarkovMobilityModel",
        "Bounds",   BoxValue(Box(-halfXY, halfXY, -halfXY, halfXY, altMin, altMax)),
        "TimeStep", TimeValue(Seconds(0.5)),
        "Alpha",    DoubleValue(0.85),
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
      uavMob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    }
  uavMob.Install(uavNodes);

  // ─────────────────────────────────────────────────────────────────────────
  //  CHANNEL CHAIN   DynamicObstacleLossModel -> LogDistance
  //
  //  Built by hand rather than with YansWifiChannelHelper::AddPropagationLoss,
  //  because PublishStats() needs direct Ptr handles to both models to compute
  //  the loss decomposition. The helper hides them.
  //
  //  ── Why ns3::NakagamiPropagationLossModel is NOT in this chain ──
  //
  //  1. It selects m by DISTANCE (m0 near / m1 mid / m2 far). That heuristic
  //     exists because stock NS-3 has no idea what is between two nodes. We
  //     do -- Gazebo ray-casts it. So the built-in model gets it backwards in
  //     both directions: a 120 m open-sky link is given Rayleigh fading, while
  //     a 30 m link through a wall is given "dominant path" fading.
  //
  //  2. Double-penalising blocked links. Nakagami as implemented is
  //     mean-preserving in linear power, so it is not extra average
  //     attenuation -- it is extra VARIANCE. A lower m fattens the lower tail.
  //     A blocked link would then pay twice: once in mean power (Gazebo's dB
  //     penalty) and once in deep-fade probability (Rayleigh regime). PDR
  //     collapses harder than either model intends and you cannot attribute it.
  //
  //  3. Two stochastic stages destroy the loss decomposition. The
  //     fading_delta_db column in the validation CSV is only meaningful if the
  //     chain has exactly ONE random stage. Add a second and that column
  //     becomes the sum of two independent Gamma processes -- far heavier
  //     tailed than either, and matching no distribution the model implements.
  //
  //  DynamicObstacleLossModel therefore does its own Nakagami draw internally
  //  with m switched by the ray-caster's LoS/NLoS state (MLos / MNlos) rather
  //  than by distance. It is a real Nakagami channel -- just correctly informed.
  //
  //  Chain ORDER (fading before path loss) looks wrong but is harmless: both
  //  stages are multiplicative in linear power and multiplication commutes.
  // ─────────────────────────────────────────────────────────────────────────
  Ptr<DynamicObstacleLossModel> obstacleLoss = CreateObject<DynamicObstacleLossModel>();
  obstacleLoss->SetAttribute("MLos",             DoubleValue(mLos));
  obstacleLoss->SetAttribute("MNlos",            DoubleValue(mNlos));
  obstacleLoss->SetAttribute("EmaAlpha",         DoubleValue(emaAlpha));
  obstacleLoss->SetAttribute("BlockThresholdDb", DoubleValue(blockThreshDb));
  obstacleLoss->SetAttribute("ClearThresholdDb", DoubleValue(clearThreshDb));

  Ptr<LogDistancePropagationLossModel> logDist =
      CreateObject<LogDistancePropagationLossModel>();
  // Exponent 2.0: UAV-to-UAV air-to-air links are close to free space. (The
  // 2.7 "urban ground-level" value from the original script was wrong for this
  // geometry; the 3.0 in rt_new.cc likewise.)
  logDist->SetAttribute("Exponent",          DoubleValue(2.0));
  logDist->SetAttribute("ReferenceDistance", DoubleValue(1.0));
  // 46.73 dB = Friis at 1 m for 802.11a channel 36 (5180 MHz):
  //   lambda = c / 5.18e9 = 0.05788 m
  //   FSPL(1m) = 20*log10(4*pi*1/lambda) = 46.73 dB
  // rt_new.cc used 47.3 dB, which corresponds to ~5.5 GHz -- not a channel in
  // use here. The obstacle_loss value (46.67) was the closer of the two.
  logDist->SetAttribute("ReferenceLoss",     DoubleValue(46.73));

  obstacleLoss->SetNext(logDist);

  // REJECTED: ns3::RandomPropagationLossModel as "log-normal shadowing"
  // (Layer 4 in rt_new.cc). Two independent reasons:
  //   a) It duplicates the obstacle loss. Gazebo MEASURES occlusion; this
  //      GUESSES at it. Stacking them re-creates the double-penalty problem.
  //   b) Independently, it is the wrong tool even without an obstacle model:
  //      RandomPropagationLossModel draws a fresh independent variate on EVERY
  //      call, with no spatial or temporal correlation. Shadowing is by
  //      definition slowly varying and correlated over metres of movement.
  //      What that layer actually adds is per-packet white noise labelled
  //      "shadowing".

  // ── Propagation delay ────────────────────────────────────────────────────
  //  Default is the physical speed-of-light model. rt_new.cc defaulted to a
  //  fixed 20 ms, which is not propagation (real delay over 50 m is ~167 ns)
  //  but a stand-in for end-to-end latency. Since the goal is realistic
  //  comms, latency should EMERGE from MAC contention, queueing and retries --
  //  hard-coding it overrides exactly what NS-3 is here to model.
  //  --delayMs is retained (default 0 = off) for deliberately emulating a
  //  satellite or relay hop.
  Ptr<PropagationDelayModel> delayModel;
  if (delayMs > 0.0)
    {
      Ptr<ConstantSpeedPropagationDelayModel> d =
          CreateObject<ConstantSpeedPropagationDelayModel>();
      // Approximate a fixed delay by slowing "light" -- avoids carrying a
      // custom PropagationDelayModel subclass just for a debug knob.
      d->SetAttribute("Speed", DoubleValue(1.0 / (delayMs * 1e-3)));
      delayModel = d;
      NS_LOG_UNCOND("Artificial fixed propagation delay ~" << delayMs << " ms enabled");
    }
  else
    {
      delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
    }

  Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
  channel->SetPropagationLossModel(obstacleLoss);
  channel->SetPropagationDelayModel(delayModel);

  // Seed the whole loss chain from one stream so runs are reproducible.
  obstacleLoss->AssignStreams(1000);

  // ── PHY / MAC: 802.11a ad-hoc ────────────────────────────────────────────
  YansWifiPhyHelper phy;
  phy.SetChannel(channel);
  phy.Set("TxPowerStart",  DoubleValue(txPowerDbm));
  phy.Set("TxPowerEnd",    DoubleValue(txPowerDbm));
  phy.Set("RxSensitivity", DoubleValue(rxSensitivity));
  phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO); // for sniffer

  WifiMacHelper mac;
  mac.SetType("ns3::AdhocWifiMac");

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211a);   // matches the paper's final stack
  // ConstantRate, not IdealWifiManager:
  //   - IdealWifiManager needs per-station SNR feedback and is known to trip
  //     assertions on 802.11a ad-hoc in ns-3.36-3.38.
  //   - More importantly, rate adaptation creates a feedback loop between the
  //     channel model and the throughput measurement, which makes results hard
  //     to attribute. A fixed rate keeps the experiment clean.
  //   - 6 Mbps is the most robust OFDM rate and sits comfortably above the
  //     ~1.4 Mbps realtime application ceiling, so the PHY rate is never the
  //     bottleneck -- the channel is.
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                               "DataMode",    StringValue("OfdmRate6Mbps"),
                               "ControlMode", StringValue("OfdmRate6Mbps"));

  NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

  // ── Optional extra burst loss ────────────────────────────────────────────
  //  Cheap and occasionally useful for testing application resilience
  //  independent of the channel. NEVER enable it while collecting
  //  channel-validation data -- it contaminates the exact measurement.
  if (lossRate > 0.0)
    {
      for (uint32_t i = 0; i < devices.GetN(); ++i)
        {
          Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
          em->SetAttribute("ErrorRate", DoubleValue(lossRate));
          devices.Get(i)->SetAttribute("ReceiveErrorModel", PointerValue(em));
        }
      NS_LOG_WARN("Extra burst error model active (lossRate=" << lossRate
                  << ") -- validation CSV is NOT clean while this is on");
    }

  // ── Internet stack + addressing ──────────────────────────────────────────
  //  .10 = GCS, .11/.12/.13 = UAV1/2/3
  InternetStackHelper internet;
  internet.Install(nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.42.0.0", "255.255.255.0", "0.0.0.10");
  Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

  // REJECTED: FlowMonitor (rt_new.cc). TapBridge UseLocal bridges the Linux
  // netns to the WifiNetDevice at layer 2, so traffic never traverses the NS-3
  // Ipv4L3Protocol where FlowMonitor installs its probes. InstallAll() would
  // report zero flows and look like a dead link. PHY-level counters below do
  // the same job at a layer that actually sees the packets.

  // ── TapBridge ────────────────────────────────────────────────────────────
  for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
      if (enableTap)
        {
          TapBridgeHelper tap;
          tap.SetAttribute("Mode",       StringValue("UseLocal"));
          tap.SetAttribute("DeviceName", StringValue(tapNames[i]));
          tap.Install(nodes.Get(i), devices.Get(i));
        }
      NS_LOG_UNCOND((i == 0 ? std::string("GCS ") : "UAV" + std::to_string(i))
                    << "  node=" << i
                    << "  tap="  << (enableTap ? tapNames[i] : "<disabled>")
                    << "  ip="   << ifaces.GetAddress(i));
    }
  if (!enableTap)
    NS_LOG_WARN("TapBridge disabled -- no external traffic will cross the "
                "simulated channel. Channel/CSV output is still valid.");

  // ── PHY counters + per-packet SNR sniffer ────────────────────────────────
  for (uint32_t i = 0; i < devices.GetN(); ++i)
    {
      Ptr<WifiNetDevice> wnd = DynamicCast<WifiNetDevice>(devices.Get(i));
      if (!wnd || !wnd->GetPhy()) continue;
      const uint32_t nodeId = nodes.Get(i)->GetId();
      wnd->GetPhy()->TraceConnectWithoutContext("PhyTxEnd",
        MakeBoundCallback(&PhyTxEndCb, nodeId));
      wnd->GetPhy()->TraceConnectWithoutContext("PhyRxEnd",
        MakeBoundCallback(&PhyRxEndCb, nodeId));
      wnd->GetPhy()->TraceConnectWithoutContext("PhyRxDrop",
        MakeBoundCallback(&PhyRxDropCb, nodeId));
    }

  if (!snrLogFile.empty())
    {
      g_snrFile.open(snrLogFile);
      if (g_snrFile.is_open())
        {
          g_snrFile << "time_s,rx_node,freq_mhz,pkt_bytes,signal_dbm,noise_dbm,snr_db\n";
          for (uint32_t i = 0; i < devices.GetN(); ++i)
            {
              std::string ctx = "/NodeList/" + std::to_string(i)
                + "/DeviceList/0/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";
              Config::Connect(ctx, MakeCallback(&MonitorSnifferCallback));
            }
          NS_LOG_UNCOND("Per-packet SNR log -> " << snrLogFile);
        }
      else
        {
          NS_LOG_WARN("Could not open SNR log: " << snrLogFile);
        }
    }

  // ── NetAnim (off by default) ─────────────────────────────────────────────
  //  Useful for demos and paper figures, but AnimationInterface writes on
  //  every position update and EnablePacketMetadata grows the XML without
  //  bound. On a live realtime mission that costs wall-clock time you do not
  //  have at a ~1.4 Mbps ceiling. Enable only for short bounded runs.
  std::unique_ptr<AnimationInterface> anim;
  const std::array<std::array<uint8_t,3>,3> clr = {{{255,0,0},{0,200,0},{0,80,255}}};
  if (enableNetAnim)
    {
      anim = std::make_unique<AnimationInterface>(animFile);
      if (simTime > 0.0 && simTime <= 300.0)
        anim->EnablePacketMetadata(true);
      else
        NS_LOG_UNCOND("NetAnim packet metadata disabled (unbounded/long run)");

      anim->UpdateNodeDescription(nodes.Get(0), "GCS " + tapNames[0]);
      anim->UpdateNodeColor(nodes.Get(0), 255, 255, 255);
      anim->UpdateNodeSize(nodes.Get(0), 7.0, 7.0);
      for (uint32_t i = 0; i < 3; ++i)
        {
          anim->UpdateNodeDescription(nodes.Get(i+1),
            "UAV" + std::to_string(i+1) + " " + tapNames[i+1]);
          anim->UpdateNodeColor(nodes.Get(i+1), clr[i][0], clr[i][1], clr[i][2]);
          anim->UpdateNodeSize(nodes.Get(i+1), 5.0, 5.0);
        }
      for (uint32_t i = 0; i < devices.GetN() && i < 4; ++i)
        {
          Ptr<WifiNetDevice> wnd = DynamicCast<WifiNetDevice>(devices.Get(i));
          if (!wnd || !wnd->GetPhy()) continue;
          uint8_t r = 255, g = 255, b = 255;
          if (i >= 1 && i <= 3) { r = clr[i-1][0]; g = clr[i-1][1]; b = clr[i-1][2]; }
          wnd->GetPhy()->TraceConnectWithoutContext("PhyRxEnd",
            MakeBoundCallback(&FlashNodeOnPhyRxEnd, anim.get(),
                              nodes.Get(i)->GetId(), r, g, b));
        }
    }

  // ── ROS 2 bridge ─────────────────────────────────────────────────────────
  std::thread rosThread;
  if (!standalone)
    {
      rclcpp::init(argc, argv);
      g_rosNode = std::make_shared<Ns3RosNode>(nodes.GetN());
      // rclcpp::spin() blocks forever, so it gets its own thread.
      rosThread = std::thread([]() {
        NS_LOG_UNCOND(">>> [ROS 2] spin thread started <<<");
        rclcpp::spin(g_rosNode);
      });
    }
  else
    {
      NS_LOG_UNCOND(">>> standalone mode: no ROS, Gauss-Markov mobility <<<");
    }

  // ── Scheduled work ───────────────────────────────────────────────────────
  // Drain the ROS feed buffers on the simulation thread at 50 Hz (see the
  // threading rule above g_pendingPos). Only needed when ROS is running.
  if (!standalone)
    Simulator::Schedule(Seconds(0.02), &ApplyFeed, nodes, obstacleLoss, 0.02);

  Simulator::Schedule(Seconds(statsPeriod), &PublishStats, obstacleLoss,
                      Ptr<PropagationLossModel>(logDist), txPowerDbm,
                      noiseFloor, statsPeriod, nodes);

  if (posLogPeriod > 0.0)
    Simulator::Schedule(Seconds(posLogPeriod), &LogNodePositions,
                        nodes, posLogPeriod);

  // Retrying co-simulation sanity report. Gazebo is deliberately launched
  // after NS-3, so a temporary absence at t=10 s is expected.
  if (!standalone)
    {
      const double checkAt = (simTime > 0.0 && simTime < 10.0)
                             ? simTime * 0.9 : 10.0;
      const double finalTimeout = 90.0;
      Simulator::Schedule(Seconds(checkAt), &CheckIntegration, nodes,
                          obstacleLoss, finalTimeout);
    }

  Simulator::Stop(simTime > 0.0 ? Seconds(simTime) : Seconds(3600.0 * 24));

  // ── Run ──────────────────────────────────────────────────────────────────
  NS_LOG_UNCOND("=== integrated UAV simulation ===");
  NS_LOG_UNCOND("  PHY      : 802.11a adhoc, ConstantRate OfdmRate6Mbps, "
                << txPowerDbm << " dBm, RxSens " << rxSensitivity << " dBm");
  NS_LOG_UNCOND("  Channel  : DynamicObstacleLoss(mLos=" << mLos
                << ", mNlos=" << mNlos << ", ema=" << emaAlpha
                << ", block/clear=" << blockThreshDb << '/' << clearThreshDb
                << ") -> LogDistance(n=2.0, Lref=46.73dB @1m)");
  NS_LOG_UNCOND("  Node ids : 0 = GCS, 1-3 = UAV1-3 -- ROS/Gazebo ids are used "
                "as-is, no offset");
  NS_LOG_UNCOND("  Links    : " << (nodes.GetN() * (nodes.GetN() - 1) / 2)
                << " (3 GCS<->UAV + 3 UAV<->UAV)"
                << (standalone ? ""
                               : "; integration checked from t=10s to t=90s"));
  NS_LOG_UNCOND("  GCS init : (" << gcsX << ", " << gcsY << ", " << gcsZ << ") m"
                << (standalone ? "  [pinned]"
                               : "  [placeholder until Gazebo reports node 0]"));
  NS_LOG_UNCOND("  App rate : keep offered load below ~1.4 Mbps (realtime ceiling)");

  Simulator::Run();

  // ── Teardown ─────────────────────────────────────────────────────────────
  g_rosRunning = false;
  if (!standalone)
    {
      rclcpp::shutdown();   // makes rclcpp::spin() return
      if (rosThread.joinable()) rosThread.join();  // unwind BEFORE Destroy()
      g_rosNode.reset();
    }

  // ── PHY-level delivery summary (the FlowMonitor replacement) ─────────────
  {
    const double dur = Simulator::Now().GetSeconds();
    uint64_t totTx = 0, totRx = 0, totDrop = 0, totRxBytes = 0;
    NS_LOG_UNCOND("\n=== PHY delivery summary ===");
    for (uint32_t i = 0; i < g_phyStats.size(); ++i)
      {
        const auto & s = g_phyStats[i];
        totTx += s.txPackets; totRx += s.rxOkPackets;
        totDrop += s.rxDropped; totRxBytes += s.rxOkBytes;
        NS_LOG_UNCOND("  " << (i == 0 ? "GCS " : "UAV" + std::to_string(i))
          << "  tx=" << s.txPackets << " (" << s.txBytes << "B)"
          << "  rxOk=" << s.rxOkPackets << " (" << s.rxOkBytes << "B)"
          << "  rxDrop=" << s.rxDropped);
      }
    const double rxMbps = dur > 0 ? totRxBytes * 8.0 / (dur * 1e6) : 0.0;
    const double dropPct = (totRx + totDrop) > 0
                           ? 100.0 * totDrop / (totRx + totDrop) : 0.0;
    NS_LOG_UNCOND("  TOTAL tx=" << totTx << " rxOk=" << totRx
      << " rxDrop=" << totDrop
      << "  drop=" << std::fixed << std::setprecision(2) << dropPct << "%"
      << "  aggregate rx=" << std::setprecision(3) << rxMbps << " Mbps over "
      << std::setprecision(1) << dur << " s");
  }

  if (g_csv.is_open())     g_csv.close();
  if (g_snrFile.is_open()) g_snrFile.close();
  if (enableNetAnim)       NS_LOG_UNCOND("NetAnim XML -> " << animFile);

  anim.reset();          // flush NetAnim before the simulator dies
  Simulator::Destroy();
  return 0;
}

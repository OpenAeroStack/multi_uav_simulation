#include "dynamic_obstacle_loss_model.hh"
#include "ns3/node.h"
#include "ns3/mobility-module.h"
#include "ns3/double.h"
#include <cmath>

namespace ns3 {

TypeId DynamicObstacleLossModel::GetTypeId()
{
  static TypeId tid = TypeId("ns3::DynamicObstacleLossModel")
    .SetParent<PropagationLossModel>()
    .SetGroupName("Propagation")
    .AddConstructor<DynamicObstacleLossModel>()
    // ADDED: LoS-aware fading + smoothing knobs
    .AddAttribute("MLos",
                  "Nakagami m when the ray-caster reports clear line-of-sight "
                  "(Rician-like: strong dominant path, stable signal).",
                  DoubleValue(3.0),
                  MakeDoubleAccessor(&DynamicObstacleLossModel::m_mLos),
                  MakeDoubleChecker<double>(0.5))
    .AddAttribute("MNlos",
                  "Nakagami m when the link is blocked by an obstacle "
                  "(m = 1.0 is Rayleigh: no dominant path, heavy scattering).",
                  DoubleValue(1.0),
                  MakeDoubleAccessor(&DynamicObstacleLossModel::m_mNlos),
                  MakeDoubleChecker<double>(0.5))
    .AddAttribute("EmaAlpha",
                  "Exponential-smoothing factor applied to obstacle loss "
                  "reports (1.0 = no smoothing).",
                  DoubleValue(0.3),
                  MakeDoubleAccessor(&DynamicObstacleLossModel::m_emaAlpha),
                  MakeDoubleChecker<double>(0.0, 1.0))
    .AddAttribute("BlockThresholdDb",
                  "Smoothed obstacle loss above this switches fading to NLoS.",
                  DoubleValue(3.0),
                  MakeDoubleAccessor(&DynamicObstacleLossModel::m_blockThresholdDb),
                  MakeDoubleChecker<double>())
    .AddAttribute("ClearThresholdDb",
                  "Smoothed obstacle loss below this switches fading back to LoS.",
                  DoubleValue(1.0),
                  MakeDoubleAccessor(&DynamicObstacleLossModel::m_clearThresholdDb),
                  MakeDoubleChecker<double>());
  return tid;
}

DynamicObstacleLossModel::DynamicObstacleLossModel()
{
  m_gammaRv = CreateObject<GammaRandomVariable>();
}

void DynamicObstacleLossModel::SetObstacleLoss(uint32_t nodeIdA, uint32_t nodeIdB, double lossDb)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  auto key = std::make_pair(std::min(nodeIdA, nodeIdB), std::max(nodeIdA, nodeIdB));
  // REMOVED: raw overwrite of the reported loss -- a ray grazing a building
  // edge made the penalty flip 0 -> 15 -> 0 dB between updates:
  // m_obstacleLoss[key] = lossDb;
  // ADDED: exponential smoothing of the reported loss, plus a hysteresis
  // window for the LoS/NLoS fading decision so it cannot flap.
  LinkState & st = m_links[key];
  st.smoothedLossDb = m_emaAlpha * lossDb + (1.0 - m_emaAlpha) * st.smoothedLossDb;
  if (!st.blocked && st.smoothedLossDb > m_blockThresholdDb) {
    st.blocked = true;
  } else if (st.blocked && st.smoothedLossDb < m_clearThresholdDb) {
    st.blocked = false;
  }
}

double DynamicObstacleLossModel::DoCalcRxPower(double txPowerDbm,
                                                Ptr<MobilityModel> a,
                                                Ptr<MobilityModel> b) const
{
  // Step 1: let the NEXT model in the chain (log-distance) compute the
  // "normal" received power based on distance.
  // (The static Nakagami model was REMOVED from the chain -- fading is now
  // applied here in Step 4, with m chosen by line-of-sight state.)
  double rxPowerDbm = const_cast<DynamicObstacleLossModel*>(this)->GetNext()->CalcRxPower(txPowerDbm, a, b);

  // Step 2: find out which two node IDs this call is for.
  Ptr<Node> nodeA = a->GetObject<Node>();
  Ptr<Node> nodeB = b->GetObject<Node>();
  if (!nodeA || !nodeB) return rxPowerDbm;

  uint32_t idA = nodeA->GetId();
  uint32_t idB = nodeB->GetId();
  auto key = std::make_pair(std::min(idA, idB), std::max(idA, idB));

  // Step 3: subtract the (smoothed) obstacle loss Gazebo reported for this
  // pair, and pick the fading regime from the link's LoS/NLoS state.
  double obstacleLossDb = 0.0;
  double m = m_mLos;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_links.find(key);
    if (it != m_links.end()) {
      obstacleLossDb = it->second.smoothedLossDb;
      if (it->second.blocked) {
        m = m_mNlos;
      }
    }
  }
  rxPowerDbm -= obstacleLossDb;

  // Step 4 (ADDED): line-of-sight-aware Nakagami fading. Received power in a
  // Nakagami-m channel is Gamma-distributed with shape m and mean equal to
  // the deterministic rx power (same math as NakagamiPropagationLossModel,
  // but m follows the ray-caster's LoS/NLoS state instead of distance).
  double powerW = std::pow(10.0, (rxPowerDbm - 30.0) / 10.0);
  double fadedPowerW = m_gammaRv->GetValue(m, powerW / m);
  return 10.0 * std::log10(fadedPowerW) + 30.0;
}

int64_t DynamicObstacleLossModel::DoAssignStreams(int64_t stream)
{
  // REMOVED: return 0;  // no random variables of our own to seed
  // ADDED: the fading random variable needs a seedable stream
  m_gammaRv->SetStream(stream);
  return 1;
}

}  // namespace ns3


#include "dynamic_obstacle_loss_model.h"
#include "ns3/node.h"

namespace ns3 {

TypeId DynamicObstacleLossModel::GetTypeId()
{
  static TypeId tid = TypeId("ns3::DynamicObstacleLossModel")
    .SetParent<PropagationLossModel>()
    .SetGroupName("Propagation")
    .AddConstructor<DynamicObstacleLossModel>();
  return tid;
}

void DynamicObstacleLossModel::SetObstacleLoss(uint32_t nodeIdA, uint32_t nodeIdB, double lossDb)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  auto key = std::make_pair(std::min(nodeIdA, nodeIdB), std::max(nodeIdA, nodeIdB));
  m_obstacleLoss[key] = lossDb;
}

double DynamicObstacleLossModel::DoCalcRxPower(double txPowerDbm,
                                                Ptr<MobilityModel> a,
                                                Ptr<MobilityModel> b) const
{
  // Step 1: let the NEXT model in the chain (log-distance, then Nakagami)
  // compute the "normal" received power based on distance and fading.
  double rxPowerDbm = GetNext()->CalcRxPower(txPowerDbm, a, b);

  // Step 2: find out which two node IDs this call is for.
  Ptr<Node> nodeA = a->GetObject<Node>();
  Ptr<Node> nodeB = b->GetObject<Node>();
  if (!nodeA || !nodeB) return rxPowerDbm;

  uint32_t idA = nodeA->GetId();
  uint32_t idB = nodeB->GetId();
  auto key = std::make_pair(std::min(idA, idB), std::max(idA, idB));

  // Step 3: if Gazebo told us there's an obstacle on this exact pair,
  // subtract the extra loss. This is the ONE line that actually
  // "adds the delay" you were looking for -- it makes the signal
  // weaker, which is how an obstacle's effect is realized in NS3.
  std::lock_guard<std::mutex> lock(m_mutex);
  auto it = m_obstacleLoss.find(key);
  if (it != m_obstacleLoss.end()) {
    rxPowerDbm -= it->second;
  }
  return rxPowerDbm;
}

int64_t DynamicObstacleLossModel::DoAssignStreams(int64_t stream)
{
  return 0;  // no random variables of our own to seed
}

}  // namespace ns3

#ifndef DYNAMIC_OBSTACLE_LOSS_MODEL_H
#define DYNAMIC_OBSTACLE_LOSS_MODEL_H

#include "ns3/propagation-loss-model.h"
#include "ns3/node.h"
#include <map>
#include <mutex>

namespace ns3 {

class DynamicObstacleLossModel : public PropagationLossModel
{
public:
  static TypeId GetTypeId();

  // Called by the bridge whenever Gazebo reports a new obstacle reading
  void SetObstacleLoss(uint32_t nodeIdA, uint32_t nodeIdB, double lossDb);

private:
  double DoCalcRxPower(double txPowerDbm,
                        Ptr<MobilityModel> a,
                        Ptr<MobilityModel> b) const override;
  int64_t DoAssignStreams(int64_t stream) override;

  std::map<std::pair<uint32_t,uint32_t>, double> m_obstacleLoss;
  mutable std::mutex m_mutex;
};

}  // namespace ns3
#endif
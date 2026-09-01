
#ifndef DYNAMIC_OBSTACLE_LOSS_MODEL_H
#define DYNAMIC_OBSTACLE_LOSS_MODEL_H

#include "ns3/propagation-loss-model.h"
#include "ns3/random-variable-stream.h"
#include "ns3/node.h"
#include <map>
#include <mutex>

namespace ns3 {

// Obstacle shadowing + line-of-sight-aware Nakagami fading, per UAV pair.
//
// DESIGN CHANGE: a static NakagamiPropagationLossModel used to sit in the
// channel chain with distance-selected m (m = 1.0 past 80 m, i.e. Rayleigh /
// NLoS scattering assumed from distance alone). That double-penalised
// blocked links: Gazebo added an NLoS dB penalty while Nakagami already
// assumed NLoS. Fading is now computed HERE, with the Nakagami m parameter
// driven by the ray-caster's obstacle state instead of distance:
//   clear line-of-sight -> m = MLos  (default 3.0, Rician-like, stable)
//   blocked             -> m = MNlos (default 1.0, Rayleigh, chaotic)
// The raw obstacle loss is exponentially smoothed (EMA) and the LoS/NLoS
// switch has hysteresis, so a ray grazing a building edge cannot make the
// penalty or the fading regime oscillate wildly.
class DynamicObstacleLossModel : public PropagationLossModel
{
public:
  static TypeId GetTypeId();
  DynamicObstacleLossModel();

  // Called by the bridge whenever Gazebo reports a new obstacle reading
  void SetObstacleLoss(uint32_t nodeIdA, uint32_t nodeIdB, double lossDb);

  // ADDED (model validation): deterministic snapshot of a link's current
  // shadowing/fading state. Does NOT draw fading, so it can be logged
  // alongside CalcRxPower() to decompose the total loss into its parts.
  struct LinkLossInfo
  {
    double obstacleLossDb = 0.0;   // smoothed shadowing currently applied (dB)
    bool   blocked        = false; // LoS (false) / NLoS (true) fading regime
    double fadingM        = 0.0;   // Nakagami m in effect (MLos or MNlos)
    bool   known          = false; // false if no report ever seen for this pair
  };
  LinkLossInfo GetLinkLossInfo(uint32_t nodeIdA, uint32_t nodeIdB) const;

private:
  // ADDED: per-link state (smoothed loss + hysteresis LoS/NLoS flag)
  struct LinkState
  {
    double smoothedLossDb = 0.0;
    bool   blocked        = false;
  };

  double DoCalcRxPower(double txPowerDbm,
                        Ptr<MobilityModel> a,
                        Ptr<MobilityModel> b) const override;
  int64_t DoAssignStreams(int64_t stream) override;

  // REMOVED: raw per-pair loss map (no smoothing, no LoS state):
  // std::map<std::pair<uint32_t,uint32_t>, double> m_obstacleLoss;
  // ADDED:
  std::map<std::pair<uint32_t,uint32_t>, LinkState> m_links;
  mutable std::mutex m_mutex;

  // ADDED: LoS-aware fading + smoothing parameters (settable as attributes)
  double m_mLos;              // Nakagami m when line-of-sight is clear
  double m_mNlos;             // Nakagami m when blocked by an obstacle
  double m_emaAlpha;          // EMA smoothing factor for obstacle loss reports
  double m_blockThresholdDb;  // smoothed loss above this -> NLoS fading
  double m_clearThresholdDb;  // smoothed loss below this -> back to LoS fading
  Ptr<GammaRandomVariable> m_gammaRv;  // Nakagami fading (Gamma-distributed power)
};

}  // namespace ns3
#endif

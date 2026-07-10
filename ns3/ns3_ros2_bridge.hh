// multi_uav_simulation/ns3/ns3_ros2_bridge.h
#ifndef NS3_ROS2_BRIDGE_H
#define NS3_ROS2_BRIDGE_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "dynamic_obstacle_loss_model.h"
#include <thread>

namespace ns3 {

// This class owns the socket connection to the Python bridge.
// It runs in its own background thread because reading from a socket
// is a BLOCKING operation — if we did this on NS3's main thread,
// the entire simulation would freeze every time we wait for new data.
class Ns3RosBridge
{
public:
  Ns3RosBridge(Ptr<DynamicObstacleLossModel> lossModel, NodeContainer uavs);
  void Start();   // launches the background thread
  void Stop();

private:
  void ListenLoop();              // runs forever in the background thread
  void HandleMessage(const std::string & jsonLine);

  Ptr<DynamicObstacleLossModel> m_lossModel;
  NodeContainer m_uavs;
  int m_sockFd = -1;
  std::thread m_thread;
  bool m_running = false;
};

}  // namespace ns3
#endif
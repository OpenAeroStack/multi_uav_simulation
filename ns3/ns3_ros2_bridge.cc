
// NS3-ROS2 Bridge (NS3 End)
// This class runs in NS3's process and listens for messages from the Python bridge process.
// It then applies those messages to the NS3 simulation state (e.g. by updating UAV positions or obstacle loss values).

#include "ns3_ros2_bridge.h"
#include "ns3/mobility-module.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <nlohmann/json.hpp>

namespace ns3 {

Ns3RosBridge::Ns3RosBridge(Ptr<DynamicObstacleLossModel> lossModel, NodeContainer uavs)
  : m_lossModel(lossModel), m_uavs(uavs) {}

void Ns3RosBridge::Start()
{
  m_running = true;
  m_thread = std::thread(&Ns3RosBridge::ListenLoop, this);
  m_thread.detach();
}

void Ns3RosBridge::ListenLoop()
{
  m_sockFd = socket(AF_UNIX, SOCK_STREAM, 0);
  struct sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, "/tmp/ns3_uav_bridge.sock", sizeof(addr.sun_path) - 1);

  // Keep retrying until the Python bridge process is up and listening
  while (connect(m_sockFd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::string buf;
  char tmp[4096];
  while (m_running) {
    ssize_t n = recv(m_sockFd, tmp, sizeof(tmp) - 1, 0);
    if (n <= 0) break;
    tmp[n] = '\0';
    buf += tmp;

    size_t pos;
    while ((pos = buf.find('\n')) != std::string::npos) {
      std::string line = buf.substr(0, pos);
      buf = buf.substr(pos + 1);
      HandleMessage(line);
    }
  }
}

void Ns3RosBridge::HandleMessage(const std::string & jsonLine)
{
  auto msg = nlohmann::json::parse(jsonLine, nullptr, false);
  if (msg.is_discarded()) return;  // ignore malformed lines instead of crashing

  if (msg["type"] == "positions") {
    auto & data = msg["data"];
    for (size_t i = 0; i + 3 < data.size(); i += 4) {
      int uid = data[i];
      if (uid < 0 || uid >= (int)m_uavs.GetN()) continue;
      double x = data[i+1], y = data[i+2], z = data[i+3];

      // ScheduleNow is used because we're on a background thread,
      // not NS3's simulation thread. NS3 is single-threaded internally,
      // so all changes to simulation state must be queued as events.
      Simulator::ScheduleNow([this, uid, x, y, z]() {
        auto mob = m_uavs.Get(uid)->GetObject<WaypointMobilityModel>();
        if (mob) {
          mob->AddWaypoint(Waypoint(Simulator::Now(), Vector(x, y, z)));
        }
      });
    }
  }
  else if (msg["type"] == "obstacle_loss") {
    auto & data = msg["data"];
    for (size_t i = 0; i + 2 < data.size(); i += 3) {
      int a = data[i], b = data[i+1];
      double lossDb = data[i+2];

      Simulator::ScheduleNow([this, a, b, lossDb]() {
        m_lossModel->SetObstacleLoss(a, b, lossDb);
      });
    }
  }
}

void Ns3RosBridge::Stop()
{
  m_running = false;
  if (m_sockFd >= 0) close(m_sockFd);
}

}  // namespace ns3
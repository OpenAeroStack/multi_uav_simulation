// gazebo_plugins/include/gazebo_plugins/obstacle_raycast_plugin.hh
#ifndef MULTI_UAV_GAZEBO_PLUGINS__OBSTACLE_RAYCAST_PLUGIN_HH_
#define MULTI_UAV_GAZEBO_PLUGINS__OBSTACLE_RAYCAST_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/marker.pb.h>
#include <ignition/msgs/Utility.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace gazebo
{

// Declares the SHAPE of the class: what it has, not what it does.
// The actual logic ("how do we cast the ray") lives in the .cc file.
class ObstacleRaycastPlugin : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override;
  ~ObstacleRaycastPlugin();

private:
  void UpdatePositions(const std::vector<float> & data);
  void OnUpdate();
  bool GetUavPosition(int id, ignition::math::Vector3d & out);
  double CastRay(const ignition::math::Vector3d & start,
                 const ignition::math::Vector3d & end,
                 int id_a, int id_b);
  double ComputeObstacleLoss(const std::string & entity_name,
                              double hit_dist,
                              const ignition::math::Vector3d & start,
                              const ignition::math::Vector3d & end);
  // ADDED: true material thickness of the obstacle along the link, found by
  // casting a ray backwards from the destination to locate the far face.
  // Returns 0.0 if the exit face can't be resolved (caller uses a fallback).
  double ObstacleThickness(const ignition::math::Vector3d & start,
                           const ignition::math::Vector3d & end,
                           const std::string & entity_name,
                           double entry_dist_from_start);
  void PublishRayMarker(int id_a, int id_b,
                        const ignition::math::Vector3d & start,
                        const ignition::math::Vector3d & end,
                        bool blocked);

  physics::WorldPtr world_;
  ignition::transport::Node ign_node_;  // drives gzclient's /marker service
  int n_uavs_ = 3;
  std::string uav_prefix_;
  event::ConnectionPtr update_conn_;
  common::Time last_check_;

  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr loss_pub_;

  std::mutex pos_mutex_;
  std::map<int, ignition::math::Vector3d> uav_positions_;
  std::thread ros_thread_;
};

}  // namespace gazebo
#endif
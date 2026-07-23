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
#include <vector>

namespace gazebo
{

// ADDED: one row of the material table -- the four ITU-R P.2040-3 constants
// that are ALL that changes with the entity name. Everything downstream
// (L_e, alpha) is computed from these by equations, never tabulated.
//
//   eps'  = a * f_GHz^b        relative permittivity
//   sigma = c * f_GHz^d        conductivity [S/m]
//
// The keyword is matched against the Gazebo entity name by substring, so a
// world author tags a model just by putting the keyword in its <name>.
struct MaterialSpec
{
  const char * keyword;
  double a, b;   // permittivity fit
  double c, d;   // conductivity fit
};

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
  bool GetNodePosition(int id, ignition::math::Vector3d & out);
  bool GetGcsPosition(ignition::math::Vector3d & out);
  bool IsFilteredEntity(const std::string & name) const;
  // ADDED: the single place where a material's RF behaviour is derived.
  // Looks the entity name up in the material table, then runs the two
  // ITU-R P.2040-3 equations on that row's constants to produce the
  // penetration loss L_e (dB) and the bulk attenuation alpha (dB/m).
  // Returns false when no keyword matches -- the caller then treats the
  // obstacle as RF-transparent (zero loss).
  bool MaterialLoss(const std::string & entity_name,
                    double & L_e, double & alpha) const;
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

  // ADDED: ground control station as node 0 (see Load()).
  bool        gcs_enabled_ = true;
  std::string gcs_model_;
  double      gcs_antenna_height_ = 2.9;
  ignition::math::Vector3d gcs_fallback_pos_;
  //int         n_nodes_ = 3;

  // ADDED: the material equations are frequency dependent, so the carrier the
  // NS-3 side is running on has to be known here. Default matches the 802.11a
  // channel used by the scenario (5.18 GHz -> ReferenceLoss 46.67 dB at 1 m).
  double freq_ghz_ = 5.18;
  // Loss beyond this is indistinguishable from a dead link, and it keeps the
  // metal row (sigma = 1e7 S/m) from producing absurd numbers.
  double max_loss_db_ = 80.0;

  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr loss_pub_;

  std::mutex pos_mutex_;
  std::map<int, ignition::math::Vector3d> uav_positions_;
  std::thread ros_thread_;
};

}  // namespace gazebo
#endif
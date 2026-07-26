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
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <atomic>
#include <map>
#include <memory>
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
  ~ObstacleRaycastPlugin() override;

private:
  void UpdatePositions(const std::vector<float> & data);
  void OnUpdate();

  // ADDED (GCS support): node ids now address BOTH the ground station and the
  // UAVs, and are identical to the NS-3 node ids:
  //     id 0        -> GCS   (when <gcs_enabled> is true)
  //     id 1..N     -> UAV models "<uav_prefix>1" .. "<uav_prefix>N"
  // With the GCS disabled the old convention is preserved (id 0 -> UAV 1).
  // REMOVED: bool GetUavPosition(int id, ignition::math::Vector3d & out);
  bool GetNodePosition(int id, ignition::math::Vector3d & out);
  bool GetGcsPosition(ignition::math::Vector3d & out);

  // ADDED: the entity filter used to live inline in BOTH CastRay() and
  // ObstacleThickness(). Adding the GCS model to only one of the two copies
  // would have produced a silently asymmetric ray pair, so it is one function
  // now and both call it.
  bool IsFilteredEntity(const std::string & name) const;

  double CastRay(const ignition::math::Vector3d & start,
                 const ignition::math::Vector3d & end,
                 int id_a, int id_b);
  void LogLinkState(int id_a, int id_b, const std::string & entity_name,
                    const std::string & material, double loss_db);
  std::string MaterialClassification(const std::string & entity_name) const;
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

  // ADDED: ground control station. n_nodes_ = n_uavs_ + 1 when enabled, and
  // the pair loop in OnUpdate() then covers GCS<->UAV links as well as
  // UAV<->UAV ones.
  bool        gcs_enabled_ = true;
  std::string gcs_model_;              // model name to look up in the world
  // Antenna phase centre offset above the model's origin. The RF link starts
  // at the antenna on top of the mast, not at the base of the cabinet -- a
  // ~3 m difference that decides whether low walls and vehicles block the link.
  double      gcs_antenna_height_ = 0.0;
  // Used only if the model is absent from the world AND no position has
  // arrived on /uav_world_positions.
  ignition::math::Vector3d gcs_fallback_pos_;
  int n_nodes_ = 4;
  event::ConnectionPtr update_conn_;
  common::Time last_check_;

  std::shared_ptr<rclcpp::Node> ros_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr loss_pub_;

  // ROS callbacks and the Gazebo update callback run on different threads.
  // Position data has its own short-held lock; the lifecycle lock is a
  // teardown barrier for an OnUpdate invocation already in progress.
  std::mutex pos_mutex_;
  std::map<int, ignition::math::Vector3d> uav_positions_;
  std::mutex update_mutex_;
  std::atomic<bool> stopping_{false};
  std::thread ros_thread_;

  struct LinkLogState
  {
    bool initialized = false;
    bool blocked = false;
    std::string entity;
    std::string material;
    double loss_db = 0.0;
  };
  std::map<std::pair<int, int>, LinkLogState> link_log_states_;
};

}  // namespace gazebo
#endif

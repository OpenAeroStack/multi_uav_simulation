
// Gazebo Ray-Cast Plugin (Full, Scalable to N drones)
//////////////////////////////////////////////////////

// Relative include: CMakeLists adds gazebo_plugins/include to the include path,
// so this resolves on any machine/checkout (was a hardcoded /home/ubuntu/... path).
#include "gazebo_plugins/obstacle_raycast_plugin.hh"

#include <algorithm>
#include <cmath>

namespace gazebo
{

void ObstacleRaycastPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  stopping_.store(false, std::memory_order_release);
  this->world_ = world;
  n_uavs_     = sdf->HasElement("n_uavs") ? sdf->Get<int>("n_uavs") : 3;
  uav_prefix_ = sdf->HasElement("uav_prefix") ? sdf->Get<std::string>("uav_prefix") : "iris_";

  // ADDED: ground control station as node 0.
  gcs_enabled_ = sdf->HasElement("gcs_enabled") ? sdf->Get<bool>("gcs_enabled") : true;
  gcs_model_   = sdf->HasElement("gcs_model")
                 ? sdf->Get<std::string>("gcs_model") : "gcs";
  gcs_antenna_height_ = sdf->HasElement("gcs_antenna_height")
                        ? sdf->Get<double>("gcs_antenna_height") : 2.9;
  gcs_fallback_pos_ = sdf->HasElement("gcs_position")
                      ? sdf->Get<ignition::math::Vector3d>("gcs_position")
                      : ignition::math::Vector3d(0, 0, 0);

  n_nodes_ = n_uavs_ + (gcs_enabled_ ? 1 : 0);

  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  ros_node_ = std::make_shared<rclcpp::Node>("obstacle_raycast_plugin");
  ros_executor_ =
    std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

  pos_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/uav_world_positions", 10,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      this->UpdatePositions(msg->data);
    });

  loss_pub_ = ros_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/link_obstacle_loss", 10);

  ros_executor_->add_node(ros_node_);
  ros_thread_ = std::thread([this]() { ros_executor_->spin(); });

  update_conn_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&ObstacleRaycastPlugin::OnUpdate, this));

  gzmsg << "ObstacleRaycastPlugin loaded: " << n_uavs_ << " UAVs"
        << (gcs_enabled_
              ? " + GCS '" + gcs_model_ + "' (node 0, antenna +"
                + std::to_string(gcs_antenna_height_) + " m)"
              : " (no GCS)")
        << " => " << n_nodes_ << " nodes, "
        << (n_nodes_ * (n_nodes_ - 1) / 2) << " links" << std::endl;
}

ObstacleRaycastPlugin::~ObstacleRaycastPlugin()
{
  // Stop new Gazebo callbacks first, then wait for any callback which was
  // already running. This keeps world_, the publisher, and marker transport
  // alive until OnUpdate has completely returned.
  stopping_.store(true, std::memory_order_release);
  update_conn_.reset();
  {
    std::lock_guard<std::mutex> update_lock(update_mutex_);
  }

  // cancel() wakes spin(); join is the lifetime barrier for the subscription
  // callback's `this` capture. Do not shut down the process-wide ROS context:
  // Gazebo may host other ROS-enabled plugins.
  if (ros_executor_) {
    ros_executor_->cancel();
  }
  if (ros_thread_.joinable()) {
    ros_thread_.join();
  }
  if (ros_executor_ && ros_node_) {
    ros_executor_->remove_node(ros_node_);
  }

  pos_sub_.reset();
  loss_pub_.reset();
  ros_node_.reset();
  ros_executor_.reset();
  world_.reset();
}

void ObstacleRaycastPlugin::UpdatePositions(const std::vector<float> & data)
{
  if (stopping_.load(std::memory_order_acquire)) return;

  std::lock_guard<std::mutex> lock(pos_mutex_);
  if (stopping_.load(std::memory_order_relaxed)) return;
  uav_positions_.clear();
  for (size_t i = 0; i + 3 < data.size(); i += 4) {
    int id = static_cast<int>(data[i]);
    uav_positions_[id] = ignition::math::Vector3d(data[i+1], data[i+2], data[i+3]);
  }
}

void ObstacleRaycastPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> update_lock(update_mutex_);
  if (stopping_.load(std::memory_order_acquire) || !world_) return;

  auto now = world_->SimTime();
  if ((now - last_check_).Double() < 0.1) return;
  last_check_ = now;

  // CHANGED: the loop now runs over ALL nodes, not just UAVs. With the GCS
  // enabled node 0 is the ground station, so this covers the three GCS<->UAV
  // links in addition to the three UAV<->UAV ones. Those GCS links matter
  // most: the station sits at ground level in a city, so it is by far the
  // likeliest link to be occluded -- and until now NS-3 modelled it as
  // permanently clear line-of-sight.
  std_msgs::msg::Float32MultiArray loss_msg;
  for (int i = 0; i < n_nodes_; i++) {
    for (int j = i + 1; j < n_nodes_; j++) {
      ignition::math::Vector3d pos_i, pos_j;
      if (!GetNodePosition(i, pos_i) || !GetNodePosition(j, pos_j)) continue;

      double extra_loss = CastRay(pos_i, pos_j, i, j);
      loss_msg.data.push_back(static_cast<float>(i));
      loss_msg.data.push_back(static_cast<float>(j));
      loss_msg.data.push_back(static_cast<float>(extra_loss));

      PublishRayMarker(i, j, pos_i, pos_j, extra_loss > 0.0);
    }
  }
  if (!loss_msg.data.empty() && loss_pub_) loss_pub_->publish(loss_msg);
}

// Positions received on /uav_world_positions take priority; when the topic
// is silent (e.g. minimal bring-up without the bridge stack), fall back to
// reading the model's pose straight from the Gazebo world.
//
// NODE ID CONVENTION (identical to NS-3 node ids -- no offset anywhere):
//   gcs_enabled_  : id 0 -> GCS,        id k>=1 -> model "<uav_prefix>k"
//   !gcs_enabled_ : id k    -> model "<uav_prefix>(k+1)"   (legacy behaviour)
bool ObstacleRaycastPlugin::GetNodePosition(int id, ignition::math::Vector3d & out)
{
  {
    std::lock_guard<std::mutex> lock(pos_mutex_);
    auto it = uav_positions_.find(id);
    if (it != uav_positions_.end()) {
      out = it->second;
      return true;
    }
  }

  if (gcs_enabled_ && id == 0) return GetGcsPosition(out);

  // Model number for this node id: with the GCS present node 1 is "<prefix>1",
  // so the +1 that used to be here is absorbed by the shifted id.
  const int model_num = gcs_enabled_ ? id : id + 1;
  std::string prefix = uav_prefix_ + std::to_string(model_num);
  for (const auto & model : world_->Models()) {
    if (model->GetName().rfind(prefix, 0) == 0) {
      out = model->WorldPose().Pos();
      return true;
    }
  }
  return false;
}

// The GCS is static, so its pose comes straight from the world. The antenna
// offset is added here because the model's origin is the base of the cabinet
// while the link actually originates at the top of the mast.
bool ObstacleRaycastPlugin::GetGcsPosition(ignition::math::Vector3d & out)
{
  for (const auto & model : world_->Models()) {
    if (model->GetName().rfind(gcs_model_, 0) == 0) {
      out = model->WorldPose().Pos();
      out.Z() += gcs_antenna_height_;
      return true;
    }
  }
  // Model not in the world -- use the configured fallback so the GCS links are
  // still evaluated rather than silently disappearing from the loss message.
  out = gcs_fallback_pos_;
  out.Z() += gcs_antenna_height_;
  return true;
}

// Entities that are never real RF obstacles:
//   - the ground plane
//   - UAV bodies/rotors (own or a fellow fleet member's)
//   - the GCS cabinet and mast (a link must not be blocked by its own antenna
//     support; the structure is small enough that shadowing OTHER links with
//     it is not worth the false positives)
//   - "noloss"-tagged thin street furniture (poles, signs, hydrants, postbox)
// Shared by the forward cast in CastRay() and the backward cast in
// ObstacleThickness() so the two can never diverge.
bool ObstacleRaycastPlugin::IsFilteredEntity(const std::string & name) const
{
  return name.find("ground_plane") != std::string::npos ||
         name.find(uav_prefix_)    != std::string::npos ||
         name.find("noloss")       != std::string::npos ||
         (gcs_enabled_ && name.find(gcs_model_) != std::string::npos);
}

double ObstacleRaycastPlugin::CastRay(const ignition::math::Vector3d & start,
                                       const ignition::math::Vector3d & end,
                                       int id_a, int id_b)
{
  auto ray = boost::dynamic_pointer_cast<physics::RayShape>(
    world_->Physics()->CreateShape("ray", physics::CollisionPtr()));
  if (!ray) return 0.0;
  
  double link_len = (end - start).Length();
  auto dir = (end - start).Normalized();
  double traveled = 0.5;  // skip past the source drone's own body

  // GetIntersection only reports the FIRST hit, so a filtered entity
  // (ground plane, another drone) would mask a real obstacle behind it.
  // Step past filtered hits and cast again until we find a real obstacle
  // or reach the destination UAV.
  for (int hop = 0; hop < 5; hop++) {
    ray->SetPoints(start + dir * traveled, end);
    ray->Update();

    std::string hit_entity;
    double hit_dist;
    ray->GetIntersection(hit_dist, hit_entity);

    // If the ray hit absolutely nothing
    if(hit_entity.empty()) {
      LogLinkState(id_a, id_b, "", "clear", 0.0);
      return 0.0;
    }

    double dist_from_start = traveled + hit_dist;

    // Strict Distance Bound : Ignore hits that are further away than the destination UAV
    if(dist_from_start > (link_len - 0.5)) {
      LogLinkState(id_a, id_b, "", "clear", 0.0);
      return 0.0;
    }

    // Filtered entities (ground, UAV bodies, the GCS structure, "noloss"
    // props) are never real obstacles -- step past and re-cast so they cannot
    // mask a genuine obstacle behind them. See IsFilteredEntity().
    if (IsFilteredEntity(hit_entity)) {
      traveled = dist_from_start + 0.1;
      continue;
    }

    const double loss =
      ComputeObstacleLoss(hit_entity, dist_from_start, start, end);
    LogLinkState(id_a, id_b, hit_entity,
                 MaterialClassification(hit_entity), loss);
    return loss;
  }
  LogLinkState(id_a, id_b, "", "clear", 0.0);
  return 0.0;
}

std::string ObstacleRaycastPlugin::MaterialClassification(
  const std::string & entity_name) const
{
  for (const char * material :
       {"glass", "wood", "concrete", "metal", "foliage", "vehicle"}) {
    if (entity_name.find(material) != std::string::npos) return material;
  }
  return entity_name.empty() ? "clear" : "unclassified";
}

void ObstacleRaycastPlugin::LogLinkState(
  int id_a, int id_b, const std::string & entity_name,
  const std::string & material, double loss_db)
{
  const auto key = std::minmax(id_a, id_b);
  auto & previous = link_log_states_[key];
  const bool blocked = loss_db > 0.0;
  constexpr double kMaterialLossChangeDb = 0.5;
  const bool changed =
    (!previous.initialized && blocked) ||
    (previous.initialized &&
     (previous.blocked != blocked ||
      previous.entity != entity_name ||
      previous.material != material ||
      std::abs(previous.loss_db - loss_db) >= kMaterialLossChangeDb));

  if (changed) {
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Ray %d->%d %s: entity=%s material=%s obstacle_loss=%.2f dB",
      id_a, id_b, blocked ? "blocked" : "clear",
      entity_name.empty() ? "none" : entity_name.c_str(),
      material.c_str(), loss_db);
  }

  previous.initialized = true;
  previous.blocked = blocked;
  previous.entity = entity_name;
  previous.material = material;
  previous.loss_db = loss_db;
}

// Draws each link as a line in gzclient via Gazebo's /marker service:
//   red    = blocked by an obstacle (any link)
//   green  = clear line-of-sight, UAV <-> UAV
//   blue   = clear line-of-sight, GCS <-> UAV   (ADDED: so the three new
//            ground-station links can be told apart at a glance)
// View them in gzclient (markers render automatically, no menu needed).
void ObstacleRaycastPlugin::PublishRayMarker(int id_a, int id_b,
                                              const ignition::math::Vector3d & start,
                                              const ignition::math::Vector3d & end,
                                              bool blocked)
{
  ignition::msgs::Marker marker;
  marker.set_ns("uav_raycast");
  marker.set_id(id_a * 100 + id_b + 1);  // stable non-zero id per pair
  marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
  marker.set_type(ignition::msgs::Marker::LINE_LIST);

  ignition::msgs::Set(marker.add_point(), start);
  ignition::msgs::Set(marker.add_point(), end);

  const bool is_gcs_link = gcs_enabled_ && (id_a == 0 || id_b == 0);
  marker.mutable_material()->mutable_script()->set_name(
    blocked ? "Gazebo/Red" : (is_gcs_link ? "Gazebo/Blue" : "Gazebo/Green"));

  ign_node_.Request("/marker", marker);
}

// Finds how much of the obstacle the ray actually passes through, by casting
// a second ray backwards from the destination towards the source. The first
// real hit on the SAME entity is the obstacle's far (exit) face; the gap
// between entry and exit faces along the link is the true material thickness.
double ObstacleRaycastPlugin::ObstacleThickness(const ignition::math::Vector3d & start,
                                                 const ignition::math::Vector3d & end,
                                                 const std::string & entity_name,
                                                 double entry_dist_from_start)
{
  auto ray = boost::dynamic_pointer_cast<physics::RayShape>(
    world_->Physics()->CreateShape("ray", physics::CollisionPtr()));
  if (!ray) return 0.0;

  double link_len = (end - start).Length();
  auto dir = (end - start).Normalized();
  double traveled = 0.5;  // skip past the destination drone's own body

  for (int hop = 0; hop < 5; hop++) {
    ray->SetPoints(end - dir * traveled, start);   // backward: destination -> source
    ray->Update();

    std::string hit_entity;
    double hit_dist;
    ray->GetIntersection(hit_dist, hit_entity);
    if (hit_entity.empty()) break;

    double dist_from_end     = traveled + hit_dist;
    double exit_dist_from_start = link_len - dist_from_end;

    // step past filtered entities, same rule as the forward cast
    if (IsFilteredEntity(hit_entity)) {
      traveled = dist_from_end + 0.1;
      continue;
    }

    // Only trust it if the far face is the SAME obstacle the forward ray hit
    if (hit_entity == entity_name) {
      double thickness = exit_dist_from_start - entry_dist_from_start;
      if (thickness > 0.0) return thickness;
    }
    break;  // different obstacle, or geometry inverted -> let caller fall back
  }
  return 0.0;
}

double ObstacleRaycastPlugin::ComputeObstacleLoss(const std::string & entity_name,
                                                    double hit_dist,
                                                    const ignition::math::Vector3d & start,
                                                    const ignition::math::Vector3d & end)
{
  // CHANGED: an obstacle whose <name> carries NO recognised material keyword
  // now contributes ZERO loss instead of a default 15 dB. Only explicitly
  // tagged materials attenuate the link; anything untagged is treated as
  // RF-transparent (same net effect as a "noloss" prop). Track whether a
  // keyword actually matched so we can bail out below.
  // REMOVED: double L_e = 15.0;  // default: unknown solid ~ masonry / concrete
  double L_e = 0.0;
  bool   material_known = false;
  if (entity_name.find("glass")    != std::string::npos) { L_e = 4.0;  material_known = true; }
  if (entity_name.find("wood")     != std::string::npos) { L_e = 8.0;  material_known = true; }
  if (entity_name.find("concrete") != std::string::npos) { L_e = 15.0; material_known = true; }
  if (entity_name.find("metal")    != std::string::npos) { L_e = 20.0; material_known = true; }
  // ADDED material classes for the small-city props. A model is tagged by
  // putting one of these keywords in its <name> in the world file, so the
  // material lives with the world (author-controlled) and the plugin stays
  // generic:
  if (entity_name.find("foliage")  != std::string::npos) { L_e = 5.0;  material_known = true; }  // trees: scattering, not a solid wall
  if (entity_name.find("vehicle")  != std::string::npos) { L_e = 12.0; material_known = true; }  // cars: hollow metal shell, ground level

  // No recognised material keyword -> zero loss (skip the thickness term too).
  if (!material_known) return 0.0;

  // REMOVED: "penetration_depth" was the free-space distance from the wall to
  // the destination drone, not the depth of material traversed -- so a drone
  // far behind a wall was penalised more than one just behind it, which is
  // physically backwards:
  // double link_len = (end - start).Length();
  // double penetration_depth = std::min((link_len - 0.5) - hit_dist, 20.0);
  // return L_e + 0.5 * penetration_depth;
  // ADDED: use the TRUE material thickness (entry face -> exit face) found by
  // a backward ray. Model: fixed entry/exit penetration loss L_e plus a bulk
  // attenuation term proportional to how much material the signal crosses
  // (0.5 dB per metre of material, clamped). If the exit face can't be
  // resolved, fall back to the base material loss alone.
  double thickness = ObstacleThickness(start, end, entity_name, hit_dist);
  thickness = std::min(thickness, 20.0);
  return L_e + 0.5 * thickness;
}

GZ_REGISTER_WORLD_PLUGIN(ObstacleRaycastPlugin)
}  // namespace gazebo

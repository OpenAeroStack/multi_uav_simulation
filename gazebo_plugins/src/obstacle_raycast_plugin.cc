
// Gazebo Ray-Cast Plugin (Full, Scalable to N drones)
//////////////////////////////////////////////////////

// Relative include: CMakeLists adds gazebo_plugins/include to the include path,
// so this resolves on any machine/checkout (was a hardcoded /home/ubuntu/... path).
#include "gazebo_plugins/obstacle_raycast_plugin.hh"


namespace gazebo
{

void ObstacleRaycastPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  this->world_ = world;
  n_uavs_     = sdf->HasElement("n_uavs") ? sdf->Get<int>("n_uavs") : 3;
  uav_prefix_ = sdf->HasElement("uav_prefix") ? sdf->Get<std::string>("uav_prefix") : "iris_";

  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  ros_node_ = std::make_shared<rclcpp::Node>("obstacle_raycast_plugin");

  pos_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/uav_world_positions", 10,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      this->UpdatePositions(msg->data);
    });

  loss_pub_ = ros_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/link_obstacle_loss", 10);

  ros_thread_ = std::thread([this]() { rclcpp::spin(ros_node_); });
  ros_thread_.detach();

  update_conn_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&ObstacleRaycastPlugin::OnUpdate, this));

  gzmsg << "ObstacleRaycastPlugin successfully loaded for " << n_uavs_ << " UAVs!" << std::endl;
}

ObstacleRaycastPlugin::~ObstacleRaycastPlugin() { rclcpp::shutdown(); }

void ObstacleRaycastPlugin::UpdatePositions(const std::vector<float> & data)
{
  std::lock_guard<std::mutex> lock(pos_mutex_);
  uav_positions_.clear();
  for (size_t i = 0; i + 3 < data.size(); i += 4) {
    int id = static_cast<int>(data[i]);
    uav_positions_[id] = ignition::math::Vector3d(data[i+1], data[i+2], data[i+3]);
  }
}

void ObstacleRaycastPlugin::OnUpdate()
{
  auto now = world_->SimTime();
  if ((now - last_check_).Double() < 0.1) return;
  last_check_ = now;

  std::lock_guard<std::mutex> lock(pos_mutex_);

  std_msgs::msg::Float32MultiArray loss_msg;
  for (int i = 0; i < n_uavs_; i++) {
    for (int j = i + 1; j < n_uavs_; j++) {
      ignition::math::Vector3d pos_i, pos_j;
      if (!GetUavPosition(i, pos_i) || !GetUavPosition(j, pos_j)) continue;

      double extra_loss = CastRay(pos_i, pos_j, i, j);
      loss_msg.data.push_back(static_cast<float>(i));
      loss_msg.data.push_back(static_cast<float>(j));
      loss_msg.data.push_back(static_cast<float>(extra_loss));

      PublishRayMarker(i, j, pos_i, pos_j, extra_loss > 0.0);
    }
  }
  if (!loss_msg.data.empty()) loss_pub_->publish(loss_msg);
}

// Positions received on /uav_world_positions take priority; when the topic
// is silent (e.g. minimal bring-up without the bridge stack), fall back to
// reading the UAV model's pose straight from the Gazebo world.
// UAV id 0 -> model named "<uav_prefix>1..." (e.g. iris_1_demo).
bool ObstacleRaycastPlugin::GetUavPosition(int id, ignition::math::Vector3d & out)
{
  auto it = uav_positions_.find(id);
  if (it != uav_positions_.end()) {
    out = it->second;
    return true;
  }

  std::string prefix = uav_prefix_ + std::to_string(id + 1);
  for (const auto & model : world_->Models()) {
    if (model->GetName().rfind(prefix, 0) == 0) {
      out = model->WorldPose().Pos();
      return true;
    }
  }
  return false;
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
    if(hit_entity.empty()) return 0.0;

    double dist_from_start = traveled + hit_dist;

    // Strict Distance Bound : Ignore hits that are further away than the destination UAV
    if(dist_from_start > (link_len - 0.5)) return 0.0;

    // Ground plane and UAV models (own body or a fellow fleet member's
    // body/rotors) are never real obstacles -- step past and re-cast.
    // "noloss"-tagged props (thin street furniture: poles, signs, hydrants,
    // dumpster, postbox) are treated the same way: they contribute zero loss
    // and must not mask a real obstacle behind them.
    if(hit_entity.find("ground_plane") != std::string::npos ||
       hit_entity.find(uav_prefix_)   != std::string::npos ||
       hit_entity.find("noloss")      != std::string::npos) {
      traveled = dist_from_start + 0.1;
      continue;
    }

    // Printing what was actually hit to the terminal for easy debugging
    RCLCPP_INFO(ros_node_->get_logger(),
      "Ray %d->%d hit obstacle : %s at distance %.2f",
      id_a, id_b, hit_entity.c_str(), dist_from_start);

    return ComputeObstacleLoss(hit_entity, dist_from_start, start, end);
  }
  return 0.0;
}

// Draws each UAV-pair ray as a line in gzclient via Gazebo's /marker
// service: green = clear line-of-sight, red = blocked by an obstacle.
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

  marker.mutable_material()->mutable_script()->set_name(
    blocked ? "Gazebo/Red" : "Gazebo/Green");

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

    // step past filtered entities (ground / UAV bodies / noloss props), same
    // as the forward cast
    if (hit_entity.find("ground_plane") != std::string::npos ||
        hit_entity.find(uav_prefix_)   != std::string::npos ||
        hit_entity.find("noloss")      != std::string::npos) {
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
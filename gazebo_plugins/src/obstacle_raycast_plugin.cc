
// Gazebo Ray-Cast Plugin (Full, Scalable to N drones)
//////////////////////////////////////////////////////

// Relative include: CMakeLists adds gazebo_plugins/include to the include path,
// so this resolves on any machine/checkout (was a hardcoded /home/ubuntu/... path).
#include "gazebo_plugins/obstacle_raycast_plugin.hh"

#include <algorithm>
#include <cmath>
#include <complex>


namespace gazebo
{

// ADDED: the material table. Each row is ONLY the four ITU-R P.2040-3 fitting
// constants; L_e and alpha are never stored, they are computed from these by
// MaterialLoss(). Values for glass/wood/concrete/metal are Table 3 of
// ITU-R P.2040-3 (valid 1-100 GHz, so 5.18 GHz is well inside range).
//
// "foliage" and "vehicle" have no Table 3 row -- P.2040 describes homogeneous
// slabs, a canopy is distributed scatterers and a car is a hollow shell. Their
// constants are EFFECTIVE-MEDIUM fits, chosen so the same two equations
// reproduce the measured 5 GHz behaviour (~1.2 dB/m through in-leaf canopy,
// ~12 dB through a car body). They are fits, not standard values.
//
// Order matters: the FIRST keyword found in the entity name wins, so the
// composite props ("vehicle" bodies are metal, "foliage" has woody parts) are
// listed before the bulk materials they would otherwise match.
namespace
{
const MaterialSpec kMaterials[] = {
  //  keyword       a        b        c          d
  { "foliage",     1.0,    0.0,        0.0,    0.0    },   // Assuming similar to air
  { "vehicle",     1.0,    0.0,    2.10e-3,    0.0    },   // c from Conductivity_Al chart.pdf | a Assumed since similar to metal
  { "glass",      6.31,    0.0,    3.60e-3,    1.3394 },   // P.2040-3 Table 3
  { "wood",       1.99,    0.0,    4.70e-3,    1.0718 },   // P.2040-3 Table 3
  { "concrete",   5.24,    0.0,    4.62e-2,    0.7822 },   // P.2040-3 Table 3
  { "brick",      3.91,    0.0,    2.38e-2,    0.1600 },   // P.2040-3 Table 3
  { "metal",      1.00,    0.0,    1.00e+7,    0.0    },   // P.2040-3 Table 3
};
}  // namespace

void ObstacleRaycastPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
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

  // ADDED: carrier frequency the material equations are evaluated at. Keep this
  // in step with the NS-3 channel (802.11a -> 5.18 GHz); expose it so a world
  // running a different band gets the right L_e/alpha without a recompile.
  freq_ghz_ = sdf->HasElement("freq_ghz") ? sdf->Get<double>("freq_ghz") : 5.18;

  n_nodes_ = n_uavs_ + (gcs_enabled_ ? 1 : 0);

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

  gzmsg << "ObstacleRaycastPlugin loaded: " << n_uavs_ << " UAVs"
        << (gcs_enabled_
              ? " + GCS '" + gcs_model_ + "' (node 0, antenna +"
                + std::to_string(gcs_antenna_height_) + " m)"
              : " (no GCS)")
        << " => " << n_nodes_ << " nodes, "
        << (n_nodes_ * (n_nodes_ - 1) / 2) << " links" << std::endl;
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
  if (!loss_msg.data.empty()) loss_pub_->publish(loss_msg);
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
  auto it = uav_positions_.find(id);
  if (it != uav_positions_.end()) {
    out = it->second;
    return true;
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
    if(hit_entity.empty()) return 0.0;

    double dist_from_start = traveled + hit_dist;

    // Strict Distance Bound : Ignore hits that are further away than the destination UAV
    if(dist_from_start > (link_len - 0.5)) return 0.0;

    // Filtered entities (ground, UAV bodies, the GCS structure, "noloss"
    // props) are never real obstacles -- step past and re-cast so they cannot
    // mask a genuine obstacle behind them. See IsFilteredEntity().
    if (IsFilteredEntity(hit_entity)) {
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



// ADDED: derives BOTH loss terms for an obstacle from its material, replacing
// the old hardcoded per-keyword numbers. The entity name only selects a row of
// kMaterials; the two numbers themselves come out of equations, so changing the
// carrier frequency changes them automatically.
//
// Step 1 -- the row's constants give the electrical properties at freq_ghz_
// (ITU-R P.2040-3 eq. 57/58, and eq. 9 for the imaginary part):
//
//        eps'  = a * f^b
//        sigma = c * f^d                              [S/m]
//        eps'' = 17.98 * sigma / f                    (f in GHz)
//        eta   = eps' - j*eps''                       complex permittivity
//
// Step 2 -- EQUATION ONE, the entry/exit penetration loss. Fresnel reflection
// at normal incidence on the entry face, then again on the exit face. The
// power fraction getting through one interface is 1 - |Gamma|^2, two faces
// square that, hence the 20 (not 10) in front of the log:
//
//        Reflection = (1 - sqrt(eta)) / (1 + sqrt(eta))
//        L_e   = -20 * log10(1 - |Reflection|^2)           [dB]
//
// Step 3 -- EQUATION TWO, the bulk attenuation constant. The complex refractive
// index is n = sqrt(eta) = n_r - j*kappa; kappa (the attenuation index) is what
// the field decays with inside the material. In dB per metre (ITU-R P.2040-3
// eq. 12/13, field decay exp(-k0*kappa*z), k0 = 2*pi/lambda):
//
//        kappa = -Im( sqrt(eta) )
//        alpha = 8.686 * (2*pi/lambda) * kappa        [dB/m]
//
// Returns false when the entity name matches no row -- untagged obstacles are
// RF-transparent, as before.
bool ObstacleRaycastPlugin::MaterialLoss(const std::string & entity_name,
                                          double & L_e, double & alpha) const
{
  L_e   = 0.0;
  alpha = 0.0;

  const MaterialSpec * spec = nullptr;
  for (const auto & m : kMaterials) {
    if (entity_name.find(m.keyword) != std::string::npos) { spec = &m; break; }
  }
  if (!spec) return false;

  // --- Step 1: electrical properties at the operating frequency -------------
  const double f     = freq_ghz_;
  const double eps_r = spec->a * std::pow(f, spec->b);          // eps'
  const double sigma = spec->c * std::pow(f, spec->d);          // S/m
  const double eps_i = 17.98 * sigma / f;                       // eps''
  if (eps_r <= 0.0) return false;                               // malformed row

  // --- Step 2: entry/exit penetration loss ---------------------------------
  const std::complex<double> eta(eps_r, -eps_i);
  const std::complex<double> n     = std::sqrt(eta);
  const std::complex<double> Reflection = (1.0 - n) / (1.0 + n);
  const double transmitted = 1.0 - std::norm(Reflection);   // norm() is |Reflection|^2
  // A near-perfect reflector (the metal row) drives the transmitted fraction
  // to zero and the log to -inf; clamp instead, the link is dead either way.
  L_e = (transmitted > 1e-12) ? -20.0 * std::log10(transmitted) : max_loss_db_;
  L_e = std::min(L_e, max_loss_db_);

  // --- Step 3: bulk attenuation constant -----------------------------------
  //const double lambda = 0.299792458 / f;   // wavelength in m, with f in GHz
  //const double kappa  = -n.imag();         // attenuation index; Im(sqrt(eta)) < 0
  const std::complex<double> j(0.0, 1.0);
  const std::complex<double> gamma = j * 20.96 * f *std::sqrt(eta);  // ITU-R P.2040-3 eq. 12
  alpha = 8.686 * gamma.real();  // dB/m
  // A lossless dielectric (sigma = 0) has kappa = 0 -> alpha = 0, so the
  // obstacle costs only its entry/exit loss. Clamp the lossy extreme.
  alpha = std::min(std::max(alpha, 0.0), max_loss_db_);

  return true;
}

double ObstacleRaycastPlugin::ComputeObstacleLoss(const std::string & entity_name,
                                                    double hit_dist,
                                                    const ignition::math::Vector3d & start,
                                                    const ignition::math::Vector3d & end)
{
  // CHANGED: an obstacle whose <name> carries NO recognised material keyword
  // now contributes ZERO loss instead of a default 15 dB. Only explicitly
  // tagged materials attenuate the link; anything untagged is treated as
  // RF-transparent (same net effect as a "noloss" prop).
  // REMOVED: double L_e = 15.0;  // default: unknown solid ~ masonry / concrete
  // CHANGED: L_e and alpha are no longer a table of hardcoded dB values --
  // MaterialLoss() computes both from the material's electrical properties.
  double L_e, alpha;
  if (!MaterialLoss(entity_name, L_e, alpha)) return 0.0;

  // CHANGED: the bulk term is alpha * t, with alpha now material-specific
  // instead of the flat 0.5 dB/m every obstacle used to get.
  double thickness = ObstacleThickness(start, end, entity_name, hit_dist);
  thickness = std::min(thickness, 20.0);
  return std::min(L_e + alpha * thickness, max_loss_db_);
}

GZ_REGISTER_WORLD_PLUGIN(ObstacleRaycastPlugin)
}  // namespace gazebo
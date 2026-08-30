#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace multi_uav_gazebo_plugins
{

class ObstacleRaycastPlugin : public gazebo::WorldPlugin
{
public:
  ObstacleRaycastPlugin() = default;
  ~ObstacleRaycastPlugin() override = default;

  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = std::move(world);

    if (!world_)
    {
      gzerr << "[obstacle_raycast] World pointer is null.\n";
      return;
    }

    ros_node_ = gazebo_ros::Node::Get(sdf);
    if (!ros_node_)
    {
      gzerr << "[obstacle_raycast] Failed to create gazebo_ros node.\n";
      return;
    }

    configured_n_uavs_ = GetSdf<uint32_t>(sdf, "n_uavs", 0u);
    uav_prefix_ = GetSdf<std::string>(sdf, "uav_prefix", "iris_");
    gcs_enabled_ = GetSdf<bool>(sdf, "gcs_enabled", true);
    gcs_model_name_ = GetSdf<std::string>(sdf, "gcs_model", "gcs");
    gcs_antenna_height_m_ = GetSdf<double>(sdf, "gcs_antenna_height", 2.9);
    uav_antenna_height_m_ = GetSdf<double>(sdf, "uav_antenna_height", 0.0);

    update_rate_hz_ = GetSdf<double>(sdf, "update_rate_hz", 10.0);
    model_refresh_rate_hz_ = GetSdf<double>(sdf, "model_refresh_rate_hz", 1.0);

    default_obstacle_loss_db_ =
      GetSdf<double>(sdf, "default_obstacle_loss_db", 8.0);
    concrete_loss_db_ = GetSdf<double>(sdf, "concrete_loss_db", 12.0);
    foliage_loss_db_ = GetSdf<double>(sdf, "foliage_loss_db", 4.0);
    vehicle_loss_db_ = GetSdf<double>(sdf, "vehicle_loss_db", 6.0);
    human_loss_db_ = GetSdf<double>(sdf, "human_loss_db", 0.0);

    ray_advance_epsilon_m_ =
      GetSdf<double>(sdf, "ray_advance_epsilon_m", 0.05);
    endpoint_guard_m_ = GetSdf<double>(sdf, "endpoint_guard_m", 0.10);
    max_ray_hits_ = GetSdf<uint32_t>(sdf, "max_ray_hits", 64u);

    if (update_rate_hz_ <= 0.0)
    {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "update_rate_hz <= 0; using 10 Hz");
      update_rate_hz_ = 10.0;
    }

    if (model_refresh_rate_hz_ <= 0.0)
    {
      model_refresh_rate_hz_ = 1.0;
    }

    if (ray_advance_epsilon_m_ <= 0.0)
    {
      ray_advance_epsilon_m_ = 0.05;
    }

    if (endpoint_guard_m_ < 0.0)
    {
      endpoint_guard_m_ = 0.0;
    }

    if (max_ray_hits_ == 0)
    {
      max_ray_hits_ = 64;
    }

    LoadIgnoreTokens(sdf);

    auto physics = world_->Physics();
    if (!physics)
    {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Gazebo physics engine is unavailable");
      return;
    }

    ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      physics->CreateShape("ray", gazebo::physics::CollisionPtr()));

    if (!ray_)
    {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Failed to create Gazebo RayShape");
      return;
    }

    publisher_ = ros_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/link_obstacle_loss",
      rclcpp::SensorDataQoS());

    RefreshNodes(true);

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ObstacleRaycastPlugin::OnUpdate, this, std::placeholders::_1));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Obstacle raycast plugin loaded: n_uavs=%u (%s), prefix='%s', "
      "GCS=%s, publish=%.2f Hz",
      configured_n_uavs_,
      configured_n_uavs_ == 0 ? "auto-discovery" : "explicit",
      uav_prefix_.c_str(),
      gcs_enabled_ ? "enabled" : "disabled",
      update_rate_hz_);
  }

private:
  struct NodeEndpoint
  {
    uint32_t id{0};
    std::string model_name;
    gazebo::physics::ModelPtr model;
    double antenna_height_m{0.0};
  };

  template<typename T>
  static T GetSdf(
    const sdf::ElementPtr & sdf,
    const std::string & name,
    const T & default_value)
  {
    if (sdf && sdf->HasElement(name))
    {
      return sdf->Get<T>(name);
    }
    return default_value;
  }

  static std::string ToLower(std::string value)
  {
    std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
  }

  static bool IsUnsignedInteger(const std::string & value)
  {
    if (value.empty())
    {
      return false;
    }

    return std::all_of(
      value.begin(), value.end(),
      [](unsigned char c) { return std::isdigit(c) != 0; });
  }

  static bool ContainsToken(
    const std::string & lower_text,
    const std::string & lower_token)
  {
    return !lower_token.empty() &&
      lower_text.find(lower_token) != std::string::npos;
  }

  void LoadIgnoreTokens(const sdf::ElementPtr & sdf)
  {
    // These names are deliberately ignored by default because the supplied
    // city world already marks non-radio-blocking geometry with "noloss" and
    // the ground plane should never count as a radio obstacle.
    ignore_tokens_.clear();
    ignore_tokens_.push_back("noloss");
    ignore_tokens_.push_back("ground_plane");

    if (!sdf || !sdf->HasElement("ignore_tokens"))
    {
      return;
    }

    const std::string raw = sdf->Get<std::string>("ignore_tokens");
    std::stringstream ss(raw);
    std::string token;

    while (std::getline(ss, token, ','))
    {
      token.erase(
        std::remove_if(
          token.begin(), token.end(),
          [](unsigned char c) { return std::isspace(c) != 0; }),
        token.end());

      token = ToLower(token);
      if (!token.empty())
      {
        ignore_tokens_.push_back(token);
      }
    }
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();

    const double refresh_period = 1.0 / model_refresh_rate_hz_;
    if (
      last_model_refresh_s_ < 0.0 ||
      now - last_model_refresh_s_ >= refresh_period)
    {
      RefreshNodes(false);
      last_model_refresh_s_ = now;
    }

    const double publish_period = 1.0 / update_rate_hz_;
    if (
      last_publish_s_ >= 0.0 &&
      now - last_publish_s_ < publish_period)
    {
      return;
    }

    last_publish_s_ = now;

    if (nodes_.size() < 2)
    {
      return;
    }

    PublishAllLinks();
  }

  void RefreshNodes(bool force_log)
  {
    std::vector<NodeEndpoint> new_nodes;

    if (gcs_enabled_)
    {
      auto gcs_model = world_->ModelByName(gcs_model_name_);
      if (gcs_model)
      {
        new_nodes.push_back(NodeEndpoint{
          0u,
          gcs_model_name_,
          gcs_model,
          gcs_antenna_height_m_});
      }
      else if (force_log)
      {
        RCLCPP_ERROR(
          ros_node_->get_logger(),
          "GCS model '%s' was not found in Gazebo",
          gcs_model_name_.c_str());
      }
    }

    if (configured_n_uavs_ > 0)
    {
      for (uint32_t id = 1; id <= configured_n_uavs_; ++id)
      {
        const std::string model_name = uav_prefix_ + std::to_string(id);
        auto model = world_->ModelByName(model_name);

        if (!model)
        {
          if (force_log)
          {
            RCLCPP_ERROR(
              ros_node_->get_logger(),
              "Expected UAV model '%s' (id=%u), but Gazebo does not contain it",
              model_name.c_str(), id);
          }
          continue;
        }

        new_nodes.push_back(NodeEndpoint{
          id,
          model_name,
          model,
          uav_antenna_height_m_});
      }
    }
    else
    {
      // Auto-discovery mode: find top-level models named prefix + integer,
      // for example iris_1, iris_2, iris_3, ... .
      for (const auto & model : world_->Models())
      {
        if (!model)
        {
          continue;
        }

        const std::string name = model->GetName();
        if (name.rfind(uav_prefix_, 0) != 0)
        {
          continue;
        }

        const std::string suffix = name.substr(uav_prefix_.size());
        if (!IsUnsignedInteger(suffix))
        {
          continue;
        }

        const unsigned long parsed = std::stoul(suffix);
        if (parsed == 0 || parsed > std::numeric_limits<uint32_t>::max())
        {
          continue;
        }

        new_nodes.push_back(NodeEndpoint{
          static_cast<uint32_t>(parsed),
          name,
          model,
          uav_antenna_height_m_});
      }
    }

    std::sort(
      new_nodes.begin(), new_nodes.end(),
      [](const NodeEndpoint & a, const NodeEndpoint & b) {
        return a.id < b.id;
      });

    // Reject duplicate IDs. This should not happen with the expected model
    // naming convention, but catching it avoids publishing ambiguous link IDs.
    std::vector<NodeEndpoint> unique_nodes;
    std::set<uint32_t> seen_ids;

    for (const auto & node : new_nodes)
    {
      if (seen_ids.insert(node.id).second)
      {
        unique_nodes.push_back(node);
      }
      else if (force_log)
      {
        RCLCPP_ERROR(
          ros_node_->get_logger(),
          "Duplicate radio-node id %u discovered in Gazebo",
          node.id);
      }
    }

    const bool changed = NodeSetChanged(unique_nodes);
    nodes_ = std::move(unique_nodes);

    if (changed || force_log)
    {
      LogNodeMap();
    }
  }

  bool NodeSetChanged(const std::vector<NodeEndpoint> & new_nodes) const
  {
    if (new_nodes.size() != nodes_.size())
    {
      return true;
    }

    for (size_t i = 0; i < new_nodes.size(); ++i)
    {
      if (
        new_nodes[i].id != nodes_[i].id ||
        new_nodes[i].model_name != nodes_[i].model_name)
      {
        return true;
      }
    }

    return false;
  }

  void LogNodeMap() const
  {
    std::ostringstream out;
    out << "radio nodes=" << nodes_.size() << " [";

    for (size_t i = 0; i < nodes_.size(); ++i)
    {
      if (i > 0)
      {
        out << ", ";
      }
      out << nodes_[i].id << "=" << nodes_[i].model_name;
    }

    out << "]";

    const size_t n = nodes_.size();
    const size_t pairs = n >= 2 ? (n * (n - 1)) / 2 : 0;

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "%s, link_pairs=%zu",
      out.str().c_str(), pairs);
  }

  ignition::math::Vector3d EndpointPosition(const NodeEndpoint & node) const
  {
    ignition::math::Vector3d p = node.model->WorldPose().Pos();
    p.Z() += node.antenna_height_m;
    return p;
  }

  void PublishAllLinks()
  {
    std_msgs::msg::Float32MultiArray msg;

    const size_t n = nodes_.size();
    const size_t pair_count = (n * (n - 1)) / 2;
    msg.data.reserve(pair_count * 3);

    for (size_t a = 0; a < nodes_.size(); ++a)
    {
      for (size_t b = a + 1; b < nodes_.size(); ++b)
      {
        const double loss_db = CalculateObstacleLoss(nodes_[a], nodes_[b]);

        msg.data.push_back(static_cast<float>(nodes_[a].id));
        msg.data.push_back(static_cast<float>(nodes_[b].id));
        msg.data.push_back(static_cast<float>(loss_db));
      }
    }

    publisher_->publish(msg);
  }

  double CalculateObstacleLoss(
    const NodeEndpoint & source,
    const NodeEndpoint & target)
  {
    const ignition::math::Vector3d start = EndpointPosition(source);
    const ignition::math::Vector3d end = EndpointPosition(target);

    const ignition::math::Vector3d delta = end - start;
    const double total_distance = delta.Length();

    if (total_distance <= 1e-6)
    {
      return 0.0;
    }

    const ignition::math::Vector3d direction = delta / total_distance;

    ignition::math::Vector3d cursor = start;
    double total_loss_db = 0.0;

    // Count each Gazebo model only once even if the ray hits both entry and
    // exit surfaces or multiple collision objects belonging to that model.
    std::set<std::string> counted_obstacle_models;

    for (uint32_t hit_index = 0; hit_index < max_ray_hits_; ++hit_index)
    {
      const double remaining = (end - cursor).Length();
      if (remaining <= endpoint_guard_m_ + ray_advance_epsilon_m_)
      {
        break;
      }

      ray_->SetPoints(cursor, end);

      double hit_distance = 0.0;
      std::string collision_name;
      ray_->GetIntersection(hit_distance, collision_name);

      if (collision_name.empty() || !std::isfinite(hit_distance))
      {
        break;
      }

      if (hit_distance < 0.0)
      {
        break;
      }

      // A hit extremely close to the target is normally the target vehicle's
      // own collision geometry and should not be considered an obstacle.
      if (hit_distance >= remaining - endpoint_guard_m_)
      {
        break;
      }

      const std::string lower_collision = ToLower(collision_name);

      if (!ShouldIgnoreCollision(lower_collision, source, target))
      {
        const std::string obstacle_key = CollisionModelKey(collision_name);

        if (counted_obstacle_models.insert(obstacle_key).second)
        {
          total_loss_db += LossForCollision(lower_collision);
        }
      }

      // Move slightly beyond the hit and cast again. This allows the plugin to
      // discover another obstacle farther along the same link instead of only
      // considering the first collision.
      const double advance = std::max(
        hit_distance + ray_advance_epsilon_m_,
        ray_advance_epsilon_m_);

      if (advance >= remaining)
      {
        break;
      }

      cursor += direction * advance;
    }

    return std::max(0.0, total_loss_db);
  }

  bool ShouldIgnoreCollision(
    const std::string & lower_collision,
    const NodeEndpoint & source,
    const NodeEndpoint & target) const
  {
    const std::string source_name = ToLower(source.model_name);
    const std::string target_name = ToLower(target.model_name);

    // Ignore the radios' own collision geometry.
    if (
      lower_collision.find(source_name) != std::string::npos ||
      lower_collision.find(target_name) != std::string::npos)
    {
      return true;
    }

    for (const auto & token : ignore_tokens_)
    {
      if (ContainsToken(lower_collision, token))
      {
        return true;
      }
    }

    return false;
  }

  static std::string CollisionModelKey(const std::string & collision_name)
  {
    // Gazebo's ray usually returns a scoped collision such as:
    //   model::link::collision
    // Using the first component makes all collisions belonging to the same
    // top-level model share one loss contribution.
    const std::size_t scope = collision_name.find("::");
    if (scope == std::string::npos)
    {
      return collision_name;
    }
    return collision_name.substr(0, scope);
  }

  double LossForCollision(const std::string & lower_collision) const
  {
    // IMPORTANT: these are configurable modelling parameters, not universal
    // material constants. Tune them using your validation experiments.

    if (
      ContainsToken(lower_collision, "foliage") ||
      ContainsToken(lower_collision, "tree"))
    {
      return foliage_loss_db_;
    }

    if (
      ContainsToken(lower_collision, "concrete") ||
      ContainsToken(lower_collision, "building") ||
      ContainsToken(lower_collision, "apartment") ||
      ContainsToken(lower_collision, "house") ||
      ContainsToken(lower_collision, "office") ||
      ContainsToken(lower_collision, "wall"))
    {
      return concrete_loss_db_;
    }

    if (
      ContainsToken(lower_collision, "vehicle") ||
      ContainsToken(lower_collision, "car") ||
      ContainsToken(lower_collision, "hatchback") ||
      ContainsToken(lower_collision, "suv") ||
      ContainsToken(lower_collision, "truck"))
    {
      return vehicle_loss_db_;
    }

    if (ContainsToken(lower_collision, "human"))
    {
      return human_loss_db_;
    }

    return default_obstacle_loss_db_;
  }

private:
  gazebo::physics::WorldPtr world_;
  gazebo::physics::RayShapePtr ray_;
  gazebo::event::ConnectionPtr update_connection_;

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

  uint32_t configured_n_uavs_{0};
  std::string uav_prefix_{"iris_"};

  bool gcs_enabled_{true};
  std::string gcs_model_name_{"gcs"};
  double gcs_antenna_height_m_{2.9};
  double uav_antenna_height_m_{0.0};

  double update_rate_hz_{10.0};
  double model_refresh_rate_hz_{1.0};

  double default_obstacle_loss_db_{8.0};
  double concrete_loss_db_{12.0};
  double foliage_loss_db_{4.0};
  double vehicle_loss_db_{6.0};
  double human_loss_db_{0.0};

  double ray_advance_epsilon_m_{0.05};
  double endpoint_guard_m_{0.10};
  uint32_t max_ray_hits_{64};

  std::vector<std::string> ignore_tokens_;
  std::vector<NodeEndpoint> nodes_;

  double last_publish_s_{-1.0};
  double last_model_refresh_s_{-1.0};
};

GZ_REGISTER_WORLD_PLUGIN(ObstacleRaycastPlugin)

}  // namespace multi_uav_gazebo_plugins

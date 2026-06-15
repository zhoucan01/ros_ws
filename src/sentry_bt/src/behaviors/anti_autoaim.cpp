#include "sentry_bt/anti_autoaim.hpp"

// SetTargetPoint(const std::string& name, const BT::NodeConfig& config)
//         : BT::SyncActionNode(name, config) {}

// namespace pb2025_sentry_behavior
// {
AntiAutoAim::AntiAutoAim(const std::string &name, const BT::NodeConfig &config,
                         std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), node_(node)
{
  params_.attack_radius = node_->declare_parameter<double>("anti_autoaim.attack_radius", params_.attack_radius);
  params_.num_sectors = node_->declare_parameter<int>("anti_autoaim.num_sectors", params_.num_sectors);
  params_.cost_threshold = node_->declare_parameter<int>("anti_autoaim.cost_threshold", params_.cost_threshold);
  params_.robot_base_frame = node_->declare_parameter<std::string>("anti_autoaim.robot_base_frame", params_.robot_base_frame);
  params_.transform_tolerance = node_->declare_parameter<double>("anti_autoaim.transform_tolerance", params_.transform_tolerance);
  params_.max_visualization_distance = node_->declare_parameter<double>(
      "anti_autoaim.max_visualization_distance", params_.max_visualization_distance);
  params_.marker_scale_base = node_->declare_parameter<double>("anti_autoaim.marker_scale_base", params_.marker_scale_base);
  params_.visualize = node_->declare_parameter<bool>("anti_autoaim.visualize", params_.visualize);
  params_.enable_attack = node_->declare_parameter<bool>("anti_autoaim.enable_attack", params_.enable_attack);
  params_.limit_chase_range = node_->declare_parameter<bool>("anti_autoaim.limit_chase_range", params_.limit_chase_range);
  params_.max_chase_distance = node_->declare_parameter<double>("anti_autoaim.max_chase_distance", params_.max_chase_distance);
  params_.distance_weight = node_->declare_parameter<double>(
      "anti_autoaim.distance_weight", params_.distance_weight);
  params_.cost_weight = node_->declare_parameter<double>(
      "anti_autoaim.cost_weight", params_.cost_weight);

  const int decision_point_count = node_->declare_parameter<int>("decision_point_count", 8);

  for (int point_id = 0; point_id < decision_point_count; ++point_id)
  {
    const auto name = std::to_string(point_id);
    Policy policy;
    policy.enable = node_->declare_parameter<bool>("anti_autoaim.policy." + name + ".enable", params_.enable_attack);
    policy.limit = node_->declare_parameter<bool>("anti_autoaim.policy." + name + ".limit", params_.limit_chase_range);
    policy_map_[name] = policy;
  }

  // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
   rviz_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/attack_pose_viz", 10);
}

BT::PortsList AntiAutoAim::providedPorts()
{

  return {
      BT::InputPort<int>("target_point_port", "target_point port"),
      BT::InputPort<nav_msgs::msg::OccupancyGrid>("costmap_port", "Global costmap"),
      BT::InputPort<geometry_msgs::msg::PointStamped>("currentpos_port", "currentpos port"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("attack_point_port", "Output target attack point"),
      BT::OutputPort<bool>("attack_target_valid", "Whether attack target is valid"),
      BT::InputPort<geometry_msgs::msg::PointStamped>("enemy_pos_point_port", "currentpos port"),};

}

BT::NodeStatus AntiAutoAim::tick()
{
  Policy policy;
  bool has_policy = false;
  auto target_point_input = getInput<int>("target_point_port");
  if (target_point_input)
  {
    const auto point_key = std::to_string(target_point_input.value());
    auto it = policy_map_.find(point_key);
    if (it != policy_map_.end())
    {
      policy = it->second;
      has_policy = true;
    }
  }

  if (has_policy)
  {
    if (!policy.enable)
    {
      setOutput("attack_target_valid", false);
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    if (!params_.enable_attack)
    {
      setOutput("attack_target_valid", false);
      return BT::NodeStatus::FAILURE;
    }
  }

// RCLCPP_INFO(node_->get_logger(), "attack_point_port exists: %s", getInput<geometry_msgs::msg::PoseStamped>("attack_point_port") ? "yes" : "no");
// RCLCPP_INFO(node_->get_logger(), "decision_point_port exists: %s", getInput<int>("decision_point_port") ? "yes" : "no");
  // 订阅追击需要的数据
  auto global_costmap = getInput<nav_msgs::msg::OccupancyGrid>("costmap_port");
  auto current_pos = getInput<geometry_msgs::msg::PointStamped>("currentpos_port");
  auto enemy_pos_point_ = getInput<geometry_msgs::msg::PointStamped>("enemy_pos_point_port");
  if (!global_costmap)
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input: costmap_port");
    setOutput("attack_target_valid", false);
    return BT::NodeStatus::FAILURE; // 修正返回类型
  }
  if (!current_pos)
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input: currentpos_port");
    setOutput("attack_target_valid", false);
    return BT::NodeStatus::FAILURE;
  }
  if (!enemy_pos_point_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input: enemy_pos_point_port");
    setOutput("attack_target_valid", false);
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PointStamped enemy_pose_point;
  enemy_pose_point = enemy_pos_point_.value();
  RCLCPP_INFO(node_->get_logger(), "enemy_pos_point_value: x=%.2f, y=%.2f", enemy_pose_point.point.x, enemy_pose_point.point.y);
  Point enemy_point;
  enemy_point.x = enemy_pose_point.point.x;
  enemy_point.y = enemy_pose_point.point.y;

  std::vector<Point> candidates;
  std::vector<Point> feasible_points;

  candidates = generateCandidatePoints(enemy_point);
  feasible_points = filterFeasiblePoints(candidates, global_costmap.value());
  if (feasible_points.empty())
  {
      RCLCPP_WARN(node_->get_logger(), "No feasible points available, cannot determine attack point");
      setOutput("attack_target_valid", false);
      return BT::NodeStatus::FAILURE;
  }

  const bool limit_enabled = has_policy ? policy.limit : params_.limit_chase_range;
  std::array<double, 4> limit_rect = {0.0, 0.0, 0.0, 0.0};
  bool has_limit_rect = false;
  if (limit_enabled)
  {
    if (!target_point_input)
    {
      RCLCPP_WARN(node_->get_logger(), "limit_chase_range enabled but target_point_port missing");
      setOutput("attack_target_valid", false);
      return BT::NodeStatus::FAILURE;
    }

    if (!getDecisionPointRect(target_point_input.value(), limit_rect))
    {
      RCLCPP_WARN(node_->get_logger(), "limit_chase_range enabled but decision rectangle not found");
      setOutput("attack_target_valid", false);
      return BT::NodeStatus::FAILURE;
    }
    has_limit_rect = true;

    std::vector<Point> limited;
    for (const auto &p : feasible_points)
    {
      if (p.x >= limit_rect[0] && p.x <= limit_rect[1] &&
          p.y >= limit_rect[2] && p.y <= limit_rect[3])
      {
        limited.push_back(p);
      }
    }

    if (limited.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No feasible points inside chase rectangle");
      setOutput("attack_target_valid", false);
      return BT::NodeStatus::FAILURE;
    }
    feasible_points.swap(limited);
  }
  

  const auto best_point = selectBestPoint(
      feasible_points, current_pos.value().point, global_costmap.value());



  // const auto best_point = selectBestPoint(feasible_points, current_pos);//遍历可经过候选点中最近的点

  // Create attack pose
  const auto attack_pose = createAttackPose(best_point, enemy_point);
  // RCLCPP_INFO(node_->get_logger(), "attack_pose: x=%.2f, y=%.2f", attack_pose.pose.position.x, attack_pose.pose.position.y);

  setOutput("attack_point_port", attack_pose);
  setOutput("attack_target_valid", true);

  if (params_.visualize)
  {
    createVisualizationMarkers(
        msg, enemy_point, candidates, feasible_points, current_pos.value().point,
        global_costmap.value(), has_limit_rect, limit_rect);
    rviz_pub_->publish(msg);
  }

  return BT::NodeStatus::SUCCESS;
}

bool AntiAutoAim::getDecisionPointRect(int point_id, std::array<double, 4> &rect) const
{
  const auto prefix = "anti_autoaim.rect." + std::to_string(point_id);
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;

  if (!node_->get_parameter(prefix + ".x_min", x_min) ||
      !node_->get_parameter(prefix + ".x_max", x_max) ||
      !node_->get_parameter(prefix + ".y_min", y_min) ||
      !node_->get_parameter(prefix + ".y_max", y_max))
  {
    return false;
  }

  rect = {std::min(x_min, x_max), std::max(x_min, x_max),
          std::min(y_min, y_max), std::max(y_min, y_max)};
  return true;
}

std::vector<geometry_msgs::msg::Point> AntiAutoAim::generateCandidatePoints(const Point enemy_point)
{
  std::vector<geometry_msgs::msg::Point> candidates;
  candidates.reserve(params_.num_sectors);

  RCLCPP_INFO(node_->get_logger(),"best_point: x=%.2f, y=%.2f", enemy_point.x, enemy_point.y);

  for (int i = 0; i < params_.num_sectors; ++i)
  {
    const double angle = i * 2 * M_PI / params_.num_sectors;
    geometry_msgs::msg::Point p;
    p.x = enemy_point.x + params_.attack_radius * cos(angle);
    p.y = enemy_point.y + params_.attack_radius * sin(angle);
    p.z = 0.0;
    candidates.push_back(p);
  }
  return candidates;
}

std::vector<Point> AntiAutoAim::filterFeasiblePoints(
    const std::vector<Point> &candidates, const nav_msgs::msg::OccupancyGrid &costmap)
{
  std::vector<Point> feasible_points;
  const auto &info = costmap.info;

  for (const auto &p : candidates)
  {
    const int cell_x = static_cast<int>((p.x - info.origin.position.x) / info.resolution);
    const int cell_y = static_cast<int>((p.y - info.origin.position.y) / info.resolution);

    if (
        cell_x < 0 || cell_x >= static_cast<int>(info.width) || cell_y < 0 ||
        cell_y >= static_cast<int>(info.height))
    {
      continue;
    }

    const int index = cell_y * info.width + cell_x;
    const int8_t cost = costmap.data[index];
    // RCLCPP_INFO(node_->get_logger(), "cost_map_cost: %d", cost);
    if (cost >= 0 && cost <= params_.cost_threshold)
    {
      feasible_points.push_back(p);
    }
  }
  return feasible_points;
}

Point AntiAutoAim::selectBestPoint(
    const std::vector<Point> &feasible_points, const Point &robot_position,
    const nav_msgs::msg::OccupancyGrid &costmap)
{
  const auto &info = costmap.info;

  double max_distance = 1e-6;
  for (const auto &point : feasible_points)
  {
    const double dx = point.x - robot_position.x;
    const double dy = point.y - robot_position.y;
    max_distance = std::max(max_distance, std::hypot(dx, dy));
  }

  auto score_point = [&](const Point &point)
  {
    const double dx = point.x - robot_position.x;
    const double dy = point.y - robot_position.y;
    const double distance = std::hypot(dx, dy);
    const double distance_norm = distance / max_distance;

    const int cell_x = static_cast<int>((point.x - info.origin.position.x) / info.resolution);
    const int cell_y = static_cast<int>((point.y - info.origin.position.y) / info.resolution);

    double cost_norm = 1.0;
    if (
      cell_x >= 0 && cell_x < static_cast<int>(info.width) &&
      cell_y >= 0 && cell_y < static_cast<int>(info.height))
    {
      const int index = cell_y * info.width + cell_x;
      const double raw_cost = static_cast<double>(std::max<int8_t>(0, costmap.data[index]));
      cost_norm = raw_cost / 252.0;
    }

    return params_.distance_weight * distance_norm +
           params_.cost_weight * cost_norm;
  };

  auto compare = [&](const Point &a, const Point &b)
  {
    return score_point(a) < score_point(b);
  };

  return *std::min_element(feasible_points.begin(), feasible_points.end(), compare);
}

PoseStamped AntiAutoAim::createAttackPose(
    const Point &attack_point, const Point &enemy_position)
{
  PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();
  pose.pose.position = attack_point;
  pose.pose.orientation.w = 1.0; // 默认朝向（无旋转）
  const double dx = enemy_position.x - attack_point.x;
  const double dy = enemy_position.y - attack_point.y;
  //   tf2::Quaternion q;
  //   q.setRPY(0, 0, atan2(dy, dx));
  //   pose.pose.orientation = tf2::toMsg(q);

  //   pose.pose.orientation = 0.0;
  return pose;
}

void AntiAutoAim::createVisualizationMarkers(
    visualization_msgs::msg::MarkerArray &msg, const Point &enemy_position,
    const std::vector<Point> &candidates, const std::vector<Point> &feasible_points,
    const Point &robot_position, const nav_msgs::msg::OccupancyGrid &costmap,
    bool limit_enabled, const std::array<double, 4> &limit_rect)
{
  msg.markers.clear();

  // Enemy marker
  visualization_msgs::msg::Marker enemy_marker;
  enemy_marker.header.frame_id = costmap.header.frame_id;
  enemy_marker.ns = "enemy";
  enemy_marker.id = 0;
  enemy_marker.type = visualization_msgs::msg::Marker::SPHERE;
  enemy_marker.pose.position = enemy_position;
  enemy_marker.scale.x = enemy_marker.scale.y = enemy_marker.scale.z = 0.3;
  enemy_marker.color.b = 1.0;
  enemy_marker.color.a = 1.0;
  msg.markers.push_back(enemy_marker);
  // RCLCPP_INFO(node_->get_logger(),"enemy_position_first: x=%.2f, y=%.2f", enemy_position.x, enemy_position.y);

  // Best attack marker
  const auto best_point = selectBestPoint(feasible_points, robot_position, costmap);
  visualization_msgs::msg::Marker best_marker;
  best_marker.header.frame_id = costmap.header.frame_id;
  best_marker.ns = "best";
  best_marker.id = 0;
  best_marker.type = visualization_msgs::msg::Marker::SPHERE;
  best_marker.pose.position = best_point;
  best_marker.scale.x = best_marker.scale.y = best_marker.scale.z = 0.4;
  best_marker.color.g = 1.0;
  best_marker.color.a = 1.0;
  msg.markers.push_back(best_marker);

  // Candidate markers
  const auto &info = costmap.info;
  for (size_t i = 0; i < candidates.size(); ++i)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = costmap.header.frame_id;
    marker.ns = "candidates";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.pose.position = candidates[i];

    // Calculate cost
    const int cell_x = (candidates[i].x - info.origin.position.x) / info.resolution;
    const int cell_y = (candidates[i].y - info.origin.position.y) / info.resolution;
    int8_t cost = -1;
    if (
        cell_x >= 0 && cell_x < static_cast<int>(info.width) && cell_y >= 0 &&
        cell_y < static_cast<int>(info.height))
    {
      cost = costmap.data[cell_y * info.width + cell_x];
    }

    // Visual properties
    marker.scale.x = marker.scale.y = marker.scale.z =
        params_.marker_scale_base + (cost / 100.0) * 0.3;

    const double distance =
        std::hypot(candidates[i].x - robot_position.x, candidates[i].y - robot_position.y);
    const float alpha =
        0.5 + 0.5 * (1.0 - std::min(distance / params_.max_visualization_distance, 1.0));

    marker.color.r = 1.0;
    marker.color.a = alpha;

    // Check feasibility
    const bool is_feasible = std::any_of(
        feasible_points.begin(), feasible_points.end(),
        [&](const auto &p)
        { return p.x == candidates[i].x && p.y == candidates[i].y; });

    if (!is_feasible)
    {
      marker.color.a *= 0.3;
    }

    msg.markers.push_back(marker);
  }

  // Range circle
  visualization_msgs::msg::Marker circle;
  circle.header.frame_id = costmap.header.frame_id;
  circle.ns = "range";
  circle.id = 0;
  circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle.pose.position.z = 0.05;
  circle.scale.x = 0.05;
  circle.color.b = 1.0;
  circle.color.a = 0.5;

  constexpr int circle_points = 36;
  // RCLCPP_INFO(node_->get_logger(),"enemy_position_second: x=%.2f, y=%.2f", enemy_position.x, enemy_position.y);
  for (int i = 0; i <= circle_points; ++i)
  {
    const double angle = i * 2 * M_PI / circle_points;
    Point p;
    p.x = enemy_position.x + params_.attack_radius * cos(angle);
    p.y = enemy_position.y + params_.attack_radius * sin(angle);
    circle.points.push_back(p);
  }
  msg.markers.push_back(circle);

  if (limit_enabled)
  {
    visualization_msgs::msg::Marker limit_rect_marker;
    limit_rect_marker.header.frame_id = costmap.header.frame_id;
    limit_rect_marker.ns = "chase_limit_rect";
    limit_rect_marker.id = 0;
    limit_rect_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    limit_rect_marker.pose.position.z = 0.08;
    limit_rect_marker.scale.x = 0.05;
    limit_rect_marker.color.r = 0.1f;
    limit_rect_marker.color.g = 0.9f;
    limit_rect_marker.color.b = 0.9f;
    limit_rect_marker.color.a = 0.95f;

    Point p;
    p.z = 0.0;
    p.x = limit_rect[0]; p.y = limit_rect[2]; limit_rect_marker.points.push_back(p);
    p.x = limit_rect[1]; p.y = limit_rect[2]; limit_rect_marker.points.push_back(p);
    p.x = limit_rect[1]; p.y = limit_rect[3]; limit_rect_marker.points.push_back(p);
    p.x = limit_rect[0]; p.y = limit_rect[3]; limit_rect_marker.points.push_back(p);
    p.x = limit_rect[0]; p.y = limit_rect[2]; limit_rect_marker.points.push_back(p);
    msg.markers.push_back(limit_rect_marker);
  }
}

// }

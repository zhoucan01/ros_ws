#include "pb_nav2_plugins/jps_minco_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb_nav2_planners
{
void JpsMincoPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock lifecycle node while configuring JpsMincoPlanner");
  }

  logger_ = node->get_logger();
  node->declare_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->declare_parameter(name_ + ".lethal_cost", static_cast<int>(lethal_cost_));
  node->declare_parameter(name_ + ".cost_travel_multiplier", cost_travel_multiplier_);
  node->declare_parameter(name_ + ".max_iterations", max_iterations_);
  node->declare_parameter(name_ + ".minco_iterations", minco_iterations_);
  node->declare_parameter(name_ + ".minco_step_size", minco_step_size_);
  node->declare_parameter(name_ + ".jps_path_topic", "jps_path");
  node->declare_parameter(name_ + ".minco_path_topic", "minco_path");

  int lethal_cost = static_cast<int>(lethal_cost_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".lethal_cost", lethal_cost);
  node->get_parameter(name_ + ".cost_travel_multiplier", cost_travel_multiplier_);
  node->get_parameter(name_ + ".max_iterations", max_iterations_);
  node->get_parameter(name_ + ".minco_iterations", minco_iterations_);
  node->get_parameter(name_ + ".minco_step_size", minco_step_size_);
  std::string jps_path_topic;
  std::string minco_path_topic;
  node->get_parameter(name_ + ".jps_path_topic", jps_path_topic);
  node->get_parameter(name_ + ".minco_path_topic", minco_path_topic);
  lethal_cost_ = static_cast<unsigned char>(std::clamp(lethal_cost, 1, 254));
  jps_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    jps_path_topic, rclcpp::QoS(1).transient_local());
  minco_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    minco_path_topic, rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(
    logger_, "Configured %s: JPS with collision-checked minimum-control smoothing", name_.c_str());
}

void JpsMincoPlanner::cleanup()
{
  jps_path_pub_.reset();
  minco_path_pub_.reset();
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
}

void JpsMincoPlanner::activate() {}

void JpsMincoPlanner::deactivate() {}

bool JpsMincoPlanner::isTraversable(int x, int y) const
{
  if (x < 0 || y < 0 || x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
    y >= static_cast<int>(costmap_->getSizeInCellsY())) {
    return false;
  }

  const auto cost = costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return cost < lethal_cost_;
}

bool JpsMincoPlanner::isStepTraversable(int x, int y, int dx, int dy) const
{
  if (!isTraversable(x + dx, y + dy)) {
    return false;
  }
  return dx == 0 || dy == 0 || (isTraversable(x + dx, y) && isTraversable(x, y + dy));
}

bool JpsMincoPlanner::hasForcedNeighbor(int x, int y, int dx, int dy) const
{
  if (dx != 0 && dy != 0) {
    return (!isTraversable(x - dx, y) && isTraversable(x - dx, y + dy)) ||
           (!isTraversable(x, y - dy) && isTraversable(x + dx, y - dy));
  }
  if (dx != 0) {
    return (!isTraversable(x, y + 1) && isTraversable(x + dx, y + 1)) ||
           (!isTraversable(x, y - 1) && isTraversable(x + dx, y - 1));
  }
  return (!isTraversable(x + 1, y) && isTraversable(x + 1, y + dy)) ||
         (!isTraversable(x - 1, y) && isTraversable(x - 1, y + dy));
}

JpsMincoPlanner::Cell JpsMincoPlanner::jump(int x, int y, int dx, int dy, const Cell & goal) const
{
  if (!isStepTraversable(x, y, dx, dy)) {
    return {-1, -1};
  }

  const Cell next{x + dx, y + dy};
  if (next.x == goal.x && next.y == goal.y) {
    return next;
  }
  if (hasForcedNeighbor(next.x, next.y, dx, dy)) {
    return next;
  }
  if (dx != 0 && dy != 0 &&
    (jump(next.x, next.y, dx, 0, goal).x >= 0 || jump(next.x, next.y, 0, dy, goal).x >= 0)) {
    return next;
  }
  return jump(next.x, next.y, dx, dy, goal);
}

std::vector<JpsMincoPlanner::Cell> JpsMincoPlanner::prunedDirections(
  const Cell & current, const Cell * parent) const
{
  if (!parent) {
    return {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
  }

  const int dx = std::clamp(current.x - parent->x, -1, 1);
  const int dy = std::clamp(current.y - parent->y, -1, 1);
  std::vector<Cell> directions;
  if (dx != 0 && dy != 0) {
    directions = {{dx, dy}, {dx, 0}, {0, dy}};
    if (!isTraversable(current.x - dx, current.y)) {
      directions.push_back({-dx, dy});
    }
    if (!isTraversable(current.x, current.y - dy)) {
      directions.push_back({dx, -dy});
    }
  } else if (dx != 0) {
    directions.push_back({dx, 0});
    if (!isTraversable(current.x, current.y + 1)) {
      directions.push_back({dx, 1});
    }
    if (!isTraversable(current.x, current.y - 1)) {
      directions.push_back({dx, -1});
    }
  } else {
    directions.push_back({0, dy});
    if (!isTraversable(current.x + 1, current.y)) {
      directions.push_back({1, dy});
    }
    if (!isTraversable(current.x - 1, current.y)) {
      directions.push_back({-1, dy});
    }
  }
  return directions;
}

double JpsMincoPlanner::movementCost(const Cell & from, const Cell & to) const
{
  const int steps = std::max(std::abs(to.x - from.x), std::abs(to.y - from.y));
  double cost = 0.0;
  Cell previous = from;
  for (int step = 1; step <= steps; ++step) {
    const Cell current{
      from.x + (to.x - from.x) * step / steps, from.y + (to.y - from.y) * step / steps};
    const auto cell_cost = costmap_->getCost(
      static_cast<unsigned int>(current.x), static_cast<unsigned int>(current.y));
    const double distance = std::hypot(
      static_cast<double>(current.x - previous.x), static_cast<double>(current.y - previous.y));
    cost += distance * (1.0 + cost_travel_multiplier_ * static_cast<double>(cell_cost) / 252.0);
    previous = current;
  }
  return cost;
}

std::vector<JpsMincoPlanner::Cell> JpsMincoPlanner::jpsSearch(
  const Cell & start, const Cell & goal) const
{
  const int width = static_cast<int>(costmap_->getSizeInCellsX());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());
  const auto index = [width](const Cell & cell) { return cell.y * width + cell.x; };
  const int total = width * height;
  std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
  std::vector<int> parent(total, -1);
  std::priority_queue<QueueEntry> open;

  g_score[index(start)] = 0.0;
  open.push({index(start), std::hypot(static_cast<double>(start.x - goal.x),
                                      static_cast<double>(start.y - goal.y))});
  int iterations = 0;
  while (!open.empty() && iterations++ < max_iterations_) {
    const auto current_entry = open.top();
    open.pop();
    const Cell current{current_entry.index % width, current_entry.index / width};
    if (current.x == goal.x && current.y == goal.y) {
      std::vector<Cell> reverse_path;
      for (
        int current_index = index(goal); current_index >= 0;
        current_index = parent[current_index]) {
        reverse_path.push_back({current_index % width, current_index / width});
      }
      std::reverse(reverse_path.begin(), reverse_path.end());
      return reverse_path;
    }

    Cell parent_cell{};
    const int parent_index = parent[index(current)];
    const Cell * parent_ptr = nullptr;
    if (parent_index >= 0) {
      parent_cell = {parent_index % width, parent_index / width};
      parent_ptr = &parent_cell;
    }

    for (const auto & direction : prunedDirections(current, parent_ptr)) {
      const Cell successor = jump(current.x, current.y, direction.x, direction.y, goal);
      if (successor.x < 0) {
        continue;
      }
      const int successor_index = index(successor);
      const double tentative_g = g_score[index(current)] + movementCost(current, successor);
      if (tentative_g >= g_score[successor_index]) {
        continue;
      }
      g_score[successor_index] = tentative_g;
      parent[successor_index] = index(current);
      open.push(
        {successor_index,
          tentative_g + std::hypot(static_cast<double>(successor.x - goal.x),
                                    static_cast<double>(successor.y - goal.y))});
    }
  }
  return {};
}

std::vector<JpsMincoPlanner::Cell> JpsMincoPlanner::densify(
  const std::vector<Cell> & jump_path) const
{
  if (jump_path.empty()) {
    return {};
  }
  std::vector<Cell> path{jump_path.front()};
  for (size_t i = 1; i < jump_path.size(); ++i) {
    const Cell from = jump_path[i - 1];
    const Cell to = jump_path[i];
    const int steps = std::max(std::abs(to.x - from.x), std::abs(to.y - from.y));
    for (int step = 1; step <= steps; ++step) {
      path.push_back(
        {from.x + (to.x - from.x) * step / steps, from.y + (to.y - from.y) * step / steps});
    }
  }
  return path;
}

bool JpsMincoPlanner::segmentTraversable(const Cell & from, const Cell & to) const
{
  const int steps = std::max(std::abs(to.x - from.x), std::abs(to.y - from.y));
  if (steps == 0) {
    return isTraversable(from.x, from.y);
  }
  Cell previous = from;
  for (int step = 1; step <= steps; ++step) {
    const Cell current{
      from.x + (to.x - from.x) * step / steps, from.y + (to.y - from.y) * step / steps};
    if (!isStepTraversable(
        previous.x, previous.y, current.x - previous.x, current.y - previous.y)) {
      return false;
    }
    previous = current;
  }
  return true;
}

std::vector<JpsMincoPlanner::Cell> JpsMincoPlanner::mincoSmooth(
  const std::vector<Cell> & path) const
{
  if (path.size() < 3) {
    return path;
  }

  // First remove unnecessary JPS samples. The remaining knots define a minimum-control path.
  std::vector<Cell> knots{path.front()};
  size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    size_t furthest = anchor + 1;
    for (size_t candidate = furthest + 1; candidate < path.size(); ++candidate) {
      if (!segmentTraversable(path[anchor], path[candidate])) {
        break;
      }
      furthest = candidate;
    }
    knots.push_back(path[furthest]);
    anchor = furthest;
  }

  // Discrete minimum-control optimization: reduce squared second derivatives while keeping
  // every candidate collision-free. This is the spatial part of the MINCO trajectory.
  for (int iteration = 0; iteration < minco_iterations_ && knots.size() > 2; ++iteration) {
    bool changed = false;
    for (size_t i = 1; i + 1 < knots.size(); ++i) {
      const Cell original = knots[i];
      const Cell target{
        static_cast<int>(std::lround((knots[i - 1].x + knots[i + 1].x) * 0.5)),
        static_cast<int>(std::lround((knots[i - 1].y + knots[i + 1].y) * 0.5))};
      const int max_step = std::max(
        1, static_cast<int>(std::lround(minco_step_size_ / costmap_->getResolution())));
      const Cell candidate{
        original.x + std::clamp(target.x - original.x, -max_step, max_step),
        original.y + std::clamp(target.y - original.y, -max_step, max_step)};
      if ((candidate.x != original.x || candidate.y != original.y) &&
        segmentTraversable(knots[i - 1], candidate) &&
        segmentTraversable(candidate, knots[i + 1])) {
        knots[i] = candidate;
        changed = true;
      }
    }
    if (!changed) {
      break;
    }
  }

  std::vector<Cell> smoothed;
  for (size_t i = 1; i < knots.size(); ++i) {
    const auto segment = densify({knots[i - 1], knots[i]});
    smoothed.insert(smoothed.end(), segment.begin() + (i == 1 ? 0 : 1), segment.end());
  }
  return smoothed;
}

geometry_msgs::msg::PoseStamped JpsMincoPlanner::toPose(
  const Cell & cell, const std::string & frame) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = costmap_->getOriginX() +
                         (static_cast<double>(cell.x) + 0.5) * costmap_->getResolution();
  pose.pose.position.y = costmap_->getOriginY() +
                         (static_cast<double>(cell.y) + 0.5) * costmap_->getResolution();
  pose.pose.orientation.w = 1.0;
  return pose;
}

nav_msgs::msg::Path JpsMincoPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path plan;
  plan.header.frame_id = global_frame_;
  plan.header.stamp = start.header.stamp;
  if (!costmap_) {
    RCLCPP_ERROR(logger_, "JpsMincoPlanner is not configured with a costmap");
    return plan;
  }

  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
    !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    RCLCPP_WARN(logger_, "Start or goal is outside the global costmap");
    return plan;
  }

  const Cell start_cell{static_cast<int>(start_x), static_cast<int>(start_y)};
  const Cell goal_cell{static_cast<int>(goal_x), static_cast<int>(goal_y)};
  if (!isTraversable(start_cell.x, start_cell.y) || !isTraversable(goal_cell.x, goal_cell.y)) {
    RCLCPP_WARN(logger_, "Start or goal lies in a non-traversable costmap cell");
    return plan;
  }

  const auto jump_path = jpsSearch(start_cell, goal_cell);
  if (jump_path.empty()) {
    RCLCPP_WARN(logger_, "JPS found no path");
    return plan;
  }

  nav_msgs::msg::Path jps_path;
  jps_path.header = plan.header;
  jps_path.poses.reserve(jump_path.size());
  for (const auto & cell : jump_path) {
    jps_path.poses.push_back(toPose(cell, global_frame_));
  }
  if (!jps_path.poses.empty()) {
    jps_path.poses.front() = start;
    jps_path.poses.front().header.frame_id = global_frame_;
    jps_path.poses.back() = goal;
    jps_path.poses.back().header.frame_id = global_frame_;
  }
  if (jps_path_pub_) {
    jps_path_pub_->publish(jps_path);
  }

  const auto smoothed_path = mincoSmooth(densify(jump_path));
  plan.poses.reserve(smoothed_path.size());
  for (const auto & cell : smoothed_path) {
    plan.poses.push_back(toPose(cell, global_frame_));
  }
  for (size_t i = 0; i + 1 < plan.poses.size(); ++i) {
    const double yaw = std::atan2(
      plan.poses[i + 1].pose.position.y - plan.poses[i].pose.position.y,
      plan.poses[i + 1].pose.position.x - plan.poses[i].pose.position.x);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    plan.poses[i].pose.orientation = tf2::toMsg(orientation);
  }
  if (!plan.poses.empty()) {
    plan.poses.front() = start;
    plan.poses.front().header.frame_id = global_frame_;
    plan.poses.back() = goal;
    plan.poses.back().header.frame_id = global_frame_;
  }
  if (minco_path_pub_) {
    minco_path_pub_->publish(plan);
  }
  return plan;
}

}  // namespace pb_nav2_planners

PLUGINLIB_EXPORT_CLASS(pb_nav2_planners::JpsMincoPlanner, nav2_core::GlobalPlanner)

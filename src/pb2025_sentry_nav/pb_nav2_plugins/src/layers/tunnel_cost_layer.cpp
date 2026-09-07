#include "pb_nav2_plugins/tunnel_cost_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace pb_nav2_costmap_2d
{

namespace
{

constexpr size_t kCornersPerTunnel = 4;

bool containsId(const std::vector<int64_t> & ids, int64_t id)
{
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

}  // namespace

void TunnelCostLayer::onInitialize()
{
  const auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TunnelCostLayer could not lock its lifecycle node");
  }

  declareParameter("corner_0_xs", std::vector<double>{});
  declareParameter("corner_0_ys", std::vector<double>{});
  declareParameter("corner_1_xs", std::vector<double>{});
  declareParameter("corner_1_ys", std::vector<double>{});
  declareParameter("corner_2_xs", std::vector<double>{});
  declareParameter("corner_2_ys", std::vector<double>{});
  declareParameter("corner_3_xs", std::vector<double>{});
  declareParameter("corner_3_ys", std::vector<double>{});
  declareParameter("entry_xs", std::vector<double>{});
  declareParameter("entry_ys", std::vector<double>{});
  declareParameter("exit_xs", std::vector<double>{});
  declareParameter("exit_ys", std::vector<double>{});
  declareParameter("entry_to_exit_ids", std::vector<int64_t>{});
  declareParameter("exit_to_entry_ids", std::vector<int64_t>{});
  declareParameter("blocked_tunnel_topic", std::string("blocked_tunnel_id"));

  const auto readDoubles = [node, this](const std::string & suffix) {
      std::vector<double> values;
      node->get_parameter(name_ + "." + suffix, values);
      return values;
    };
  const auto corner_0_xs = readDoubles("corner_0_xs");
  const auto corner_0_ys = readDoubles("corner_0_ys");
  const auto corner_1_xs = readDoubles("corner_1_xs");
  const auto corner_1_ys = readDoubles("corner_1_ys");
  const auto corner_2_xs = readDoubles("corner_2_xs");
  const auto corner_2_ys = readDoubles("corner_2_ys");
  const auto corner_3_xs = readDoubles("corner_3_xs");
  const auto corner_3_ys = readDoubles("corner_3_ys");
  const auto entry_xs = readDoubles("entry_xs");
  const auto entry_ys = readDoubles("entry_ys");
  const auto exit_xs = readDoubles("exit_xs");
  const auto exit_ys = readDoubles("exit_ys");

  std::vector<int64_t> entry_to_exit_ids;
  std::vector<int64_t> exit_to_entry_ids;
  node->get_parameter(name_ + ".entry_to_exit_ids", entry_to_exit_ids);
  node->get_parameter(name_ + ".exit_to_entry_ids", exit_to_entry_ids);
  node->get_parameter(name_ + ".blocked_tunnel_topic", blocked_tunnel_topic_);

  const size_t tunnel_count = corner_0_xs.size();
  const std::array<size_t, 12> lengths = {
    corner_0_xs.size(), corner_0_ys.size(), corner_1_xs.size(), corner_1_ys.size(),
    corner_2_xs.size(), corner_2_ys.size(), corner_3_xs.size(), corner_3_ys.size(),
    entry_xs.size(), entry_ys.size(), exit_xs.size(), exit_ys.size()};
  if (std::any_of(lengths.begin(), lengths.end(), [tunnel_count](size_t length) {
      return length != tunnel_count;
    }))
  {
    throw std::runtime_error("TunnelCostLayer polygon, entry, and exit parameter arrays must have equal length");
  }

  tunnels_.clear();
  tunnels_.reserve(tunnel_count);
  for (size_t index = 0; index < tunnel_count; ++index) {
    int oneway_dir = 0;
    if (containsId(entry_to_exit_ids, static_cast<int64_t>(index))) {
      oneway_dir = 1;
    }
    if (containsId(exit_to_entry_ids, static_cast<int64_t>(index))) {
      if (oneway_dir != 0) {
        throw std::runtime_error("A tunnel cannot be configured in both one-way direction lists");
      }
      oneway_dir = -1;
    }
    tunnels_.push_back({{
      Point{corner_0_xs[index], corner_0_ys[index]},
      Point{corner_1_xs[index], corner_1_ys[index]},
      Point{corner_2_xs[index], corner_2_ys[index]},
      Point{corner_3_xs[index], corner_3_ys[index]}},
      Point{entry_xs[index], entry_ys[index]}, Point{exit_xs[index], exit_ys[index]}, oneway_dir});
  }
  direction_blocked_.assign(tunnels_.size(), false);

  blocked_tunnel_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    blocked_tunnel_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::Int32::SharedPtr message) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      blocked_tunnel_id_ = message->data;
    });
  current_ = true;

  RCLCPP_INFO(
    logger_, "Loaded %zu polygon tunnels; entry-to-exit=%zu, exit-to-entry=%zu, blocked topic='%s'",
    tunnels_.size(), entry_to_exit_ids.size(), exit_to_entry_ids.size(), blocked_tunnel_topic_.c_str());
}

void TunnelCostLayer::reset()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  blocked_tunnel_id_ = -1;
  std::fill(direction_blocked_.begin(), direction_blocked_.end(), false);
  current_ = true;
}

bool TunnelCostLayer::pointInPolygon(const Point & point, const std::array<Point, 4> & polygon)
{
  bool inside = false;
  for (size_t current = 0, previous = kCornersPerTunnel - 1; current < kCornersPerTunnel;
       previous = current++)
  {
    const auto & a = polygon[current];
    const auto & b = polygon[previous];
    const bool crosses = ((a.y > point.y) != (b.y > point.y)) &&
      (point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

bool TunnelCostLayer::shouldBlockForDirection(
  const TunnelDef & tunnel, double robot_x, double robot_y) const
{
  if (tunnel.oneway_dir == 0) {
    return false;
  }
  // A robot that has already entered from the permitted side must be able to
  // finish traversing the tunnel. Direction filtering only applies while it is
  // outside the tunnel, approaching from one end.
  if (pointInPolygon(Point{robot_x, robot_y}, tunnel.polygon)) {
    return false;
  }
  const double direction_x = tunnel.exit.x - tunnel.entry.x;
  const double direction_y = tunnel.exit.y - tunnel.entry.y;
  const double squared_length = direction_x * direction_x + direction_y * direction_y;
  if (squared_length <= std::numeric_limits<double>::epsilon()) {
    return true;
  }
  const double midpoint_x = (tunnel.entry.x + tunnel.exit.x) * 0.5;
  const double midpoint_y = (tunnel.entry.y + tunnel.exit.y) * 0.5;
  const double projection =
    ((robot_x - midpoint_x) * direction_x + (robot_y - midpoint_y) * direction_y) / squared_length;
  return tunnel.oneway_dir == 1 ? projection > 0.0 : projection < 0.0;
}

bool TunnelCostLayer::tunnelIsBlocked(size_t tunnel_id) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return direction_blocked_[tunnel_id] || blocked_tunnel_id_ == static_cast<int>(tunnel_id);
}

void TunnelCostLayer::updateBounds(
  double robot_x, double robot_y, double,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t index = 0; index < tunnels_.size(); ++index) {
    direction_blocked_[index] = shouldBlockForDirection(tunnels_[index], robot_x, robot_y);
    // Keep every tunnel inside the update window. When a temporary block expires,
    // Nav2 then refreshes the formerly lethal cells on this same update cycle.
    for (const auto & point : tunnels_[index].polygon) {
      *min_x = std::min(*min_x, point.x);
      *min_y = std::min(*min_y, point.y);
      *max_x = std::max(*max_x, point.x);
      *max_y = std::max(*max_y, point.y);
    }
  }
}

bool TunnelCostLayer::isInsideMapBounds(
  unsigned int x, unsigned int y, int min_i, int min_j, int max_i, int max_j)
{
  return static_cast<int>(x) >= min_i && static_cast<int>(x) < max_i &&
    static_cast<int>(y) >= min_j && static_cast<int>(y) < max_j;
}

void TunnelCostLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  for (size_t index = 0; index < tunnels_.size(); ++index) {
    if (!tunnelIsBlocked(index)) {
      continue;
    }

    const auto & polygon = tunnels_[index].polygon;
    const auto minmax_x = std::minmax_element(
      polygon.begin(), polygon.end(), [](const Point & left, const Point & right) {return left.x < right.x;});
    const auto minmax_y = std::minmax_element(
      polygon.begin(), polygon.end(), [](const Point & left, const Point & right) {return left.y < right.y;});

    unsigned int lower_x;
    unsigned int lower_y;
    unsigned int upper_x;
    unsigned int upper_y;
    if (!master_grid.worldToMap(minmax_x.first->x, minmax_y.first->y, lower_x, lower_y) ||
      !master_grid.worldToMap(minmax_x.second->x, minmax_y.second->y, upper_x, upper_y))
    {
      continue;
    }

    for (unsigned int x = lower_x; x <= upper_x && x < master_grid.getSizeInCellsX(); ++x) {
      for (unsigned int y = lower_y; y <= upper_y && y < master_grid.getSizeInCellsY(); ++y) {
        if (!isInsideMapBounds(x, y, min_i, min_j, max_i, max_j)) {
          continue;
        }
        double world_x;
        double world_y;
        master_grid.mapToWorld(x, y, world_x, world_y);
        if (pointInPolygon(Point{world_x, world_y}, polygon)) {
          master_grid.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
}

}  // namespace pb_nav2_costmap_2d

PLUGINLIB_EXPORT_CLASS(pb_nav2_costmap_2d::TunnelCostLayer, nav2_costmap_2d::Layer)

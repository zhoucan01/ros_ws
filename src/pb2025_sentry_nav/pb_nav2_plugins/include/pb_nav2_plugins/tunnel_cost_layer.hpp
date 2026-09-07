#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace pb_nav2_costmap_2d
{

class TunnelCostLayer : public nav2_costmap_2d::Layer
{
public:
  TunnelCostLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  struct Point
  {
    double x{};
    double y{};
  };

  struct TunnelDef
  {
    std::array<Point, 4> polygon;
    Point entry;
    Point exit;
    int oneway_dir{};  // 0: bidirectional, 1: entry -> exit, -1: exit -> entry
  };

  static bool pointInPolygon(const Point & point, const std::array<Point, 4> & polygon);
  static bool isInsideMapBounds(
    unsigned int x, unsigned int y, int min_i, int min_j, int max_i, int max_j);
  bool shouldBlockForDirection(const TunnelDef & tunnel, double robot_x, double robot_y) const;
  bool tunnelIsBlocked(size_t tunnel_id) const;

  std::vector<TunnelDef> tunnels_;
  std::vector<bool> direction_blocked_;
  std::string blocked_tunnel_topic_;
  int blocked_tunnel_id_{-1};
  mutable std::mutex state_mutex_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr blocked_tunnel_sub_;
};

}  // namespace pb_nav2_costmap_2d

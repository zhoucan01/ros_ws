#ifndef SENTRY_BT_ANTI_AUTOAIM_HPP
#define SENTRY_BT_ANTI_AUTOAIM_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <sentry_decision_msg/msg/enemy_pos.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "geometry_msgs/msg/point.hpp"
// #include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <array>

using PointStamped = geometry_msgs::msg::PointStamped;
using Point = geometry_msgs::msg::Point;
using PoseStamped = geometry_msgs::msg::PoseStamped;

// namespace pb2025_sentry_behavior
// {
class AntiAutoAim : public BT::SyncActionNode
{
public:
  AntiAutoAim(const std::string &name, const BT::NodeConfig &config,
              std::shared_ptr<rclcpp::Node> node);

  // bool setMessage(visualization_msgs::msg::MarkerArray &msg) override;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  std::vector<geometry_msgs::msg::Point> generateCandidatePoints(const Point enemy_point);
  std::vector<Point> filterFeasiblePoints(const std::vector<Point> &candidates, const nav_msgs::msg::OccupancyGrid &costmap);
  Point selectBestPoint(
    const std::vector<Point> &feasible_points, const Point &robot_position,
    const nav_msgs::msg::OccupancyGrid &costmap);
  PoseStamped createAttackPose(const Point &attack_point, const Point &enemy_position);
  void createVisualizationMarkers(
      visualization_msgs::msg::MarkerArray &msg, const Point &enemy_position,
      const std::vector<Point> &candidates, const std::vector<Point> &feasible_points,
      const Point &robot_position, const nav_msgs::msg::OccupancyGrid &costmap,
      bool limit_enabled, const std::array<double, 4> &limit_rect);
  bool getDecisionPointRect(int point_id, std::array<double, 4> &rect) const;
  // PoseStamped AntiAutoAim::createAttackPose(const Point & attack_point, const PointStamped & enemy_position)

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  visualization_msgs::msg::MarkerArray msg;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub_;

  struct Parameters
  {
    double attack_radius = 1.5;
    int num_sectors = 36;
    int cost_threshold = 10;
    std::string robot_base_frame = "base_link";
    double transform_tolerance = 0.1;
    double max_visualization_distance = 6.0;
    double marker_scale_base = 0.2;
    bool visualize = true;
    bool enable_attack = true;
    bool limit_chase_range = false;
    double max_chase_distance = 3.0;
    double distance_weight = 0.6;
    double cost_weight = 0.4;
    // double marker_scale_base = 0.1;
  } params_;

  struct Policy
  {
    bool enable = true;
    bool limit = false;
  };
  std::unordered_map<std::string, Policy> policy_map_;
};
// }

#endif

#ifndef PB_NAV2_PLUGINS__JPS_MINCO_PLANNER_HPP_
#define PB_NAV2_PLUGINS__JPS_MINCO_PLANNER_HPP_

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace pb_nav2_planners
{

class JpsMincoPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct Cell
  {
    int x;
    int y;
  };

  struct QueueEntry
  {
    int index;
    double score;

    bool operator<(const QueueEntry & other) const { return score > other.score; }
  };

  bool isTraversable(int x, int y) const;
  bool isStepTraversable(int x, int y, int dx, int dy) const;
  bool hasForcedNeighbor(int x, int y, int dx, int dy) const;
  Cell jump(int x, int y, int dx, int dy, const Cell & goal) const;
  std::vector<Cell> prunedDirections(const Cell & current, const Cell * parent) const;
  std::vector<Cell> jpsSearch(const Cell & start, const Cell & goal) const;
  std::vector<Cell> densify(const std::vector<Cell> & jump_path) const;
  std::vector<Cell> mincoSmooth(const std::vector<Cell> & path) const;
  bool segmentTraversable(const Cell & from, const Cell & to) const;
  double movementCost(const Cell & from, const Cell & to) const;
  geometry_msgs::msg::PoseStamped toPose(const Cell & cell, const std::string & frame) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("JpsMincoPlanner")};
  std::string name_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr jps_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr minco_path_pub_;
  bool allow_unknown_{true};
  unsigned char lethal_cost_{253};
  double cost_travel_multiplier_{2.0};
  int max_iterations_{200000};
  int minco_iterations_{80};
  double minco_step_size_{0.25};
};

}  // namespace pb_nav2_planners

#endif  // PB_NAV2_PLUGINS__JPS_MINCO_PLANNER_HPP_

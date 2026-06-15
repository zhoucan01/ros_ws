#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_decision_msg/msg/tunnel_monitor.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

enum TunnelStatus
{
  NORMAL = 0,
  WILL_PASS = 1,
  APPROACHING = 2,
  IN_TUNNEL = 3,
  PASSED = 4,
  RECOVERY_ACTIVE = 5
};

struct RectRegion
{
  double x_min{};
  double x_max{};
  double y_min{};
  double y_max{};

  bool contains(double x, double y, double margin = 0.0) const
  {
    return x >= x_min - margin && x <= x_max + margin && y >= y_min - margin &&
           y <= y_max + margin;
  }
};

class TunnelRegionMonitor : public rclcpp::Node
{
public:
  TunnelRegionMonitor()
  : Node("tunnel_region_monitor"),
    enable_tunnel_mode_(true),
    in_tunnel_(false),
    has_pose_(false),
    has_plan_(false),
    will_pass_tunnel_(false),
    disable_spin_(false),
    tunnel_recovery_active_(false),
    low_clearance_mode_(false),
    default_controller_id_("FollowPath"),
    tunnel_controller_id_("TunnelFollowPath"),
    activation_margin_(0.15),
    path_check_margin_(0.4),
    disable_spin_margin_(0.3),
    publish_rate_(2.0),
    transform_tolerance_(0.1),
    pose_hold_timeout_(0.5),
    approach_stuck_timeout_(2.0),
    tunnel_stuck_timeout_(1.5),
    min_progress_distance_(0.15),
    recovery_retreat_distance_(0.8),
    recovery_clear_margin_(0.05),
    map_origin_offset_x_(0.0),
    map_origin_offset_y_(0.0),
    min_path_points_in_region_(3),
    target_tunnel_id_(-1),
    tracked_tunnel_id_(-1),
    recovery_tunnel_id_(-1),
    approach_tunnel_id_(-1),
    tunnel_status_(NORMAL),
    tunnel_count_(0),
    robot_x_(0.0),
    robot_y_(0.0),
    current_map_yaw_(0.0),
    tunnel_target_yaw_(0.0),
    tunnel_yaw_error_(0.0),
    progress_reference_x_(0.0),
    progress_reference_y_(0.0),
    tracking_in_tunnel_phase_(false),
    approach_from_exit_side_(false)
  {
    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "odometry");
    controller_selector_topic_ =
      this->declare_parameter<std::string>("controller_selector_topic", "controller_selector");
    tunnel_state_topic_ = this->declare_parameter<std::string>("tunnel_state_topic", "in_tunnel");
    global_pose_topic_ = this->declare_parameter<std::string>("global_pose_topic", "global_pose");
    global_point_topic_ =
      this->declare_parameter<std::string>("global_point_topic", "global_point");
    global_plan_topic_ = this->declare_parameter<std::string>("global_plan_topic", "plan");
    will_pass_tunnel_topic_ =
      this->declare_parameter<std::string>("will_pass_tunnel_topic", "will_pass_tunnel");
    target_tunnel_id_topic_ =
      this->declare_parameter<std::string>("target_tunnel_id_topic", "target_tunnel_id");
    tunnel_status_topic_ =
      this->declare_parameter<std::string>("tunnel_status_topic", "tunnel_status");
    disable_spin_topic_ =
      this->declare_parameter<std::string>("disable_spin_topic", "disable_spin");
    tunnel_recovery_active_topic_ = this->declare_parameter<std::string>(
      "tunnel_recovery_active_topic", "tunnel_recovery_active");
    tunnel_recovery_goal_topic_ =
      this->declare_parameter<std::string>("tunnel_recovery_goal_topic", "tunnel_recovery_goal");
    tunnel_target_yaw_topic_ =
      this->declare_parameter<std::string>("tunnel_target_yaw_topic", "tunnel_target_yaw");
    current_map_yaw_topic_ =
      this->declare_parameter<std::string>("current_map_yaw_topic", "current_map_yaw");
    tunnel_yaw_error_topic_ =
      this->declare_parameter<std::string>("tunnel_yaw_error_topic", "tunnel_yaw_error");
    tunnel_markers_topic_ =
      this->declare_parameter<std::string>("tunnel_markers_topic", "tunnel_markers");
    tunnel_monitor_topic_ =
      this->declare_parameter<std::string>("tunnel_monitor_topic", "tunnel_monitor");
    low_clearance_mode_topic_ =
      this->declare_parameter<std::string>("low_clearance_mode_topic", "low_clearance_mode");
    enable_tunnel_mode_ = this->declare_parameter<bool>("enable_tunnel_mode", true);
    default_controller_id_ =
      this->declare_parameter<std::string>("default_controller_id", "FollowPath");
    tunnel_controller_id_ =
      this->declare_parameter<std::string>("tunnel_controller_id", "TunnelFollowPath");
    activation_margin_ = this->declare_parameter<double>("activation_margin", 0.15);
    path_check_margin_ = this->declare_parameter<double>("path_check_margin", 0.4);
    disable_spin_margin_ = this->declare_parameter<double>("disable_spin_margin", 0.3);
    min_path_points_in_region_ = this->declare_parameter<int>("min_path_points_in_region", 3);
    publish_rate_ = this->declare_parameter<double>("publish_rate", 2.0);
    transform_tolerance_ = this->declare_parameter<double>("transform_tolerance", 0.1);
    pose_hold_timeout_ = this->declare_parameter<double>("pose_hold_timeout", 0.5);
    approach_stuck_timeout_ = this->declare_parameter<double>("approach_stuck_timeout", 2.0);
    tunnel_stuck_timeout_ = this->declare_parameter<double>("tunnel_stuck_timeout", 1.5);
    min_progress_distance_ = this->declare_parameter<double>("min_progress_distance", 0.15);
    recovery_retreat_distance_ =
      this->declare_parameter<double>("recovery_retreat_distance", 0.8);
    recovery_clear_margin_ = this->declare_parameter<double>("recovery_clear_margin", 0.05);
    map_origin_offset_x_ = this->declare_parameter<double>("map_origin_offset_x", 0.0);
    map_origin_offset_y_ = this->declare_parameter<double>("map_origin_offset_y", 0.0);
    low_clearance_tunnel_ids_ = this->declare_parameter<std::vector<int64_t>>(
      "low_clearance_tunnel_ids", std::vector<int64_t>{});
    enable_low_clearance_param_switch_ =
      this->declare_parameter<bool>("enable_low_clearance_param_switch", false);
    low_clearance_vehicle_height_ =
      this->declare_parameter<double>("low_clearance_vehicle_height", 0.20);
    normal_vehicle_height_ =
      this->declare_parameter<double>("normal_vehicle_height", 0.23);
    low_clearance_target_nodes_ = this->declare_parameter<std::vector<std::string>>(
      "low_clearance_target_nodes",
      std::vector<std::string>{"terrain_analysis", "terrain_analysis_ext"});

    const auto empty_vec = std::vector<double>{};
    trigger_x_mins_ = this->declare_parameter<std::vector<double>>("trigger_x_mins", empty_vec);
    trigger_x_maxs_ = this->declare_parameter<std::vector<double>>("trigger_x_maxs", empty_vec);
    trigger_y_mins_ = this->declare_parameter<std::vector<double>>("trigger_y_mins", empty_vec);
    trigger_y_maxs_ = this->declare_parameter<std::vector<double>>("trigger_y_maxs", empty_vec);
    tunnel_x_mins_ = this->declare_parameter<std::vector<double>>("tunnel_x_mins", empty_vec);
    tunnel_x_maxs_ = this->declare_parameter<std::vector<double>>("tunnel_x_maxs", empty_vec);
    tunnel_y_mins_ = this->declare_parameter<std::vector<double>>("tunnel_y_mins", empty_vec);
    tunnel_y_maxs_ = this->declare_parameter<std::vector<double>>("tunnel_y_maxs", empty_vec);
    entry_xs_ = this->declare_parameter<std::vector<double>>("entry_xs", empty_vec);
    entry_ys_ = this->declare_parameter<std::vector<double>>("entry_ys", empty_vec);
    exit_xs_ = this->declare_parameter<std::vector<double>>("exit_xs", empty_vec);
    exit_ys_ = this->declare_parameter<std::vector<double>>("exit_ys", empty_vec);

    validateRegions();
    applyCoordinateOffset();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    controller_selector_pub_ =
      this->create_publisher<std_msgs::msg::String>(controller_selector_topic_, latched_qos);
    tunnel_state_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(tunnel_state_topic_, latched_qos);
    global_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(global_pose_topic_, latched_qos);
    global_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(global_point_topic_, latched_qos);
    will_pass_tunnel_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(will_pass_tunnel_topic_, latched_qos);
    target_tunnel_id_pub_ =
      this->create_publisher<std_msgs::msg::Int32>(target_tunnel_id_topic_, latched_qos);
    tunnel_status_pub_ =
      this->create_publisher<std_msgs::msg::Int32>(tunnel_status_topic_, latched_qos);
    disable_spin_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(disable_spin_topic_, latched_qos);
    tunnel_recovery_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      tunnel_recovery_active_topic_, latched_qos);
    tunnel_recovery_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      tunnel_recovery_goal_topic_, latched_qos);
    tunnel_target_yaw_pub_ =
      this->create_publisher<std_msgs::msg::Float64>(tunnel_target_yaw_topic_, latched_qos);
    current_map_yaw_pub_ =
      this->create_publisher<std_msgs::msg::Float64>(current_map_yaw_topic_, latched_qos);
    tunnel_yaw_error_pub_ =
      this->create_publisher<std_msgs::msg::Float64>(tunnel_yaw_error_topic_, latched_qos);
    tunnel_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      tunnel_markers_topic_, latched_qos);
    tunnel_monitor_pub_ = this->create_publisher<sentry_decision_msg::msg::TunnelMonitor>(
      tunnel_monitor_topic_, latched_qos);
    low_clearance_mode_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(low_clearance_mode_topic_, latched_qos);

    for (const auto & node_name : low_clearance_target_nodes_) {
    low_clearance_param_clients_.push_back(
        std::make_shared<rclcpp::AsyncParametersClient>(
          this->get_node_base_interface(),
          this->get_node_topics_interface(),
          this->get_node_graph_interface(),
          this->get_node_services_interface(),
          resolveTargetNodeName(node_name)));
      low_clearance_target_nodes_resolved_.push_back(resolveTargetNodeName(node_name));
    }

    global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      global_plan_topic_, rclcpp::QoS(10),
      std::bind(&TunnelRegionMonitor::planCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&TunnelRegionMonitor::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "TunnelRegionMonitor using odom pose from topic '%s' and transforming it to '%s' for global position.",
      odom_topic_.c_str(), global_frame_.c_str());

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_));
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TunnelRegionMonitor::update, this));

    tunnel_recovery_goal_.header.frame_id = global_frame_;
    tunnel_recovery_goal_.pose.orientation.w = 1.0;
    last_progress_time_ = this->get_clock()->now();

    publishOutputs();
  }

private:
  void validateRegions()
  {
    const size_t trigger_count = std::min(
      {trigger_x_mins_.size(), trigger_x_maxs_.size(), trigger_y_mins_.size(), trigger_y_maxs_.size()});
    const size_t tunnel_count = std::min(
      {tunnel_x_mins_.size(), tunnel_x_maxs_.size(), tunnel_y_mins_.size(), tunnel_y_maxs_.size()});
    const size_t entry_count = std::min({entry_xs_.size(), entry_ys_.size()});
    const size_t exit_count = std::min({exit_xs_.size(), exit_ys_.size()});

    const bool trigger_aligned =
      trigger_x_mins_.size() == trigger_x_maxs_.size() &&
      trigger_x_mins_.size() == trigger_y_mins_.size() &&
      trigger_x_mins_.size() == trigger_y_maxs_.size();
    const bool tunnel_aligned =
      tunnel_x_mins_.size() == tunnel_x_maxs_.size() &&
      tunnel_x_mins_.size() == tunnel_y_mins_.size() &&
      tunnel_x_mins_.size() == tunnel_y_maxs_.size();
    const bool entry_aligned = entry_xs_.size() == entry_ys_.size();
    const bool exit_aligned = exit_xs_.size() == exit_ys_.size();

    if (!trigger_aligned || !tunnel_aligned || !entry_aligned || !exit_aligned) {
      RCLCPP_FATAL(
        get_logger(),
        "Tunnel parameter arrays are not aligned. "
        "trigger=(%zu,%zu,%zu,%zu) tunnel=(%zu,%zu,%zu,%zu) entry=(%zu,%zu) exit=(%zu,%zu)",
        trigger_x_mins_.size(), trigger_x_maxs_.size(), trigger_y_mins_.size(), trigger_y_maxs_.size(),
        tunnel_x_mins_.size(), tunnel_x_maxs_.size(), tunnel_y_mins_.size(), tunnel_y_maxs_.size(),
        entry_xs_.size(), entry_ys_.size(), exit_xs_.size(), exit_ys_.size());
      throw std::runtime_error("Tunnel parameter arrays are not aligned");
    }

    tunnel_count_ = std::max(trigger_count, std::max(tunnel_count, std::max(entry_count, exit_count)));
    if (tunnel_count_ == 0) {
      RCLCPP_WARN(get_logger(), "No tunnel regions configured.");
      return;
    }

    if (trigger_count != tunnel_count_ || entry_count != tunnel_count_ || exit_count != tunnel_count_) {
      RCLCPP_FATAL(
        get_logger(),
        "Tunnel group counts are not aligned. trigger=%zu tunnel=%zu entry=%zu exit=%zu",
        trigger_count, tunnel_count, entry_count, exit_count);
      throw std::runtime_error("Tunnel group counts are not aligned");
    }

    auto resize_with_last = [this](std::vector<double> & values, size_t target_size, double fallback) {
      if (values.empty()) {
        values.resize(target_size, fallback);
        return;
      }
      values.resize(target_size, values.back());
    };

    resize_with_last(trigger_x_mins_, tunnel_count_, 0.0);
    resize_with_last(trigger_x_maxs_, tunnel_count_, 0.0);
    resize_with_last(trigger_y_mins_, tunnel_count_, 0.0);
    resize_with_last(trigger_y_maxs_, tunnel_count_, 0.0);

    resize_with_last(tunnel_x_mins_, tunnel_count_, 0.0);
    resize_with_last(tunnel_x_maxs_, tunnel_count_, 0.0);
    resize_with_last(tunnel_y_mins_, tunnel_count_, 0.0);
    resize_with_last(tunnel_y_maxs_, tunnel_count_, 0.0);
    resize_with_last(entry_xs_, tunnel_count_, 0.0);
    resize_with_last(entry_ys_, tunnel_count_, 0.0);
    resize_with_last(exit_xs_, tunnel_count_, 0.0);
    resize_with_last(exit_ys_, tunnel_count_, 0.0);

    for (size_t i = 0; i < tunnel_count_; ++i) {
      if (tunnel_x_mins_[i] == 0.0 && tunnel_x_maxs_[i] == 0.0 && tunnel_y_mins_[i] == 0.0 &&
          tunnel_y_maxs_[i] == 0.0)
      {
        tunnel_x_mins_[i] = trigger_x_mins_[i];
        tunnel_x_maxs_[i] = trigger_x_maxs_[i];
        tunnel_y_mins_[i] = trigger_y_mins_[i];
        tunnel_y_maxs_[i] = trigger_y_maxs_[i];
      }
    }
  }

  void applyCoordinateOffset()
  {
    if (map_origin_offset_x_ == 0.0 && map_origin_offset_y_ == 0.0) {
      return;
    }

    auto offset_x = [this](std::vector<double> & values) {
      for (auto & value : values) {
        value -= map_origin_offset_x_;
      }
    };
    auto offset_y = [this](std::vector<double> & values) {
      for (auto & value : values) {
        value -= map_origin_offset_y_;
      }
    };

    offset_x(trigger_x_mins_);
    offset_x(trigger_x_maxs_);
    offset_x(tunnel_x_mins_);
    offset_x(tunnel_x_maxs_);
    offset_x(entry_xs_);
    offset_x(exit_xs_);

    offset_y(trigger_y_mins_);
    offset_y(trigger_y_maxs_);
    offset_y(tunnel_y_mins_);
    offset_y(tunnel_y_maxs_);
    offset_y(entry_ys_);
    offset_y(exit_ys_);

    RCLCPP_INFO(
      get_logger(), "Applied tunnel coordinate offset: x=%.3f, y=%.3f",
      map_origin_offset_x_, map_origin_offset_y_);
  }

  RectRegion getTriggerRegion(size_t index) const
  {
    return RectRegion{
      trigger_x_mins_[index], trigger_x_maxs_[index], trigger_y_mins_[index], trigger_y_maxs_[index]};
  }

  RectRegion getTunnelRegion(size_t index) const
  {
    return RectRegion{
      tunnel_x_mins_[index], tunnel_x_maxs_[index], tunnel_y_mins_[index], tunnel_y_maxs_[index]};
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  bool pointInTrigger(size_t index, double x, double y, double margin = 0.0) const
  {
    return getTriggerRegion(index).contains(x, y, margin);
  }

  bool pointInTunnel(size_t index, double x, double y, double margin = 0.0) const
  {
    return getTunnelRegion(index).contains(x, y, margin);
  }

  bool isLowClearanceTunnel(int tunnel_id) const
  {
    if (tunnel_id < 0) {
      return false;
    }
    return std::find(
             low_clearance_tunnel_ids_.begin(),
             low_clearance_tunnel_ids_.end(),
             static_cast<int64_t>(tunnel_id)) != low_clearance_tunnel_ids_.end();
  }

  int findCurrentTunnel(double x, double y, double margin, bool use_tunnel_region) const
  {
    for (size_t i = 0; i < tunnel_count_; ++i) {
      const bool inside = use_tunnel_region ? pointInTunnel(i, x, y, margin) : pointInTrigger(i, x, y, margin);
      if (inside) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  int findTargetTunnelFromPlan() const
  {
    if (!has_plan_ || latest_plan_.poses.empty()) {
      return -1;
    }

    std::vector<int> points_in_region(tunnel_count_, 0);
    for (const auto & pose : latest_plan_.poses) {
      for (size_t tunnel_id = 0; tunnel_id < tunnel_count_; ++tunnel_id) {
        if (pointInTrigger(tunnel_id, pose.pose.position.x, pose.pose.position.y, path_check_margin_)) {
          ++points_in_region[tunnel_id];
          if (points_in_region[tunnel_id] >= min_path_points_in_region_) {
            return static_cast<int>(tunnel_id);
          }
        }
      }
    }

    return -1;
  }

  double getTunnelBaseYaw(int tunnel_id) const
  {
    const double dx = exit_xs_[tunnel_id] - entry_xs_[tunnel_id];
    const double dy = exit_ys_[tunnel_id] - entry_ys_[tunnel_id];
    return std::atan2(dy, dx);
  }

  double computeTargetYaw(int tunnel_id) const
  {
    const double forward_yaw = getTunnelBaseYaw(tunnel_id);
    bool approach_from_exit_side = false;
    if (approach_tunnel_id_ == tunnel_id) {
      approach_from_exit_side = approach_from_exit_side_;
    } else {
      const double dist_to_entry =
        std::hypot(robot_x_ - entry_xs_[tunnel_id], robot_y_ - entry_ys_[tunnel_id]);
      const double dist_to_exit =
        std::hypot(robot_x_ - exit_xs_[tunnel_id], robot_y_ - exit_ys_[tunnel_id]);
      approach_from_exit_side = dist_to_exit < dist_to_entry;
    }

    if (approach_from_exit_side) {
      return normalizeAngle(forward_yaw + M_PI);
    }
    return forward_yaw;
  }

  void startProgressTracking(bool in_tunnel_phase)
  {
    tracking_in_tunnel_phase_ = in_tunnel_phase;
    progress_reference_x_ = robot_x_;
    progress_reference_y_ = robot_y_;
    last_progress_time_ = this->get_clock()->now();
  }

  void rememberApproachSide(int tunnel_id)
  {
    if (!has_pose_ || tunnel_id < 0) {
      return;
    }

    const double dist_to_entry = std::hypot(robot_x_ - entry_xs_[tunnel_id], robot_y_ - entry_ys_[tunnel_id]);
    const double dist_to_exit = std::hypot(robot_x_ - exit_xs_[tunnel_id], robot_y_ - exit_ys_[tunnel_id]);
    approach_tunnel_id_ = tunnel_id;
    approach_from_exit_side_ = dist_to_exit < dist_to_entry;
  }

  void updateProgressTracking(bool in_tunnel_phase)
  {
    if (!has_pose_) {
      return;
    }

    if (tunnel_status_ != APPROACHING && tunnel_status_ != IN_TUNNEL) {
      return;
    }

    if (tracking_in_tunnel_phase_ != in_tunnel_phase) {
      startProgressTracking(in_tunnel_phase);
      return;
    }

    const double moved = std::hypot(robot_x_ - progress_reference_x_, robot_y_ - progress_reference_y_);
    if (moved >= min_progress_distance_) {
      progress_reference_x_ = robot_x_;
      progress_reference_y_ = robot_y_;
      last_progress_time_ = this->get_clock()->now();
    }
  }

  bool shouldTriggerRecovery(int active_tunnel_id)
  {
    if (!has_pose_ || active_tunnel_id < 0 || tunnel_recovery_active_) {
      return false;
    }

    if (tunnel_status_ != APPROACHING && tunnel_status_ != IN_TUNNEL) {
      return false;
    }

    const double timeout = tunnel_status_ == IN_TUNNEL ? tunnel_stuck_timeout_ : approach_stuck_timeout_;
    const auto elapsed = (this->get_clock()->now() - last_progress_time_).seconds();
    return elapsed >= timeout;
  }



    const double distance_to_goal = std::hypot(
      robot_x_ - tunnel_recovery_goal_.pose.position.x,
      robot_y_ - tunnel_recovery_goal_.pose.position.y);
    return distance_to_goal <= std::max(0.05, recovery_clear_margin_);
  }

  void refreshState()
  {
    if (!enable_tunnel_mode_ || tunnel_count_ == 0) {
      will_pass_tunnel_ = false;
      target_tunnel_id_ = -1;
      tracked_tunnel_id_ = -1;
      recovery_tunnel_id_ = -1;
      approach_tunnel_id_ = -1;
      in_tunnel_ = false;
      disable_spin_ = false;
      tunnel_recovery_active_ = false;
      tunnel_status_ = NORMAL;
      tunnel_target_yaw_ = current_map_yaw_;
      tunnel_yaw_error_ = 0.0;
      approach_from_exit_side_ = false;
      low_clearance_mode_ = false;
      return;
    }

    target_tunnel_id_ = findTargetTunnelFromPlan();
    will_pass_tunnel_ = target_tunnel_id_ >= 0;

    const int tunnel_id_from_pose = has_pose_ ? findCurrentTunnel(robot_x_, robot_y_, 0.0, true) : -1;
    in_tunnel_ = tunnel_id_from_pose >= 0;
    if (in_tunnel_) {
      tracked_tunnel_id_ = tunnel_id_from_pose;
    } else if (will_pass_tunnel_) {
      tracked_tunnel_id_ = target_tunnel_id_;
    } else {
      tracked_tunnel_id_ = -1;
    }

    const int active_tunnel_id = tracked_tunnel_id_;
    if (active_tunnel_id >= 0 && has_pose_) {
      tunnel_target_yaw_ = computeTargetYaw(active_tunnel_id);
      tunnel_yaw_error_ = normalizeAngle(tunnel_target_yaw_ - current_map_yaw_);
    } else {
      tunnel_target_yaw_ = current_map_yaw_;
      tunnel_yaw_error_ = 0.0;
    }

    const bool in_trigger = active_tunnel_id >= 0 && has_pose_ &&
      pointInTrigger(active_tunnel_id, robot_x_, robot_y_, activation_margin_);
    disable_spin_ = active_tunnel_id >= 0 && has_pose_ &&
      pointInTrigger(active_tunnel_id, robot_x_, robot_y_, disable_spin_margin_);

    if (active_tunnel_id >= 0 && has_pose_ && !tunnel_recovery_active_) {
      const bool entering_approach =
        !in_tunnel_ && in_trigger && will_pass_tunnel_ && tunnel_status_ != APPROACHING;
      const bool entering_tunnel = in_tunnel_ && tunnel_status_ != IN_TUNNEL;
      if (entering_approach || entering_tunnel) {
        rememberApproachSide(active_tunnel_id);
      }
    }

    if (!will_pass_tunnel_ && !in_tunnel_ && !tunnel_recovery_active_) {
      tunnel_recovery_active_ = false;
      recovery_tunnel_id_ = -1;
      approach_tunnel_id_ = -1;
      approach_from_exit_side_ = false;
      tunnel_status_ = NORMAL;
      low_clearance_mode_ = false;
      return;
    }

    if (tunnel_recovery_active_) {
      const bool path_changed_away_from_recovery_tunnel =
        target_tunnel_id_ < 0 || (recovery_tunnel_id_ >= 0 && target_tunnel_id_ != recovery_tunnel_id_);
      const bool no_longer_near_recovery_tunnel =
        !in_tunnel_ && !(recovery_tunnel_id_ >= 0 && has_pose_ &&
        pointInTrigger(static_cast<size_t>(recovery_tunnel_id_), robot_x_, robot_y_, activation_margin_));

      if (path_changed_away_from_recovery_tunnel && no_longer_near_recovery_tunnel) {
        RCLCPP_INFO(
          get_logger(),
          "Cancel tunnel recovery because current path no longer passes recovery tunnel (recovery=%d target=%d)",
          recovery_tunnel_id_, target_tunnel_id_);
        tunnel_recovery_active_ = false;
        recovery_tunnel_id_ = -1;
        approach_tunnel_id_ = -1;
        approach_from_exit_side_ = false;
        if (will_pass_tunnel_ && !in_tunnel_) {
          tunnel_status_ = in_trigger ? APPROACHING : WILL_PASS;
        } else {
          tunnel_status_ = NORMAL;
        }
      if (tunnel_status_ != IN_TUNNEL && tunnel_status_ != APPROACHING) {
        tunnel_recovery_active_ = false;
        recovery_tunnel_id_ = -1;
        approach_tunnel_id_ = -1;
        approach_from_exit_side_ = false;
        if (will_pass_tunnel_ && !in_tunnel_) {
          tunnel_status_ = WILL_PASS;
        } else if (!will_pass_tunnel_ && !in_tunnel_) {
          tunnel_status_ = PASSED;
        }
      } else {
        tunnel_status_ = RECOVERY_ACTIVE;
        low_clearance_mode_ = isLowClearanceTunnel(recovery_tunnel_id_);
        return;
      }
    }

    if (in_tunnel_) {
      if (tunnel_status_ != IN_TUNNEL) {
        startProgressTracking(true);
      }
      tunnel_status_ = IN_TUNNEL;
      updateProgressTracking(true);
    } else if (in_trigger && will_pass_tunnel_) {
      if (tunnel_status_ != APPROACHING) {
        startProgressTracking(false);
      }
      tunnel_status_ = APPROACHING;
      updateProgressTracking(false);
    } else if (will_pass_tunnel_) {
      tunnel_status_ = WILL_PASS;
    } else {
      tunnel_status_ = PASSED;
    }

    low_clearance_mode_ =
      isLowClearanceTunnel(active_tunnel_id) &&
      (tunnel_status_ == APPROACHING || tunnel_status_ == IN_TUNNEL ||
       tunnel_status_ == RECOVERY_ACTIVE);

    if (shouldTriggerRecovery(active_tunnel_id)) {
      tunnel_recovery_active_ = true;
      recovery_tunnel_id_ = active_tunnel_id;
      tunnel_recovery_goal_ = recovery_goal;  // BT handles the decision
      RCLCPP_INFO(get_logger(), "Tunnel stuck! Recovery active, BT decides next action");

#include <algorithm>
#include <array>
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
  RECOVERY_ACTIVE = 5,
  WAITING_FOR_REOPEN = 6
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

struct PolygonRegion
{
  struct Point
  {
    double x{};
    double y{};
  };

  std::array<Point, 4> points;

  bool contains(double x, double y, double margin = 0.0) const
  {
    bool inside = false;
    for (size_t current = 0, previous = points.size() - 1; current < points.size(); previous = current++) {
      const auto & a = points[current];
      const auto & b = points[previous];
      const bool crosses = ((a.y > y) != (b.y > y)) &&
        (x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x);
      if (crosses) {
        inside = !inside;
      }
    }
    if (inside || margin <= 0.0) {
      return inside;
    }

    const auto squared_distance_to_segment = [x, y](const Point & a, const Point & b) {
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double squared_length = dx * dx + dy * dy;
        if (squared_length <= std::numeric_limits<double>::epsilon()) {
          const double px = x - a.x;
          const double py = y - a.y;
          return px * px + py * py;
        }
        const double projection = std::clamp(
          ((x - a.x) * dx + (y - a.y) * dy) / squared_length, 0.0, 1.0);
        const double px = x - (a.x + projection * dx);
        const double py = y - (a.y + projection * dy);
        return px * px + py * py;
      };
    const double squared_margin = margin * margin;
    for (size_t current = 0, previous = points.size() - 1; current < points.size(); previous = current++) {
      if (squared_distance_to_segment(points[previous], points[current]) <= squared_margin) {
        return true;
      }
    }
    return false;
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
    blocked_tunnel_duration_(30.0),
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
    approach_progress_armed_(false),
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
    tunnel_waiting_for_reopen_topic_ = this->declare_parameter<std::string>(
      "tunnel_waiting_for_reopen_topic", "tunnel_waiting_for_reopen");
    tunnel_target_yaw_topic_ =
      this->declare_parameter<std::string>("tunnel_target_yaw_topic", "tunnel_target_yaw");
    current_map_yaw_topic_ =
      this->declare_parameter<std::string>("current_map_yaw_topic", "current_map_yaw");
    tunnel_yaw_error_topic_ =
      this->declare_parameter<std::string>("tunnel_yaw_error_topic", "tunnel_yaw_error");
    tunnel_markers_topic_ =
      this->declare_parameter<std::string>("tunnel_markers_topic", "tunnel_markers");
    blocked_tunnel_topic_ =
      this->declare_parameter<std::string>("blocked_tunnel_topic", "blocked_tunnel_id");
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
    approach_alignment_tolerance_ =
      this->declare_parameter<double>("approach_alignment_tolerance", 0.35);
    tunnel_stuck_timeout_ = this->declare_parameter<double>("tunnel_stuck_timeout", 1.5);
    min_progress_distance_ = this->declare_parameter<double>("min_progress_distance", 0.15);
    recovery_retreat_distance_ =
      this->declare_parameter<double>("recovery_retreat_distance", 0.8);
    recovery_clear_margin_ = this->declare_parameter<double>("recovery_clear_margin", 0.05);
    blocked_tunnel_duration_ = this->declare_parameter<double>("blocked_tunnel_duration", 30.0);
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
    corner_0_xs_ = this->declare_parameter<std::vector<double>>("corner_0_xs", empty_vec);
    corner_0_ys_ = this->declare_parameter<std::vector<double>>("corner_0_ys", empty_vec);
    corner_1_xs_ = this->declare_parameter<std::vector<double>>("corner_1_xs", empty_vec);
    corner_1_ys_ = this->declare_parameter<std::vector<double>>("corner_1_ys", empty_vec);
    corner_2_xs_ = this->declare_parameter<std::vector<double>>("corner_2_xs", empty_vec);
    corner_2_ys_ = this->declare_parameter<std::vector<double>>("corner_2_ys", empty_vec);
    corner_3_xs_ = this->declare_parameter<std::vector<double>>("corner_3_xs", empty_vec);
    corner_3_ys_ = this->declare_parameter<std::vector<double>>("corner_3_ys", empty_vec);

    validateRegions();
    applyCoordinateOffset();
    loadTunnelPolygons();

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
    blocked_tunnel_pub_ =
      this->create_publisher<std_msgs::msg::Int32>(blocked_tunnel_topic_, latched_qos);
    disable_spin_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(disable_spin_topic_, latched_qos);
    tunnel_recovery_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      tunnel_recovery_active_topic_, latched_qos);
    tunnel_recovery_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      tunnel_recovery_goal_topic_, latched_qos);
    tunnel_waiting_for_reopen_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      tunnel_waiting_for_reopen_topic_, latched_qos);
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

    const std::array<size_t, 8> polygon_lengths = {
      corner_0_xs_.size(), corner_0_ys_.size(), corner_1_xs_.size(), corner_1_ys_.size(),
      corner_2_xs_.size(), corner_2_ys_.size(), corner_3_xs_.size(), corner_3_ys_.size()};
    const bool polygon_configured = std::any_of(
      polygon_lengths.begin(), polygon_lengths.end(), [](size_t length) {return length != 0;});
    if (polygon_configured && std::any_of(
      polygon_lengths.begin(), polygon_lengths.end(), [this](size_t length) {
        return length != tunnel_count_;
      }))
    {
      RCLCPP_FATAL(
        get_logger(), "Tunnel polygon arrays must all contain %zu points, but got (%zu,%zu,%zu,%zu,%zu,%zu,%zu,%zu)",
        tunnel_count_, corner_0_xs_.size(), corner_0_ys_.size(), corner_1_xs_.size(), corner_1_ys_.size(),
        corner_2_xs_.size(), corner_2_ys_.size(), corner_3_xs_.size(), corner_3_ys_.size());
      throw std::runtime_error("Tunnel polygon parameter arrays are not aligned");
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
    offset_x(corner_0_xs_);
    offset_x(corner_1_xs_);
    offset_x(corner_2_xs_);
    offset_x(corner_3_xs_);

    offset_y(trigger_y_mins_);
    offset_y(trigger_y_maxs_);
    offset_y(tunnel_y_mins_);
    offset_y(tunnel_y_maxs_);
    offset_y(entry_ys_);
    offset_y(exit_ys_);
    offset_y(corner_0_ys_);
    offset_y(corner_1_ys_);
    offset_y(corner_2_ys_);
    offset_y(corner_3_ys_);

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

  void loadTunnelPolygons()
  {
    tunnel_polygons_.clear();
    if (corner_0_xs_.empty()) {
      return;
    }
    tunnel_polygons_.reserve(tunnel_count_);
    for (size_t index = 0; index < tunnel_count_; ++index) {
      tunnel_polygons_.push_back({{
        PolygonRegion::Point{corner_0_xs_[index], corner_0_ys_[index]},
        PolygonRegion::Point{corner_1_xs_[index], corner_1_ys_[index]},
        PolygonRegion::Point{corner_2_xs_[index], corner_2_ys_[index]},
        PolygonRegion::Point{corner_3_xs_[index], corner_3_ys_[index]}}});
    }
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
    if (index < tunnel_polygons_.size()) {
      return tunnel_polygons_[index].contains(x, y, margin);
    }
    return getTriggerRegion(index).contains(x, y, margin);
  }

  bool pointInTunnel(size_t index, double x, double y, double margin = 0.0) const
  {
    if (index < tunnel_polygons_.size()) {
      return tunnel_polygons_[index].contains(x, y, margin);
    }
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
    approach_progress_armed_ = in_tunnel_phase;
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

    if (!in_tunnel_phase && !approach_progress_armed_) {
      if (std::abs(tunnel_yaw_error_) > approach_alignment_tolerance_) {
        return;
      }
      // Do not count chassis alignment time as an entrance blockage. Start the
      // five-second progress timer only after the base faces the tunnel.
      approach_progress_armed_ = true;
      progress_reference_x_ = robot_x_;
      progress_reference_y_ = robot_y_;
      last_progress_time_ = this->get_clock()->now();
      RCLCPP_INFO(
        get_logger(), "Tunnel %d chassis aligned (yaw error %.2f rad); start entrance progress timer",
        tracked_tunnel_id_, tunnel_yaw_error_);
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
    if (tunnel_status_ == APPROACHING && !approach_progress_armed_) {
      return false;
    }

    const double timeout = tunnel_status_ == IN_TUNNEL ? tunnel_stuck_timeout_ : approach_stuck_timeout_;
    const auto elapsed = (this->get_clock()->now() - last_progress_time_).seconds();
    return elapsed >= timeout;
  }

  void clearExpiredTunnelBlock()
  {
    if (blocked_tunnel_id_ < 0 || std::chrono::steady_clock::now() < blocked_until_) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Tunnel %d block expired after %.1f seconds", blocked_tunnel_id_,
      blocked_tunnel_duration_);
    blocked_tunnel_id_ = -1;
  }

  void blockTunnel(int tunnel_id)
  {
    blocked_tunnel_id_ = tunnel_id;
    blocked_until_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(blocked_tunnel_duration_));
    RCLCPP_WARN(
      get_logger(), "Tunnel %d has no entrance progress for %.1f seconds; block it for %.1f seconds",
      tunnel_id, approach_stuck_timeout_, blocked_tunnel_duration_);
  }


  geometry_msgs::msg::PoseStamped buildRecoveryGoal(int tunnel_id)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = global_frame_;
    goal.header.stamp = this->get_clock()->now();

    const double forward_yaw = getTunnelBaseYaw(tunnel_id);
    const bool approach_from_exit_side =
      approach_tunnel_id_ == tunnel_id ? approach_from_exit_side_ : false;

    const double anchor_x = approach_from_exit_side ? exit_xs_[tunnel_id] : entry_xs_[tunnel_id];
    const double anchor_y = approach_from_exit_side ? exit_ys_[tunnel_id] : entry_ys_[tunnel_id];
    const double retreat_yaw = approach_from_exit_side ? forward_yaw : normalizeAngle(forward_yaw + M_PI);

    goal.pose.position.x = anchor_x + recovery_retreat_distance_ * std::cos(retreat_yaw);
    goal.pose.position.y = anchor_y + recovery_retreat_distance_ * std::sin(retreat_yaw);
    tf2::Quaternion retreat_quat;
    retreat_quat.setRPY(0.0, 0.0, retreat_yaw);
    goal.pose.orientation = tf2::toMsg(retreat_quat);
    return goal;
  }

  bool reachedRecoveryGoal() const
  {
    if (!has_pose_ || tunnel_recovery_goal_.header.frame_id.empty()) {
      return false;
    }

    const double distance_to_goal = std::hypot(
      robot_x_ - tunnel_recovery_goal_.pose.position.x,
      robot_y_ - tunnel_recovery_goal_.pose.position.y);
    return distance_to_goal <= std::max(0.05, recovery_clear_margin_);
  }

  void refreshState()
  {
    clearExpiredTunnelBlock();
    if (!enable_tunnel_mode_ || tunnel_count_ == 0) {
      will_pass_tunnel_ = false;
      target_tunnel_id_ = -1;
      tracked_tunnel_id_ = -1;
      recovery_tunnel_id_ = -1;
      approach_tunnel_id_ = -1;
      in_tunnel_ = false;
      disable_spin_ = false;
      tunnel_recovery_active_ = false;
      tunnel_waiting_for_reopen_ = false;
      tunnel_status_ = NORMAL;
      tunnel_target_yaw_ = current_map_yaw_;
      tunnel_yaw_error_ = 0.0;
      approach_from_exit_side_ = false;
      low_clearance_mode_ = false;
      return;
    }

    if (tunnel_waiting_for_reopen_) {
      if (blocked_tunnel_id_ >= 0) {
        tunnel_status_ = WAITING_FOR_REOPEN;
        low_clearance_mode_ = false;
        return;
      }
      tunnel_waiting_for_reopen_ = false;
      recovery_tunnel_id_ = -1;
      approach_tunnel_id_ = -1;
      approach_from_exit_side_ = false;
      RCLCPP_INFO(get_logger(), "Tunnel reopened; request a fresh path to the original goal");
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
      tunnel_waiting_for_reopen_ = false;
      recovery_tunnel_id_ = -1;
      approach_tunnel_id_ = -1;
      approach_from_exit_side_ = false;
      tunnel_status_ = NORMAL;
      low_clearance_mode_ = false;
      return;
    }

    if (tunnel_recovery_active_) {
      if (reachedRecoveryGoal()) {
        tunnel_recovery_active_ = false;
        tunnel_waiting_for_reopen_ = blocked_tunnel_id_ >= 0;
        if (tunnel_waiting_for_reopen_) {
          tunnel_status_ = WAITING_FOR_REOPEN;
          low_clearance_mode_ = false;
          RCLCPP_INFO(
            get_logger(), "Reached safe waiting point for blocked tunnel %d", recovery_tunnel_id_);
          return;
        }
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
      if (tunnel_status_ == APPROACHING) {
        blockTunnel(active_tunnel_id);
      }
      tunnel_recovery_active_ = true;
      tunnel_waiting_for_reopen_ = false;
      recovery_tunnel_id_ = active_tunnel_id;
      tunnel_recovery_goal_ = buildRecoveryGoal(active_tunnel_id);
      tunnel_status_ = RECOVERY_ACTIVE;
      low_clearance_mode_ = isLowClearanceTunnel(active_tunnel_id);
    }
  }

  void update()
  {
    refreshState();
    publishOutputs();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;
    odom_pose.header.frame_id = "odom";

    try {
      auto map_pose = tf_buffer_->transform(
        odom_pose, global_frame_, tf2::durationFromSec(transform_tolerance_));

      robot_x_ = map_pose.pose.position.x;
      robot_y_ = map_pose.pose.position.y;
      last_global_pose_ = map_pose;
      last_global_pose_.header.frame_id = global_frame_;
      current_map_yaw_ = tf2::getYaw(map_pose.pose.orientation);
      has_pose_ = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform odom pose from topic '%s' into '%s': %s",
        odom_topic_.c_str(), global_frame_.c_str(), ex.what());
      has_pose_ = false;
    }
  }

  void planCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_plan_ = *msg;
    has_plan_ = !latest_plan_.poses.empty();
    refreshState();
    publishOutputs();
  }

  visualization_msgs::msg::Marker makeRectMarker(
    int id, const std::string & ns, const RectRegion & rect, float r, float g, float b, float a)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = (rect.x_min + rect.x_max) * 0.5;
    marker.pose.position.y = (rect.y_min + rect.y_max) * 0.5;
    marker.pose.position.z = 0.05;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.05, rect.x_max - rect.x_min);
    marker.scale.y = std::max(0.05, rect.y_max - rect.y_min);
    marker.scale.z = 0.05;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
  }

  visualization_msgs::msg::Marker makeArrowMarker(
    int id, const std::string & ns, double x, double y, double yaw, float r, float g, float b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.15;
    tf2::Quaternion orientation_quat;
    orientation_quat.setRPY(0.0, 0.0, yaw);
    marker.pose.orientation = tf2::toMsg(orientation_quat);
    marker.scale.x = 0.5;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.9f;
    return marker;
  }

  visualization_msgs::msg::Marker makeSphereMarker(
    int id, const std::string & ns, double x, double y, float r, float g, float b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.12;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.9f;
    return marker;
  }

  void publishOutputs()
  {
    const auto stamp = this->get_clock()->now();
    const std::string active_controller_id =
      (tunnel_status_ == APPROACHING || tunnel_status_ == IN_TUNNEL ||
      tunnel_status_ == RECOVERY_ACTIVE) ? tunnel_controller_id_ : default_controller_id_;

    std_msgs::msg::String controller_msg;
    controller_msg.data = active_controller_id;
    controller_selector_pub_->publish(controller_msg);

    std_msgs::msg::Bool bool_msg;
    bool_msg.data = in_tunnel_;
    tunnel_state_pub_->publish(bool_msg);

    bool_msg.data = will_pass_tunnel_;
    will_pass_tunnel_pub_->publish(bool_msg);

    bool_msg.data = disable_spin_;
    disable_spin_pub_->publish(bool_msg);

    bool_msg.data = tunnel_recovery_active_;
    tunnel_recovery_active_pub_->publish(bool_msg);

    bool_msg.data = tunnel_waiting_for_reopen_;
    tunnel_waiting_for_reopen_pub_->publish(bool_msg);

    bool_msg.data = low_clearance_mode_;
    low_clearance_mode_pub_->publish(bool_msg);

    std_msgs::msg::Int32 int_msg;
    int_msg.data = target_tunnel_id_;
    target_tunnel_id_pub_->publish(int_msg);

    int_msg.data = tunnel_status_;
    tunnel_status_pub_->publish(int_msg);

    int_msg.data = blocked_tunnel_id_;
    blocked_tunnel_pub_->publish(int_msg);

    std_msgs::msg::Float64 float_msg;
    float_msg.data = tunnel_target_yaw_;
    tunnel_target_yaw_pub_->publish(float_msg);

    float_msg.data = current_map_yaw_;
    current_map_yaw_pub_->publish(float_msg);

    float_msg.data = tunnel_yaw_error_;
    tunnel_yaw_error_pub_->publish(float_msg);

    applyLowClearanceParameterSwitchIfNeeded();

    if (has_pose_) {
      last_global_pose_.header.stamp = stamp;
      global_pose_pub_->publish(last_global_pose_);

      geometry_msgs::msg::PointStamped point_msg;
      point_msg.header = last_global_pose_.header;
      point_msg.point = last_global_pose_.pose.position;
      global_point_pub_->publish(point_msg);
    }

    tunnel_recovery_goal_.header.stamp = stamp;
    tunnel_recovery_goal_pub_->publish(tunnel_recovery_goal_);

    sentry_decision_msg::msg::TunnelMonitor monitor_msg;
    monitor_msg.enable_tunnel_mode = enable_tunnel_mode_;
    monitor_msg.has_pose = has_pose_;
    monitor_msg.has_plan = has_plan_;
    monitor_msg.will_pass_tunnel = will_pass_tunnel_;
    monitor_msg.in_tunnel = in_tunnel_;
    monitor_msg.disable_spin = disable_spin_;
    monitor_msg.tunnel_recovery_active = tunnel_recovery_active_;
    monitor_msg.target_tunnel_id = target_tunnel_id_;
    monitor_msg.tracked_tunnel_id = tracked_tunnel_id_;
    monitor_msg.recovery_tunnel_id = recovery_tunnel_id_;
    monitor_msg.tunnel_status = tunnel_status_;
    monitor_msg.robot_x = robot_x_;
    monitor_msg.robot_y = robot_y_;
    monitor_msg.current_map_yaw = current_map_yaw_;
    monitor_msg.tunnel_target_yaw = tunnel_target_yaw_;
    monitor_msg.tunnel_yaw_error = tunnel_yaw_error_;
    monitor_msg.controller_id = active_controller_id;
    monitor_msg.global_pose = last_global_pose_;
    monitor_msg.tunnel_recovery_goal = tunnel_recovery_goal_;
    tunnel_monitor_pub_->publish(monitor_msg);

    publishMarkers();
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    for (size_t i = 0; i < tunnel_count_; ++i) {
      const bool is_target = static_cast<int>(i) == tracked_tunnel_id_;
      const auto trigger = getTriggerRegion(i);
      const auto tunnel = getTunnelRegion(i);
      markers.markers.push_back(makeRectMarker(
        marker_id++, "trigger", trigger, is_target ? 1.0f : 0.2f, 0.8f, 0.2f, 0.18f));
      markers.markers.push_back(makeRectMarker(
        marker_id++, "tunnel", tunnel, is_target ? 1.0f : 0.1f, 0.3f, 1.0f, 0.30f));
      markers.markers.push_back(makeSphereMarker(
        marker_id++, "entry", entry_xs_[i], entry_ys_[i], 0.1f, 0.9f, 0.1f));
      markers.markers.push_back(makeSphereMarker(
        marker_id++, "exit", exit_xs_[i], exit_ys_[i], 0.9f, 0.2f, 0.2f));
    }

    if (tracked_tunnel_id_ >= 0) {
      markers.markers.push_back(makeArrowMarker(
        marker_id++, "target_yaw", robot_x_, robot_y_, tunnel_target_yaw_, 1.0f, 0.7f, 0.0f));
    }

    if (tunnel_recovery_active_) {
      markers.markers.push_back(makeSphereMarker(
        marker_id++, "recovery_goal", tunnel_recovery_goal_.pose.position.x,
        tunnel_recovery_goal_.pose.position.y, 1.0f, 0.0f, 1.0f));
    }

    tunnel_markers_pub_->publish(markers);
  }

  void applyLowClearanceParameterSwitchIfNeeded()
  {
    if (!enable_low_clearance_param_switch_) {
      return;
    }

    if (low_clearance_param_clients_.empty()) {
      return;
    }

    if (last_vehicle_height_mode_valid_ &&
      last_vehicle_height_mode_low_clearance_ == low_clearance_mode_)
    {
      return;
    }

    if (parameter_switch_inflight_) {
      return;
    }

    const double target_vehicle_height =
      low_clearance_mode_ ? low_clearance_vehicle_height_ : normal_vehicle_height_;

    for (size_t i = 0; i < low_clearance_param_clients_.size(); ++i) {
      const auto & client = low_clearance_param_clients_[i];
      if (!client || !client->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "low_clearance param service not ready for node '%s'",
          low_clearance_target_nodes_resolved_[i].c_str());
        return;
      }
    }

    parameter_switch_inflight_ = true;
    pending_request_id_++;
    pending_mode_low_clearance_ = low_clearance_mode_;
    pending_response_count_ = 0;
    pending_success_count_ = 0;

    for (size_t i = 0; i < low_clearance_param_clients_.size(); ++i) {
      const auto node_name = low_clearance_target_nodes_resolved_[i];
      const auto request_id = pending_request_id_;
      auto callback =
        [this, node_name, target_vehicle_height, request_id](
          std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
          if (request_id != pending_request_id_) {
            return;
          }

          const auto results = future.get();
          bool ok = true;
          for (const auto & result : results) {
            if (!result.successful) {
              ok = false;
              RCLCPP_WARN(
                get_logger(),
                "Failed to set vehicleHeight on '%s': %s",
                node_name.c_str(),
                result.reason.c_str());
            }
          }
          if (ok) {
            RCLCPP_INFO(
              get_logger(),
              "Set '%s' vehicleHeight to %.3f",
              node_name.c_str(),
              target_vehicle_height);
          }

          pending_response_count_++;
          if (ok) {
            pending_success_count_++;
          }

          if (pending_response_count_ == low_clearance_param_clients_.size()) {
            if (pending_success_count_ == low_clearance_param_clients_.size()) {
              last_vehicle_height_mode_valid_ = true;
              last_vehicle_height_mode_low_clearance_ = pending_mode_low_clearance_;
            } else {
              RCLCPP_WARN(
                get_logger(),
                "low_clearance parameter switch incomplete (%zu/%zu), will retry",
                pending_success_count_,
                low_clearance_param_clients_.size());
            }
            parameter_switch_inflight_ = false;
          }
        };

      low_clearance_param_clients_[i]->set_parameters(
        {rclcpp::Parameter("vehicleHeight", target_vehicle_height)},
        callback);
    }
  }

  std::string resolveTargetNodeName(const std::string & node_name) const
  {
    if (node_name.empty()) {
      return node_name;
    }

    if (node_name.front() == '/') {
      return node_name;
    }

    const std::string ns = this->get_namespace();
    if (ns.empty() || ns == "/") {
      return "/" + node_name;
    }

    return ns + "/" + node_name;
  }

  std::string global_frame_;
  std::string odom_topic_;
  std::string controller_selector_topic_;
  std::string tunnel_state_topic_;
  std::string global_pose_topic_;
  std::string global_point_topic_;
  std::string global_plan_topic_;
  std::string will_pass_tunnel_topic_;
  std::string target_tunnel_id_topic_;
  std::string tunnel_status_topic_;
  std::string disable_spin_topic_;
  std::string tunnel_recovery_active_topic_;
  std::string tunnel_recovery_goal_topic_;
  std::string tunnel_waiting_for_reopen_topic_;
  std::string tunnel_target_yaw_topic_;
  std::string current_map_yaw_topic_;
  std::string tunnel_yaw_error_topic_;
  std::string tunnel_markers_topic_;
  std::string blocked_tunnel_topic_;
  std::string tunnel_monitor_topic_;
  std::string low_clearance_mode_topic_;

  bool enable_tunnel_mode_;
  bool in_tunnel_;
  bool has_pose_;
  bool has_plan_;
  bool will_pass_tunnel_;
  bool disable_spin_;
  bool tunnel_recovery_active_;
  bool low_clearance_mode_;

  std::string default_controller_id_;
  std::string tunnel_controller_id_;

  double activation_margin_;
  double path_check_margin_;
  double disable_spin_margin_;
  double publish_rate_;
  double transform_tolerance_;
  double pose_hold_timeout_;
  double map_origin_offset_x_;
  double map_origin_offset_y_;
  double approach_stuck_timeout_;
  double approach_alignment_tolerance_;
  double tunnel_stuck_timeout_;
  double min_progress_distance_;
  double recovery_retreat_distance_;
  double recovery_clear_margin_;
  double blocked_tunnel_duration_;

  int min_path_points_in_region_;
  int target_tunnel_id_;
  int tracked_tunnel_id_;
  int recovery_tunnel_id_;
  int approach_tunnel_id_;
  int tunnel_status_;
  int blocked_tunnel_id_{-1};

  size_t tunnel_count_;

  double robot_x_;
  double robot_y_;
  double current_map_yaw_;
  double tunnel_target_yaw_;
  double tunnel_yaw_error_;
  double progress_reference_x_;
  double progress_reference_y_;
  bool tracking_in_tunnel_phase_;
  bool approach_progress_armed_;
  bool approach_from_exit_side_;
  bool tunnel_waiting_for_reopen_{false};
  std::chrono::steady_clock::time_point blocked_until_{};

  std::vector<double> trigger_x_mins_;
  std::vector<double> trigger_x_maxs_;
  std::vector<double> trigger_y_mins_;
  std::vector<double> trigger_y_maxs_;
  std::vector<double> tunnel_x_mins_;
  std::vector<double> tunnel_x_maxs_;
  std::vector<double> tunnel_y_mins_;
  std::vector<double> tunnel_y_maxs_;
  std::vector<double> entry_xs_;
  std::vector<double> entry_ys_;
  std::vector<double> exit_xs_;
  std::vector<double> exit_ys_;
  std::vector<double> corner_0_xs_;
  std::vector<double> corner_0_ys_;
  std::vector<double> corner_1_xs_;
  std::vector<double> corner_1_ys_;
  std::vector<double> corner_2_xs_;
  std::vector<double> corner_2_ys_;
  std::vector<double> corner_3_xs_;
  std::vector<double> corner_3_ys_;
  std::vector<PolygonRegion> tunnel_polygons_;
  std::vector<int64_t> low_clearance_tunnel_ids_;
  std::vector<std::string> low_clearance_target_nodes_;
  std::vector<std::string> low_clearance_target_nodes_resolved_;

  geometry_msgs::msg::PoseStamped last_global_pose_;
  geometry_msgs::msg::PoseStamped tunnel_recovery_goal_;
  nav_msgs::msg::Path latest_plan_;
  rclcpp::Time last_progress_time_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_selector_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tunnel_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr will_pass_tunnel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr disable_spin_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tunnel_recovery_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tunnel_waiting_for_reopen_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tunnel_recovery_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr global_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_tunnel_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tunnel_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr blocked_tunnel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tunnel_target_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_map_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tunnel_yaw_error_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tunnel_markers_pub_;
  rclcpp::Publisher<sentry_decision_msg::msg::TunnelMonitor>::SharedPtr tunnel_monitor_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr low_clearance_mode_pub_;
  std::vector<std::shared_ptr<rclcpp::AsyncParametersClient>> low_clearance_param_clients_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  bool enable_low_clearance_param_switch_{false};
  double low_clearance_vehicle_height_{0.20};
  double normal_vehicle_height_{0.23};
  bool last_vehicle_height_mode_valid_{false};
  bool last_vehicle_height_mode_low_clearance_{false};
  bool parameter_switch_inflight_{false};
  size_t pending_request_id_{0};
  size_t pending_response_count_{0};
  size_t pending_success_count_{0};
  bool pending_mode_low_clearance_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TunnelRegionMonitor>());
  rclcpp::shutdown();
  return 0;
}

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <Eigen/Dense>
#include <Eigen/LDLT>
#include "algo_master/msg/wheel_raw.hpp"
#include <string>
#include <memory>

namespace wheel_observer {

class WheelObserverNode : public rclcpp::Node {
public:
  explicit WheelObserverNode(const rclcpp::NodeOptions &options);

private:
  void wheelRawCallback(const algo_master::msg::WheelRaw::SharedPtr msg);

  rclcpp::Subscription<algo_master::msg::WheelRaw>::SharedPtr wheel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rms_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  bool enable_yaw_correction_;
  double wheel_counts_per_m_{10000.0};
  double wheel_dist_{0.3};

  Eigen::Matrix<double, 3, 4> K_;  // inverse kinematics matrix
};

}  // namespace wheel_observer

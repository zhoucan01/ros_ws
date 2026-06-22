#include "wheel_observer/wheel_observer.hpp"
#include "algo_master/msg/wheel_raw.hpp"
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <cmath>

namespace wheel_observer {

WheelObserverNode::WheelObserverNode(const rclcpp::NodeOptions &options)
  : Node("wheel_observer", options) {
  input_topic_ = declare_parameter<std::string>("input_topic", "/wheel_raw");
  output_topic_ = declare_parameter<std::string>("output_topic", "/wheel_odom_transformed");
  output_frame_id_ = declare_parameter<std::string>("output_frame_id", "base_link");
  enable_yaw_correction_ = declare_parameter<bool>("enable_yaw_correction", true);

  RCLCPP_INFO(get_logger(),
    "wheel_observer started: %s -> %s",
    input_topic_.c_str(), output_topic_.c_str());

  wheel_sub_ = create_subscription<algo_master::msg::WheelRaw>(
    input_topic_, 10, std::bind(&WheelObserverNode::wheelRawCallback, this, std::placeholders::_1));
  wheel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
  rms_pub_ = create_publisher<std_msgs::msg::Float64>("/wheel_rms", 10);
}

void WheelObserverNode::wheelRawCallback(const algo_master::msg::WheelRaw::SharedPtr msg) {
  std_msgs::msg::Float64 rms_msg;
  rms_msg.data = static_cast<double>(msg->wheel_rms);
  rms_pub_->publish(rms_msg);

  auto out = std::make_shared<geometry_msgs::msg::TwistStamped>();
  out->header.stamp = now();
  out->header.frame_id = output_frame_id_;

  double yaw = enable_yaw_correction_ ? static_cast<double>(msg->gimbal_yaw) : 0.0;
  const double vx = static_cast<double>(msg->vx_wheel) / 10000.0;
  const double vy = static_cast<double>(msg->vy_wheel) / 10000.0;
  if (enable_yaw_correction_) {
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    out->twist.linear.x =  vx * c + vy * s;
    out->twist.linear.y = -vx * s + vy * c;
  } else {
    out->twist.linear.x = vx;
    out->twist.linear.y = vy;
  }
  out->twist.linear.z = 0.0;
  out->twist.angular.x = 0.0;
  out->twist.angular.y = 0.0;
  out->twist.angular.z = 0.0;

  wheel_pub_->publish(*out);
}

}  // namespace wheel_observer
RCLCPP_COMPONENTS_REGISTER_NODE(wheel_observer::WheelObserverNode)

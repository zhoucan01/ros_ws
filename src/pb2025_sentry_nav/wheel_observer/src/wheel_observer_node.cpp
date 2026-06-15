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

  // Kinematic matrix: pinv(H) mapping [vlf,vlb,vrb,vrf] -> [vx,vy,omega_z]
  // Default: standard 4-omni, for mecanum wheels you need to override this.
  std::vector<double> default_pinv = {
    // output: vx has small contribution from each wheel
     1.0,  1.0,  1.0,  1.0,   // row 0: vx coefficient (0 if 4-omni balanced)
    -1.0,  1.0, -1.0,  1.0,   // row 1: vy coefficient
    -1.0, -1.0,  1.0,  1.0    // row 2: omega coefficient (scaled by wheel_dist)
  };
  auto user_pinv = declare_parameter<std::vector<double>>("inverse_kinematics", default_pinv);
  // Build Eigen matrix from param
  for (int i = 0; i < 3 && i * 4 < (int)user_pinv.size(); i++) {
    for (int j = 0; j < 4; j++) {
      K_(i, j) = user_pinv[i * 4 + j];
    }
  }

  // Scale factor: how to convert wheel encoder counts to m/s
  // wheel_speed_mps = wheel_counts / wheel_counts_per_m
  wheel_counts_per_m_ = declare_parameter<double>("wheel_counts_per_m", 10000.0);
  wheel_dist_ = declare_parameter<double>("wheel_dist", 0.3);  // Lx+Ly (chassis half-length + half-width)

  // If omega is zero in pinv, compute it:
  if (K_.row(2).norm() < 1e-9) {
    // auto-generate omega row: typical 4-omni formula
    K_(2, 0) = -1.0 / (4.0 * wheel_dist_);
    K_(2, 1) = -1.0 / (4.0 * wheel_dist_);
    K_(2, 2) =  1.0 / (4.0 * wheel_dist_);
    K_(2, 3) =  1.0 / (4.0 * wheel_dist_);
  }

  RCLCPP_INFO(get_logger(),
    "wheel_observer started: %s -> %s  |  K = [[%+.3f %+.3f %+.3f %+.3f]\n"
    "                                         [%+.3f %+.3f %+.3f %+.3f]\n"
    "                                         [%+.3f %+.3f %+.3f %+.3f]]",
    input_topic_.c_str(), output_topic_.c_str(),
    K_(0,0), K_(0,1), K_(0,2), K_(0,3),
    K_(1,0), K_(1,1), K_(1,2), K_(1,3),
    K_(2,0), K_(2,1), K_(2,2), K_(2,3));

  wheel_sub_ = create_subscription<algo_master::msg::WheelRaw>(
    input_topic_, 10, std::bind(&WheelObserverNode::wheelRawCallback, this, std::placeholders::_1));
  wheel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
  rms_pub_ = create_publisher<std_msgs::msg::Float64>("/wheel_rms", 10);
}

void WheelObserverNode::wheelRawCallback(const algo_master::msg::WheelRaw::SharedPtr msg) {
  // Convert raw wheel counts to m/s
  Eigen::Vector4d w;
  w << static_cast<double>(msg->vlf),
       static_cast<double>(msg->vlb),
       static_cast<double>(msg->vrb),
       static_cast<double>(msg->vrf);
  w /= wheel_counts_per_m_;

  // Forward kinematics: [vx; vy; omega] = K * w
  Eigen::Vector3d chassis_vel = K_ * w;

  // Predict wheel speeds from chassis velocity (for RMS computation)
  Eigen::Matrix<double, 4, 3> H = (K_.transpose() * (K_ * K_.transpose()).inverse()).transpose();
  // Actually just use pseudoinverse: H = K^T * (K*K^T)^{-1}
  // But simpler: if K is 3x4, H = K^T * inv(K*K^T)
  Eigen::Matrix<double, 4, 3> H_pinv;  // this is the transpose relationship
  // K is 3x4, H should be 4x3: w_pred = H * [vx; vy; omega]
  // Using standard omni kinematics: H_i = -sin(theta_i)... etc
  // For now, just compute RMS as the residual of the overdetermined system
  // Since K is 3x4, K * w fits [vx vy omega] in LS sense.
  // RMS = ||w - K^T * (K*K^T)^{-1} * K * w||^2  (projection residual)
  Eigen::Vector4d w_pred = K_.transpose() * (K_ * K_.transpose()).ldlt().solve(K_ * w);
  double rms = (w - w_pred).squaredNorm();

  // Publish RMS
  std_msgs::msg::Float64 rms_msg;
  rms_msg.data = rms;
  rms_pub_->publish(rms_msg);

  // Rotate chassis velocity by gimbal_yaw to get IMU-frame velocity
  auto out = std::make_shared<geometry_msgs::msg::TwistStamped>();
  out->header.stamp = now();
  out->header.frame_id = output_frame_id_;

  double yaw = enable_yaw_correction_ ? static_cast<double>(msg->gimbal_yaw) : 0.0;
  if (enable_yaw_correction_) {
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    out->twist.linear.x =  chassis_vel(0) * c + chassis_vel(1) * s;
    out->twist.linear.y = -chassis_vel(0) * s + chassis_vel(1) * c;
  } else {
    out->twist.linear.x = chassis_vel(0);
    out->twist.linear.y = chassis_vel(1);
  }
  out->twist.linear.z = 0.0;
  out->twist.angular.x = 0.0;
  out->twist.angular.y = 0.0;
  out->twist.angular.z = chassis_vel(2);  // omega from kinematics (may be noisy)

  wheel_pub_->publish(*out);
}

}  // namespace wheel_observer
RCLCPP_COMPONENTS_REGISTER_NODE(wheel_observer::WheelObserverNode)

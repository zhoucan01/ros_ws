#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class TfPosePublisher : public rclcpp::Node
{
public:
  TfPosePublisher()
  : Node("tf_pose_publisher")
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "gimbal_yaw_fake");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "state_estimate_pose");
    point_topic_ = declare_parameter<std::string>("point_topic", "state_estimate_point");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "initialpose");
    publish_map_to_odom_ = declare_parameter<bool>("publish_map_to_odom", true);
    const double publish_rate = declare_parameter<double>("publish_rate", 20.0);

    map_to_odom_.setIdentity();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(point_topic_, 10);

    if (publish_map_to_odom_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
      initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic_, 10,
        std::bind(&TfPosePublisher::initialPoseCallback, this, std::placeholders::_1));
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(publish_rate, 1.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TfPosePublisher::publishPose, this));
  }

private:
  static geometry_msgs::msg::Pose poseFromTransform(const tf2::Transform & transform)
  {
    geometry_msgs::msg::Pose pose;
    const auto & origin = transform.getOrigin();
    pose.position.x = origin.x();
    pose.position.y = origin.y();
    pose.position.z = origin.z();
    pose.orientation = tf2::toMsg(transform.getRotation());
    return pose;
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != global_frame_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring initial pose in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), global_frame_.c_str());
      return;
    }

    requested_global_to_robot_ = tf2::Transform();
    tf2::fromMsg(message->pose.pose, *requested_global_to_robot_);
    applyRequestedInitialPose();
  }

  void applyRequestedInitialPose()
  {
    if (!requested_global_to_robot_.has_value()) {
      return;
    }

    try {
      // T_map_odom = T_map_robot * T_robot_odom.
      const auto robot_to_odom = tf_buffer_->lookupTransform(
        robot_frame_, odom_frame_, tf2::TimePointZero);
      tf2::Transform robot_to_odom_transform;
      tf2::fromMsg(robot_to_odom.transform, robot_to_odom_transform);
      map_to_odom_ = *requested_global_to_robot_ * robot_to_odom_transform;
      requested_global_to_robot_.reset();
      RCLCPP_INFO(
        get_logger(), "Applied RViz initial pose and updated TF %s -> %s",
        global_frame_.c_str(), odom_frame_.c_str());
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for TF %s -> %s to apply initial pose: %s",
        robot_frame_.c_str(), odom_frame_.c_str(), exception.what());
    }
  }

  void publishPose()
  {
    applyRequestedInitialPose();

    try {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = now();

      if (publish_map_to_odom_) {
        const auto odom_to_robot = tf_buffer_->lookupTransform(
          odom_frame_, robot_frame_, tf2::TimePointZero);
        tf2::Transform odom_to_robot_transform;
        tf2::fromMsg(odom_to_robot.transform, odom_to_robot_transform);
        pose.pose = poseFromTransform(map_to_odom_ * odom_to_robot_transform);

        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header = pose.header;
        map_to_odom.child_frame_id = odom_frame_;
        map_to_odom.transform = tf2::toMsg(map_to_odom_);
        tf_broadcaster_->sendTransform(map_to_odom);
      } else {
        const auto global_to_robot = tf_buffer_->lookupTransform(
          global_frame_, robot_frame_, tf2::TimePointZero);
        pose.header.stamp = global_to_robot.header.stamp;
        pose.pose.position.x = global_to_robot.transform.translation.x;
        pose.pose.position.y = global_to_robot.transform.translation.y;
        pose.pose.position.z = global_to_robot.transform.translation.z;
        pose.pose.orientation = global_to_robot.transform.rotation;
      }

      pose_pub_->publish(pose);
      geometry_msgs::msg::PointStamped point;
      point.header = pose.header;
      point.point = pose.pose.position;
      point_pub_->publish(point);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for state-estimate TF: %s", exception.what());
    }
  }

  std::string global_frame_;
  std::string odom_frame_;
  std::string robot_frame_;
  std::string pose_topic_;
  std::string point_topic_;
  std::string initial_pose_topic_;
  bool publish_map_to_odom_;
  tf2::Transform map_to_odom_;
  std::optional<tf2::Transform> requested_global_to_robot_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfPosePublisher>());
  rclcpp::shutdown();
  return 0;
}

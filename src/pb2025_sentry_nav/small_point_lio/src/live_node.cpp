#include <small_point_lio/pch.h>
#include <small_point_lio/small_point_lio.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("small_point_lio_node");

    std::string config_path;
    node->declare_parameter("config_path", "");
    node->get_parameter("config_path", config_path);
    auto config = YAML::LoadFile(config_path);

    small_point_lio::SmallPointLio lio(config["small_point_lio"]);
    common::Odometry current_odometry;
    std::mutex odom_mutex;

    // Odometry publisher & TF
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    lio.set_odometry_callback([&](const common::Odometry &odom) {
        std::lock_guard<std::mutex> lock(odom_mutex);
        current_odometry = odom;

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(odom.timestamp * 1e9));
        msg.header.frame_id = "camera_init";
        msg.child_frame_id = "body";
        msg.pose.pose.position.x = odom.position.x();
        msg.pose.pose.position.y = odom.position.y();
        msg.pose.pose.position.z = odom.position.z();
        msg.pose.pose.orientation.x = odom.orientation.x();
        msg.pose.pose.orientation.y = odom.orientation.y();
        msg.pose.pose.orientation.z = odom.orientation.z();
        msg.pose.pose.orientation.w = odom.orientation.w();
        msg.twist.twist.linear.x = odom.velocity.x();
        msg.twist.twist.linear.y = odom.velocity.y();
        msg.twist.twist.linear.z = odom.velocity.z();
        msg.twist.twist.angular.x = odom.angular_velocity.x();
        msg.twist.twist.angular.y = odom.angular_velocity.y();
        msg.twist.twist.angular.z = odom.angular_velocity.z();
        odom_pub->publish(msg);

        geometry_msgs::msg::TransformStamped tf;
        tf.header = msg.header;
        tf.child_frame_id = "body";
        tf.transform.translation.x = odom.position.x();
        tf.transform.translation.y = odom.position.y();
        tf.transform.translation.z = odom.position.z();
        tf.transform.rotation = msg.pose.pose.orientation;
        tf_broadcaster->sendTransform(tf);
    });

    // IMU subscription
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/livox/imu", 500,
        [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
            common::ImuMsg imu;
            imu.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            imu.linear_acceleration << msg->linear_acceleration.x,
                msg->linear_acceleration.y, msg->linear_acceleration.z;
            imu.angular_velocity << msg->angular_velocity.x,
                msg->angular_velocity.y, msg->angular_velocity.z;
            lio.on_imu_callback(imu);
        });

    // LiDAR subscription
    auto lidar_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            std::vector<common::Point> points;
            sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
            for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z) {
                if (!std::isfinite(*iter_x)) continue;
                common::Point p;
                p.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                p.position << *iter_x, *iter_y, *iter_z;
                points.push_back(p);
            }
            lio.on_point_cloud_callback(points);
        });

    // Wheel odometry subscription
    auto wheel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "/wheel_odom", 100,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            lio.on_wheel_callback(msg->linear.x, msg->linear.y);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

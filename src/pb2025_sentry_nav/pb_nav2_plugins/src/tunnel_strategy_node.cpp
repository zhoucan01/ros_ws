#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sentry_decision_msg/msg/tunnel_monitor.hpp>
#include <cmath>

class TunnelStrategyNode : public rclcpp::Node {
public:
  TunnelStrategyNode() : Node("tunnel_strategy_node") {
    home_wait_x_ = declare_parameter("home_wait_x", 0.0);
    home_wait_y_ = declare_parameter("home_wait_y", 0.0);
    central_wait_x_ = declare_parameter("central_wait_x", 6.0);
    central_wait_y_ = declare_parameter("central_wait_y", 5.0);
    enemy_wait_x_ = declare_parameter("enemy_wait_x", 12.0);
    enemy_wait_y_ = declare_parameter("enemy_wait_y", 5.0);
    strategic_goal_ = declare_parameter("strategic_goal", 1);
    retry_cooldown_ = declare_parameter("retry_cooldown", 5.0);
    max_retries_ = declare_parameter("max_retries", 3);

    sub_ = create_subscription<sentry_decision_msg::msg::TunnelMonitor>(
      "tunnel_monitor", 10, std::bind(&TunnelStrategyNode::on_tunnel, this, std::placeholders::_1));
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("tunnel_recovery_goal", 10);

    retry_time_ = 0; retry_count_ = 0; failed_tunnel_id_ = -1;
  }

private:
  void on_tunnel(const sentry_decision_msg::msg::TunnelMonitor::SharedPtr msg) {
    strategic_goal_ = get_parameter("strategic_goal").as_int();
    bool active = msg->tunnel_recovery_active;
    int sid = msg->target_tunnel_id;
    double rx = msg->robot_x, ry = msg->robot_y;

    // 不在恢复中 → 可能到了等待位，启动倒计时
    if (!active) {
      if (at_wait_pos(rx, ry) && failed_tunnel_id_ >= 0) {
        retry_time_ = now().seconds() + retry_cooldown_;
        RCLCPP_INFO(get_logger(), "at wait pos, retry in %.1fs (attempt %d/%d)",
          retry_cooldown_, retry_count_ + 1, max_retries_);
      }
      return;
    }

    // ===== 正在恢复中 =====
    failed_tunnel_id_ = sid;
    geometry_msgs::msg::PoseStamped g;
    g.header.frame_id = "map"; g.pose.orientation.w = 1.0;

    // 冷却已过，并且还没放弃 → 放行 retry
    if (retry_time_ > 0 && now().seconds() >= retry_time_) {
      retry_time_ = 0; retry_count_++;
      if (retry_count_ <= max_retries_) {
        RCLCPP_INFO(get_logger(), "RETRY %d/%d tunnel %d ...", retry_count_, max_retries_, sid);
        return;  // 不干涉，让 BT 原 goal 生效
      }
    }

    // 还在冷却 或 已放弃 → 送等待位
    if (strategic_goal_ == 0) {
      g.pose.position.x = central_wait_x_; g.pose.position.y = central_wait_y_;
    } else if (strategic_goal_ == 2) {
      g.pose.position.x = central_wait_x_; g.pose.position.y = central_wait_y_;
    } else {
      g.pose.position.x = central_wait_x_; g.pose.position.y = central_wait_y_;
    }
    pub_->publish(g);

    double remain = std::max(0.0, retry_time_ - now().seconds());
    RCLCPP_INFO(get_logger(), "stuck t%d -> wait pos (retry in %.0fs, attempt %d/%d)",
      sid, remain, retry_count_ + 1, max_retries_);
  }

  bool at_wait_pos(double rx, double ry) {
    return std::hypot(rx - central_wait_x_, ry - central_wait_y_) < 1.5;
  }

  double home_wait_x_, home_wait_y_, central_wait_x_, central_wait_y_, enemy_wait_x_, enemy_wait_y_;
  double retry_cooldown_, retry_time_;
  int strategic_goal_, failed_tunnel_id_, retry_count_, max_retries_;
  rclcpp::Subscription<sentry_decision_msg::msg::TunnelMonitor>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TunnelStrategyNode>());
  rclcpp::shutdown();
  return 0;
}

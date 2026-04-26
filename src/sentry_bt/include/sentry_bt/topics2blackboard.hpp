
#ifndef SENTRY_BT_TOPICS2BLACKBOARD_HPP
#define SENTRY_BT_TOPICS2BLACKBOARD_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/blackboard.h>
#include <sentry_decision_msg/msg/sentry_decision.hpp>
// #include <sentry_decision_msg/msg/enemy_pos.hpp>
#include <sentry_decision_msg/msg/enemy_pos.hpp>
#include <sentry_decision_msg/msg/referee_raw.hpp>
#include <sentry_decision_msg/msg/manual_pos.hpp>
#include <sentry_decision_msg/msg/tunnel_monitor.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include "algo_master/msg/plc2_target.hpp"

class BlackboardUpdater : public rclcpp::Node
{
public:
    explicit BlackboardUpdater(BT::Blackboard::Ptr blackboard);

private:
    BT::Blackboard::Ptr blackboard_;
    rclcpp::Subscription<sentry_decision_msg::msg::SentryDecision>::SharedPtr sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::RefereeRaw>::SharedPtr referee_raw_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::EnemyPos>::SharedPtr enemypos_sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::TunnelMonitor>::SharedPtr tunnel_monitor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr current_pos_sub_;
    rclcpp::Subscription<algo_master::msg::PLC2Target>::SharedPtr target_sub_;
    sentry_decision_msg::msg::SentryDecision sentry_decision_msg_;
    sentry_decision_msg::msg::TunnelMonitor tunnel_monitor_msg_;
    geometry_msgs::msg::PointStamped current_msg;
    bool has_current_pos_{false};
    int switch_flag=0;
    double enemy_pos_scale_{0.1};
    bool enemy_pos_is_delta_{true};
    std::string tunnel_monitor_topic_{"tunnel_monitor"};
    
    

};

#endif

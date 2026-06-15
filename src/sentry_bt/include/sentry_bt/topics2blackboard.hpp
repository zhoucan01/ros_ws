
#ifndef SENTRY_BT_TOPICS2BLACKBOARD_HPP
#define SENTRY_BT_TOPICS2BLACKBOARD_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/blackboard.h>
#include <sentry_decision_msg/msg/enemy_pos.hpp>
#include <sentry_decision_msg/msg/host_decision.hpp>
#include <sentry_decision_msg/msg/referee_raw.hpp>
#include <sentry_decision_msg/msg/manual_pos.hpp>
#include <sentry_decision_msg/msg/tunnel_monitor.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include "algo_master/msg/plc2_target.hpp"

class BlackboardUpdater : public rclcpp::Node
{
public:
    explicit BlackboardUpdater(BT::Blackboard::Ptr blackboard);

private:
    void UpdateHostDecision();
    void ResetMatchDerivedState();
    void UpdateMatchLifecycle();
    void ResetAttitudeDerivedState();
    void UpdateStayHomeState();
    void UpdateAttitudeDecision();
    void UpdateAttitudeTimers();
    void ApplyRefereeRawToBlackboard(const sentry_decision_msg::msg::RefereeRaw &msg);
    void judge_if_need_allow_17();
    void judge_if_hp_state();
    void judge_if_hurt_state();
    bool judge_if_allowance_less_50() const;
    bool judge_if_allowance_less_100() const;
    bool judge_if_need_hp_recover() const;
    bool judge_if_match_started() const;
    bool judge_if_enemy_outpost_alive() const;
    bool judge_if_radar_outpost_target() const;
    bool judge_if_force_enemy_outpost() const;
    bool judge_if_base_full_hp() const;
    bool judge_if_base_low_hp() const;
    bool judge_if_can_rebuild_outpost() const;
    bool judge_if_manual_target_valid() const;
    bool judge_if_target_far() const;
    bool judge_if_energy_below_15() const;
    bool judge_if_attack_attitude_weakened() const;
    bool judge_if_defense_attitude_weakened() const;
    bool judge_if_move_attitude_weakened() const;
    bool isAttackableArmorId(uint8_t armor_id) const;
    bool judge_if_force_stay_home() const;
    bool getBlackboardBool(const std::string &key, bool fallback = false) const;
    int getBlackboardInt(const std::string &key, int fallback = 0) const;
    double calc_distance_to_home_depot() const;
    double calc_distance_to_target() const;
    void refresh_target_state();
    uint16_t calc_allow_to_get_17mm() const;
    uint16_t calc_available_allowance_17() const;
    uint8_t calc_remain_time() const;
    int calc_attack_attitude_score() const;
    int calc_defense_attitude_score() const;
    int calc_move_attitude_score() const;
    int select_desired_sentry_attitude() const;
    sentry_decision_msg::msg::HostDecision BuildHostDecisionMsg(bool if_get_allow_17) const;

    BT::Blackboard::Ptr blackboard_;
    rclcpp::Subscription<sentry_decision_msg::msg::RefereeRaw>::SharedPtr referee_raw_sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::ManualPos>::SharedPtr manual_pos_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::EnemyPos>::SharedPtr enemypos_sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::TunnelMonitor>::SharedPtr tunnel_monitor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr current_pos_sub_;
    rclcpp::Subscription<algo_master::msg::PLC2Target>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrived_sub_;
    rclcpp::Publisher<sentry_decision_msg::msg::HostDecision>::SharedPtr host_decision_pub_;
    rclcpp::TimerBase::SharedPtr decision_timer_;
    sentry_decision_msg::msg::RefereeRaw referee_raw_msg_;
    geometry_msgs::msg::PoseStamped manual_target_pose_;
    sentry_decision_msg::msg::TunnelMonitor tunnel_monitor_msg_;
    geometry_msgs::msg::PointStamped current_msg;
    bool has_current_pos_{false};
    bool has_referee_raw_{false};
    bool has_manual_target_{false};
    int switch_flag=0;
    uint8_t latest_game_state_{0};
    uint16_t latest_game_remain_time_{420};
    uint16_t last_allowance_17_{300};
    uint16_t allow_to_get_17mm_{0};
    uint16_t already_allowance_17_{0};
    uint16_t available_allowance_17_{0};
    uint16_t last_current_hp_{400};
    uint16_t last_we_outpost_hp_{0};
    uint16_t rebuild_outpost_used_count_{0};
    uint16_t last_game_remain_time_seen_{420};
    uint8_t remain_time_{0};
    bool if_get_allow_17_{false};
    bool if_hp_less_50_{false};
    bool if_hp_less_100_{false};
    bool if_recently_hurt_{false};
    bool force_stay_home_{false};
    bool stay_home_for_ammo_{false};
    bool if_5s_not_hurted_{true};
    bool if_3s_not_hurted_{true};
    uint16_t attitude_time_attack_{0};
    uint16_t attitude_time_defense_{0};
    uint16_t attitude_time_move_{0};
    int desired_sentry_attitude_{3};
    int attack_attitude_score_{0};
    int defense_attitude_score_{0};
    int move_attitude_score_{0};
    int attack_score_need_attack_{40};
    int attack_score_arrived_{30};
    int attack_score_recently_hurt_penalty_{20};
    int attack_score_far_penalty_{20};
    int attack_score_near_bonus_{10};
    int attack_score_weakened_penalty_{60};
    int defense_score_recently_hurt_{40};
    int defense_score_low_hp_{20};
    int defense_score_far_penalty_{10};
    int defense_score_near_bonus_{10};
    int defense_score_weakened_penalty_{40};
    int move_score_target_far_{40};
    int move_score_arrived_penalty_{10};
    int move_score_not_arrived_bonus_{10};
    int move_score_recently_hurt_penalty_{10};
    int move_score_weakened_penalty_{30};
    uint16_t attitude_weaken_threshold_s_{180};
    double latest_target_x_{0.0};
    double latest_target_y_{0.0};
    bool has_latest_target_{false};
    bool has_seen_we_outpost_hp_{false};
    bool last_match_started_{false};
    uint16_t last_hurt_game_remain_time_{420};
    uint8_t last_real_sentry_attitude_switch_{3};
    rclcpp::Time last_attitude_log_time_{0, 0, RCL_ROS_TIME};
    double enemy_pos_scale_{0.1};
    bool enemy_pos_is_delta_{true};
    double target_far_threshold_{1.5};
    double allowance_return_speed_{1.0};
    double allowance_return_buffer_s_{3.0};
    double init_target_x_{3.76};
    double init_target_y_{8.0};
    double home_target_x_{2.71};
    double home_target_y_{2.24};
    uint8_t remaining_energy_flags_{0};
    int opening_outpost_force_time_s_{240};
    double manual_map_x_max_{28.0};
    double manual_map_y_max_{15.0};
    std::string tunnel_monitor_topic_{"tunnel_monitor"};
    std::string manual_pos_topic_{"manual_pos_msg"};
    std::vector<int64_t> attackable_armor_ids_;
    
    

};

#endif


#ifndef SENTRY_BT_TOPICS2BLACKBOARD_HPP
#define SENTRY_BT_TOPICS2BLACKBOARD_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/blackboard.h>
#include <array>
#include <sentry_decision_msg/msg/enemy_pos.hpp>
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
    struct PredictiveState
    {
        bool need_attack{false};
        bool arrived{false};
        bool target_far{true};
        bool recently_hurt{false};
        bool hp_low{false};
        bool need_home{false};
        int current_attitude{3};
        int cooldown_remaining_s{0};
        uint16_t shoot_heat{0};
        uint16_t heat_limit{0};
        uint16_t heat_cool_rate{0};
        uint16_t attack_time{0};
        uint16_t defense_time{0};
        uint16_t move_time{0};
    };

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
    // 丢目标滞回：只要最后一次"看到可攻击目标"在 target_lost_timeout_s_ 内，
    // 就保持 if_need_to_attack（继续用最后已知敌人位置追击），超时才置 false。
    void update_attack_intent();
    double calc_distance_to_home_depot() const;
    double calc_distance_to_target() const;
    void refresh_target_state();
    uint16_t calc_allow_to_get_17mm() const;
    uint16_t calc_available_allowance_17() const;
    uint8_t calc_remain_time() const;
    PredictiveState BuildPredictiveState() const;
    int calc_attitude_cooldown_remaining_s() const;
    int calc_attitude_stage_score(int attitude, const PredictiveState &state) const;
    int calc_attitude_rollout_score(const PredictiveState &state, int depth) const;
    int calc_attitude_rollout_score_for_action(const PredictiveState &state, int action, int depth) const;
    PredictiveState simulate_attitude_step(const PredictiveState &state, int action) const;
    int calc_attack_attitude_score() const;
    int calc_defense_attitude_score() const;
    int calc_move_attitude_score() const;
    int select_desired_sentry_attitude() const;

    BT::Blackboard::Ptr blackboard_;
    rclcpp::Subscription<sentry_decision_msg::msg::RefereeRaw>::SharedPtr referee_raw_sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::ManualPos>::SharedPtr manual_pos_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::EnemyPos>::SharedPtr enemypos_sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::TunnelMonitor>::SharedPtr tunnel_monitor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr current_pos_sub_;
    rclcpp::Subscription<algo_master::msg::PLC2Target>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrived_sub_;
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
    int move_score_go_home_bonus_{25};
    int move_score_weakened_penalty_{30};
    int attack_score_heat_risk_penalty_{40};
    int defense_score_heat_risk_penalty_{15};
    int move_score_heat_relief_bonus_{20};
    uint16_t attitude_weaken_threshold_s_{180};
    int attitude_prediction_horizon_{4};
    int attitude_prediction_step_s_{1};
    int attitude_switch_penalty_{12};
    int attitude_keep_current_bonus_{3};
    int attitude_cooldown_s_{5};
    double latest_target_x_{0.0};
    double latest_target_y_{0.0};
    bool has_latest_target_{false};
    bool has_seen_we_outpost_hp_{false};
    bool last_match_started_{false};
    uint16_t last_hurt_game_remain_time_{420};
    uint8_t last_real_sentry_attitude_switch_{3};
    uint16_t last_attitude_change_game_remain_time_{420};
    bool has_real_attitude_seen_{false};
    rclcpp::Time last_attitude_log_time_{0, 0, RCL_ROS_TIME};
    double enemy_pos_scale_{0.1};
    bool enemy_pos_is_delta_{true};
    double target_far_threshold_{1.5};
    double allowance_return_speed_{1.0};
    double allowance_return_buffer_s_{3.0};
    // 丢目标滞回：最后一次成功看到可攻击目标的时间
    rclcpp::Time last_valid_enemy_time_{0, 0, RCL_ROS_TIME};
    bool has_valid_enemy_{false};
    double target_lost_timeout_s_{3.0};
    double init_target_x_{3.76};
    double init_target_y_{8.0};
    double home_target_x_{2.71};
    double home_target_y_{2.24};
    uint16_t current_shoot_heat_17mm_{0};
    uint16_t heat_limit_17mm_{0};
    uint16_t heat_cool_rate_17mm_{0};
    uint8_t remaining_energy_flags_{0};
    int opening_outpost_force_time_s_{240};
    double manual_map_x_max_{28.0};
    double manual_map_y_max_{15.0};
    std::string tunnel_monitor_topic_{"tunnel_monitor"};
    std::string manual_pos_topic_{"manual_pos_msg"};
    std::vector<int64_t> attackable_armor_ids_;
    
    

};

#endif

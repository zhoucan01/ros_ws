// 黑板更新器：订阅 /sentry_decision 话题，将数据写入黑板

#include "sentry_bt/topics2blackboard.hpp"
#include <cmath>
#include <limits>

void BlackboardUpdater::ResetMatchDerivedState()
{
    latest_game_remain_time_ = 420;
    last_game_remain_time_seen_ = 420;
    last_allowance_17_ = 300;
    allow_to_get_17mm_ = 0;
    already_allowance_17_ = 0;
    available_allowance_17_ = 0;
    remain_time_ = 0;
    if_get_allow_17_ = false;
    if_hp_less_50_ = false;
    if_hp_less_100_ = false;
    if_recently_hurt_ = false;
    force_stay_home_ = false;
    stay_home_for_ammo_ = false;
    if_5s_not_hurted_ = true;
    if_3s_not_hurted_ = true;
    last_current_hp_ = 400;
    rebuild_outpost_used_count_ = 0;
    has_seen_we_outpost_hp_ = false;
    last_we_outpost_hp_ = 0;
    last_hurt_game_remain_time_ = 420;
    ResetAttitudeDerivedState();
}

void BlackboardUpdater::ResetAttitudeDerivedState()
{
    attitude_time_attack_ = 0;
    attitude_time_defense_ = 0;
    attitude_time_move_ = 0;
    desired_sentry_attitude_ = 3;
    attack_attitude_score_ = 0;
    defense_attitude_score_ = 0;
    move_attitude_score_ = 0;
    last_real_sentry_attitude_switch_ = 3;
}

void BlackboardUpdater::UpdateMatchLifecycle()
{
    const bool match_started = judge_if_match_started();
    if (match_started && !last_match_started_) {
        ResetMatchDerivedState();
        last_match_started_ = true;
        return;
    }
    if (!match_started && last_match_started_) {
        ResetMatchDerivedState();
        last_match_started_ = false;
        return;
    }
    if (latest_game_remain_time_ > last_game_remain_time_seen_) {
        ResetMatchDerivedState();
    }
}

bool BlackboardUpdater::judge_if_match_started() const
{
    return latest_game_state_ == 0x04;
}

bool BlackboardUpdater::judge_if_allowance_less_50() const
{
    return static_cast<int>(referee_raw_msg_.projectile_allowance_17mm) < 50;
}

bool BlackboardUpdater::judge_if_allowance_less_100() const
{
    return static_cast<int>(referee_raw_msg_.projectile_allowance_17mm) < 100;
}

bool BlackboardUpdater::judge_if_need_hp_recover() const
{
    return if_hp_less_100_;
}

bool BlackboardUpdater::judge_if_enemy_outpost_alive() const
{
    return referee_raw_msg_.enemy_outpost_hp > 0;
}

bool BlackboardUpdater::judge_if_radar_outpost_target() const
{
    return referee_raw_msg_.if_get_radar_msg != 0 && judge_if_enemy_outpost_alive();
}

bool BlackboardUpdater::judge_if_force_enemy_outpost() const
{
    return latest_game_remain_time_ >= static_cast<uint16_t>(420 - opening_outpost_force_time_s_);
}

bool BlackboardUpdater::judge_if_base_full_hp() const
{
    return referee_raw_msg_.my_base_hp == 5000;
}

bool BlackboardUpdater::judge_if_base_low_hp() const
{
    return referee_raw_msg_.my_base_hp < 2000;
}

bool BlackboardUpdater::judge_if_can_rebuild_outpost() const
{
    if (latest_game_remain_time_ <= 120) {
        return false;
    }
    const uint16_t lost_base_hp = static_cast<uint16_t>(5000 - std::min<uint16_t>(referee_raw_msg_.my_base_hp, 5000));
    const uint16_t rebuild_chances = lost_base_hp / 1000;
    return rebuild_chances > rebuild_outpost_used_count_;
}

bool BlackboardUpdater::judge_if_manual_target_valid() const
{
    if (!has_manual_target_ || referee_raw_msg_.if_get_manual_msg == 0) {
        return false;
    }
    const double x = manual_target_pose_.pose.position.x;
    const double y = manual_target_pose_.pose.position.y;
    return std::isfinite(x) && std::isfinite(y) &&
        std::fabs(x) <= manual_map_x_max_ && std::fabs(y) <= manual_map_y_max_;
}

bool BlackboardUpdater::judge_if_target_far() const
{
    if (!has_current_pos_ || !has_latest_target_) {
        return true;
    }
    return calc_distance_to_target() > target_far_threshold_;
}

bool BlackboardUpdater::judge_if_energy_below_15() const
{
    constexpr uint8_t kEnergyGe15Bit = 1u << 4;
    return (remaining_energy_flags_ & kEnergyGe15Bit) == 0;
}

bool BlackboardUpdater::judge_if_attack_attitude_weakened() const
{
    return attitude_time_attack_ > attitude_weaken_threshold_s_;
}

bool BlackboardUpdater::judge_if_defense_attitude_weakened() const
{
    return attitude_time_defense_ > attitude_weaken_threshold_s_;
}

bool BlackboardUpdater::judge_if_move_attitude_weakened() const
{
    return attitude_time_move_ > attitude_weaken_threshold_s_;
}

int BlackboardUpdater::calc_attitude_cooldown_remaining_s() const
{
    if (!has_real_attitude_seen_ || last_game_remain_time_seen_ < latest_game_remain_time_)
    {
        return 0;
    }

    const int elapsed =
        static_cast<int>(last_attitude_change_game_remain_time_) - static_cast<int>(latest_game_remain_time_);
    if (elapsed < 0)
    {
        return attitude_cooldown_s_;
    }

    return std::max(0, attitude_cooldown_s_ - elapsed);
}

BlackboardUpdater::PredictiveState BlackboardUpdater::BuildPredictiveState() const
{
    PredictiveState state;
    state.need_attack = getBlackboardBool("if_need_to_attack");
    state.arrived = getBlackboardBool("if_arrived");
    state.target_far = judge_if_target_far();
    state.recently_hurt = if_recently_hurt_;
    state.hp_low = if_hp_less_100_;
    state.need_home = force_stay_home_ || if_get_allow_17_ || if_hp_less_100_;
    state.current_attitude = static_cast<int>(referee_raw_msg_.real_sentry_attitude_switch);
    if (state.current_attitude < 1 || state.current_attitude > 3)
    {
        state.current_attitude = 3;
    }
    state.cooldown_remaining_s = calc_attitude_cooldown_remaining_s();
    state.shoot_heat = current_shoot_heat_17mm_;
    state.heat_limit = heat_limit_17mm_;
    state.heat_cool_rate = heat_cool_rate_17mm_;
    state.attack_time = attitude_time_attack_;
    state.defense_time = attitude_time_defense_;
    state.move_time = attitude_time_move_;
    return state;
}

int BlackboardUpdater::calc_attitude_stage_score(int attitude, const PredictiveState &state) const
{
    int score = 0;
    const int effective_cool_rate = attitude == 1 ? static_cast<int>(state.heat_cool_rate) : 0;
    const int projected_net_heat = std::max(
        0,
        static_cast<int>(state.shoot_heat) - effective_cool_rate);
    const bool near_heat_limit =
        state.heat_limit > 0 &&
        projected_net_heat >= static_cast<int>(state.heat_limit) * 8 / 10;
    const bool over_heat_limit =
        state.heat_limit > 0 && projected_net_heat >= static_cast<int>(state.heat_limit);

    switch (attitude)
    {
    case 1:
        score += state.need_attack ? attack_score_need_attack_ : 0;
        score += state.arrived ? attack_score_arrived_ : 0;
        score += state.recently_hurt ? -attack_score_recently_hurt_penalty_ : 0;
        score += state.target_far ? -attack_score_far_penalty_ : attack_score_near_bonus_;
        score += state.attack_time > attitude_weaken_threshold_s_ ? -attack_score_weakened_penalty_ : 0;
        score += state.hp_low ? -defense_score_low_hp_ : 0;
        score += state.need_home ? -move_score_go_home_bonus_ : 0;
        score += over_heat_limit ? -attack_score_heat_risk_penalty_ * 2 : 0;
        score += near_heat_limit ? -attack_score_heat_risk_penalty_ : 0;
        break;

    case 2:
        score += state.recently_hurt ? defense_score_recently_hurt_ : 0;
        score += state.hp_low ? defense_score_low_hp_ : 0;
        score += state.target_far ? -defense_score_far_penalty_ : defense_score_near_bonus_;
        score += state.defense_time > attitude_weaken_threshold_s_ ? -defense_score_weakened_penalty_ : 0;
        score += state.need_attack ? attack_score_need_attack_ / 5 : 0;
        score += over_heat_limit ? -defense_score_heat_risk_penalty_ * 2 : 0;
        score += near_heat_limit ? -defense_score_heat_risk_penalty_ : 0;
        break;

    case 3:
    default:
        score += state.target_far ? move_score_target_far_ : 0;
        score += state.arrived ? -move_score_arrived_penalty_ : move_score_not_arrived_bonus_;
        score += state.recently_hurt ? -move_score_recently_hurt_penalty_ : 0;
        score += state.move_time > attitude_weaken_threshold_s_ ? -move_score_weakened_penalty_ : 0;
        score += state.need_home ? move_score_go_home_bonus_ : 0;
        score += near_heat_limit ? move_score_heat_relief_bonus_ : 0;
        score += over_heat_limit ? move_score_heat_relief_bonus_ * 2 : 0;
        break;
    }

    if (attitude == state.current_attitude)
    {
        score += attitude_keep_current_bonus_;
    }
    else if (state.cooldown_remaining_s > 0)
    {
        score -= 1000;
    }
    else
    {
        score -= attitude_switch_penalty_;
    }

    return score;
}

BlackboardUpdater::PredictiveState BlackboardUpdater::simulate_attitude_step(
    const PredictiveState &state, int action) const
{
    PredictiveState next = state;

    if (action != next.current_attitude && next.cooldown_remaining_s == 0)
    {
        next.current_attitude = action;
        next.cooldown_remaining_s = attitude_cooldown_s_;
    }

    const int step = std::max(1, attitude_prediction_step_s_);
    next.cooldown_remaining_s = std::max(0, next.cooldown_remaining_s - step);

    int predicted_heat = static_cast<int>(next.shoot_heat);
    if (next.need_attack)
    {
        predicted_heat += step * 15;
    }
    if (next.current_attitude == 1)
    {
        predicted_heat -= static_cast<int>(next.heat_cool_rate) * step;
    }
    next.shoot_heat = static_cast<uint16_t>(std::max(0, predicted_heat));

    switch (next.current_attitude)
    {
    case 1:
        next.attack_time = static_cast<uint16_t>(next.attack_time + step);
        break;

    case 2:
        next.defense_time = static_cast<uint16_t>(next.defense_time + step);
        break;

    case 3:
    default:
        next.move_time = static_cast<uint16_t>(next.move_time + step);
        break;
    }

    return next;
}

int BlackboardUpdater::calc_attitude_rollout_score(const PredictiveState &state, int depth) const
{
    if (depth <= 0)
    {
        return 0;
    }

    int best_score = std::numeric_limits<int>::min();
    for (int action = 1; action <= 3; ++action)
    {
        best_score = std::max(best_score, calc_attitude_rollout_score_for_action(state, action, depth));
    }
    return best_score;
}

int BlackboardUpdater::calc_attitude_rollout_score_for_action(
    const PredictiveState &state, int action, int depth) const
{
    const int stage_score = calc_attitude_stage_score(action, state);
    if (stage_score <= -1000)
    {
        return stage_score;
    }

    const PredictiveState next_state = simulate_attitude_step(state, action);
    return stage_score + calc_attitude_rollout_score(next_state, depth - 1);
}

bool BlackboardUpdater::isAttackableArmorId(uint8_t armor_id) const
{
    return std::find(
               attackable_armor_ids_.begin(),
               attackable_armor_ids_.end(),
               static_cast<int64_t>(armor_id)) != attackable_armor_ids_.end();
}

bool BlackboardUpdater::judge_if_force_stay_home() const
{
    return force_stay_home_;
}

bool BlackboardUpdater::getBlackboardBool(const std::string &key, bool fallback) const
{
    bool value = fallback;
    if (blackboard_->get(key, value)) {
        return value;
    }
    return fallback;
}

int BlackboardUpdater::getBlackboardInt(const std::string &key, int fallback) const
{
    int value = fallback;
    if (blackboard_->get(key, value)) {
        return value;
    }
    return fallback;
}

uint16_t BlackboardUpdater::calc_allow_to_get_17mm() const
{
    return static_cast<uint16_t>(((420 - latest_game_remain_time_) / 60) * 100);
}

uint16_t BlackboardUpdater::calc_available_allowance_17() const
{
    return allow_to_get_17mm_ > already_allowance_17_ ?
        static_cast<uint16_t>(allow_to_get_17mm_ - already_allowance_17_) : 0;
}

double BlackboardUpdater::calc_distance_to_home_depot() const
{
    if (!has_current_pos_) {
        return 0.0;
    }
    const auto home_x = home_target_x_ - init_target_x_;
    const auto home_y = home_target_y_ - init_target_y_;
    const double dx = home_x - current_msg.point.x;
    const double dy = home_y - current_msg.point.y;
    return std::hypot(dx, dy);
}

double BlackboardUpdater::calc_distance_to_target() const
{
    if (!has_current_pos_ || !has_latest_target_) {
        return 0.0;
    }
    const double dx = latest_target_x_ - current_msg.point.x;
    const double dy = latest_target_y_ - current_msg.point.y;
    return std::hypot(dx, dy);
}

void BlackboardUpdater::refresh_target_state()
{
    blackboard_->set("if_target_far", judge_if_target_far());
}

uint8_t BlackboardUpdater::calc_remain_time() const
{
    return static_cast<uint8_t>(latest_game_remain_time_ % 60);
}

void BlackboardUpdater::UpdateAttitudeTimers()
{
    if (!judge_if_match_started()) {
        return;
    }
    if (last_game_remain_time_seen_ < latest_game_remain_time_) {
        return;
    }
    const uint16_t dt = static_cast<uint16_t>(last_game_remain_time_seen_ - latest_game_remain_time_);
    if (dt == 0) {
        return;
    }

    switch (referee_raw_msg_.real_sentry_attitude_switch)
    {
    case 1:
        attitude_time_attack_ += dt;
        break;
    case 2:
        attitude_time_defense_ += dt;
        break;
    case 3:
    default:
        attitude_time_move_ += dt;
        break;
    }

    if (!has_real_attitude_seen_)
    {
        has_real_attitude_seen_ = true;
    }
    else if (referee_raw_msg_.real_sentry_attitude_switch != last_real_sentry_attitude_switch_)
    {
        last_attitude_change_game_remain_time_ = latest_game_remain_time_;
    }
    last_real_sentry_attitude_switch_ = referee_raw_msg_.real_sentry_attitude_switch;
}

int BlackboardUpdater::calc_attack_attitude_score() const
{
    return calc_attitude_rollout_score_for_action(
        BuildPredictiveState(), 1, std::max(1, attitude_prediction_horizon_));
}

int BlackboardUpdater::calc_defense_attitude_score() const
{
    return calc_attitude_rollout_score_for_action(
        BuildPredictiveState(), 2, std::max(1, attitude_prediction_horizon_));
}

int BlackboardUpdater::calc_move_attitude_score() const
{
    return calc_attitude_rollout_score_for_action(
        BuildPredictiveState(), 3, std::max(1, attitude_prediction_horizon_));
}

int BlackboardUpdater::select_desired_sentry_attitude() const
{
    if (referee_raw_msg_.current_hp == 0) {
        return 2;
    }
    if (attack_attitude_score_ >= defense_attitude_score_ &&
        attack_attitude_score_ >= move_attitude_score_) {
        return 1;
    }
    if (defense_attitude_score_ >= move_attitude_score_) {
        return 2;
    }
    return 3;
}

void BlackboardUpdater::UpdateAttitudeDecision()
{
    UpdateAttitudeTimers();
    attack_attitude_score_ = calc_attack_attitude_score();
    defense_attitude_score_ = calc_defense_attitude_score();
    move_attitude_score_ = calc_move_attitude_score();
    desired_sentry_attitude_ = select_desired_sentry_attitude();

    blackboard_->set("desired_sentry_attitude", desired_sentry_attitude_);
    blackboard_->set("real_sentry_attitude_switch", static_cast<int>(referee_raw_msg_.real_sentry_attitude_switch));
    blackboard_->set("attack_attitude_time", static_cast<int>(attitude_time_attack_));
    blackboard_->set("defense_attitude_time", static_cast<int>(attitude_time_defense_));
    blackboard_->set("move_attitude_time", static_cast<int>(attitude_time_move_));
    blackboard_->set("attack_attitude_weakened", judge_if_attack_attitude_weakened());
    blackboard_->set("defense_attitude_weakened", judge_if_defense_attitude_weakened());
    blackboard_->set("move_attitude_weakened", judge_if_move_attitude_weakened());

    const auto now = this->now();
    if (last_attitude_log_time_.nanoseconds() == 0 || (now - last_attitude_log_time_).seconds() >= 1.0)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "attitude select: real=%u desired=%d score[a=%d d=%d m=%d] time[a=%u d=%u m=%u] weak[a=%d d=%d m=%d] target_far=%d arrived=%d need_attack=%d hurt=%d",
            static_cast<unsigned>(referee_raw_msg_.real_sentry_attitude_switch),
            desired_sentry_attitude_,
            attack_attitude_score_,
            defense_attitude_score_,
            move_attitude_score_,
            static_cast<unsigned>(attitude_time_attack_),
            static_cast<unsigned>(attitude_time_defense_),
            static_cast<unsigned>(attitude_time_move_),
            judge_if_attack_attitude_weakened() ? 1 : 0,
            judge_if_defense_attitude_weakened() ? 1 : 0,
            judge_if_move_attitude_weakened() ? 1 : 0,
            judge_if_target_far() ? 1 : 0,
            getBlackboardBool("if_arrived") ? 1 : 0,
            getBlackboardBool("if_need_to_attack") ? 1 : 0,
            if_recently_hurt_ ? 1 : 0);
        last_attitude_log_time_ = now;
    }
}

void BlackboardUpdater::judge_if_need_allow_17()
{
    const int current_allowance = std::max(0, static_cast<int>(referee_raw_msg_.projectile_allowance_17mm));

    if (current_allowance > static_cast<int>(last_allowance_17_))
    {
        already_allowance_17_ += static_cast<uint16_t>(current_allowance - last_allowance_17_);
    }

    allow_to_get_17mm_ = calc_allow_to_get_17mm();
    remain_time_ = calc_remain_time();
    available_allowance_17_ = calc_available_allowance_17();

    const double distance_to_home = calc_distance_to_home_depot();
    const double travel_time_threshold =
        distance_to_home / std::max(allowance_return_speed_, 1e-3) + allowance_return_buffer_s_;

    if_get_allow_17_ =
        (current_allowance < 50 && available_allowance_17_ >= 200) ||
        (current_allowance < 20 && available_allowance_17_ >= 100) ||
        (current_allowance < 5 && static_cast<double>(remain_time_) < travel_time_threshold);

    last_allowance_17_ = static_cast<uint16_t>(current_allowance);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "allowance judge: cur=%d allow_to_get=%u got=%u left=%u remain=%u dist_home=%.2f travel_th=%.2f need=%d",
        current_allowance,
        static_cast<unsigned>(allow_to_get_17mm_),
        static_cast<unsigned>(already_allowance_17_),
        static_cast<unsigned>(available_allowance_17_),
        static_cast<unsigned>(remain_time_),
        distance_to_home,
        travel_time_threshold,
        if_get_allow_17_ ? 1 : 0);
}

void BlackboardUpdater::UpdateStayHomeState()
{
    const bool need_home_for_hp = if_hp_less_100_;
    const bool need_home_for_ammo = if_get_allow_17_;
    const bool need_home_now = need_home_for_hp || need_home_for_ammo;
    const bool at_home = has_current_pos_ && calc_distance_to_home_depot() < 0.5;
    const bool hp_full = referee_raw_msg_.current_hp >= 400;
    const bool ammo_not_needed = !if_get_allow_17_;

    if (need_home_now)
    {
        force_stay_home_ = true;
        if (need_home_for_ammo)
        {
            stay_home_for_ammo_ = true;
        }
    }

    if (force_stay_home_ && at_home)
    {
        if (stay_home_for_ammo_)
        {
            if (hp_full && ammo_not_needed)
            {
                force_stay_home_ = false;
                stay_home_for_ammo_ = false;
            }
        }
        else if (hp_full)
        {
            force_stay_home_ = false;
        }
    }
}

void BlackboardUpdater::judge_if_hp_state()
{
    const uint16_t current_hp = referee_raw_msg_.current_hp;
    if (current_hp < 50) {
        if_hp_less_50_ = true;
    } else if (current_hp == 400) {
        if_hp_less_50_ = false;
    }

    if (current_hp < 100) {
        if_hp_less_100_ = true;
    } else if (current_hp == 400) {
        if_hp_less_100_ = false;
    }
}

void BlackboardUpdater::judge_if_hurt_state()
{
    const uint16_t current_hp = referee_raw_msg_.current_hp;
    const int hp_drop = static_cast<int>(last_current_hp_) - static_cast<int>(current_hp);
    if (hp_drop >= 10) {
        last_hurt_game_remain_time_ = latest_game_remain_time_;
        if_recently_hurt_ = true;
        if_5s_not_hurted_ = false;
        if_3s_not_hurted_ = false;
    } else {
        if (last_hurt_game_remain_time_ < 420) {
            const uint16_t dt = static_cast<uint16_t>(last_hurt_game_remain_time_ - latest_game_remain_time_);
            if_3s_not_hurted_ = dt >= 3;
            if_5s_not_hurted_ = dt >= 5;
            if_recently_hurt_ = dt < 5;
        } else {
            if_recently_hurt_ = false;
            if_5s_not_hurted_ = true;
            if_3s_not_hurted_ = true;
        }
    }
    last_current_hp_ = current_hp;
}

sentry_decision_msg::msg::HostDecision BlackboardUpdater::BuildHostDecisionMsg(bool if_get_allow_17) const
{
    sentry_decision_msg::msg::HostDecision host_decision_msg;
    host_decision_msg.if_match_started = judge_if_match_started() ? 1 : 0;
    host_decision_msg.if_hp_less_50 = if_hp_less_50_ ? 1 : 0;
    host_decision_msg.if_hp_less_100 = if_hp_less_100_ ? 1 : 0;
    host_decision_msg.if_need_hp_recover = judge_if_need_hp_recover() ? 1 : 0;
    host_decision_msg.if_get_allow_17 = if_get_allow_17 ? 1 : 0;
    host_decision_msg.if_allowance_less_50 = judge_if_allowance_less_50() ? 1 : 0;
    host_decision_msg.if_allowance_less_100 = judge_if_allowance_less_100() ? 1 : 0;
    host_decision_msg.if_get_manual_msg = referee_raw_msg_.if_get_manual_msg;
    host_decision_msg.if_manual_target_valid = judge_if_manual_target_valid() ? 1 : 0;
    host_decision_msg.if_get_radar_msg = referee_raw_msg_.if_get_radar_msg;
    host_decision_msg.if_enemy_outpost_alive = judge_if_enemy_outpost_alive() ? 1 : 0;
    host_decision_msg.if_can_rebuild_outpost = judge_if_can_rebuild_outpost() ? 1 : 0;
    host_decision_msg.if_base_full_hp = judge_if_base_full_hp() ? 1 : 0;
    host_decision_msg.if_base_low_hp = judge_if_base_low_hp() ? 1 : 0;
    host_decision_msg.if_recently_hurt = if_recently_hurt_ ? 1 : 0;
    host_decision_msg.if_5s_not_hurted = if_5s_not_hurted_ ? 1 : 0;
    host_decision_msg.if_target_far = judge_if_target_far() ? 1 : 0;
    host_decision_msg.if_force_enemy_outpost = judge_if_force_enemy_outpost() ? 1 : 0;
    host_decision_msg.real_sentry_attitude_switch = referee_raw_msg_.real_sentry_attitude_switch;
    host_decision_msg.desired_sentry_attitude = static_cast<uint8_t>(desired_sentry_attitude_);
    host_decision_msg.attack_attitude_weakened = judge_if_attack_attitude_weakened() ? 1 : 0;
    host_decision_msg.defense_attitude_weakened = judge_if_defense_attitude_weakened() ? 1 : 0;
    host_decision_msg.move_attitude_weakened = judge_if_move_attitude_weakened() ? 1 : 0;
    host_decision_msg.projectile_allowance_17mm =
        static_cast<uint16_t>(std::max(0, static_cast<int>(referee_raw_msg_.projectile_allowance_17mm)));
    host_decision_msg.current_shoot_heat_17mm = current_shoot_heat_17mm_;
    host_decision_msg.heat_limit_17mm = heat_limit_17mm_;
    host_decision_msg.heat_cool_rate_17mm = heat_cool_rate_17mm_;
    host_decision_msg.attack_attitude_score = static_cast<int16_t>(attack_attitude_score_);
    host_decision_msg.defense_attitude_score = static_cast<int16_t>(defense_attitude_score_);
    host_decision_msg.move_attitude_score = static_cast<int16_t>(move_attitude_score_);
    host_decision_msg.current_hp = referee_raw_msg_.current_hp;
    host_decision_msg.game_remain_time = latest_game_remain_time_;
    host_decision_msg.my_base_hp = referee_raw_msg_.my_base_hp;
    host_decision_msg.we_outpost_hp = referee_raw_msg_.we_outpost_hp;
    host_decision_msg.enemy_outpost_hp = referee_raw_msg_.enemy_outpost_hp;
    host_decision_msg.attack_attitude_time = attitude_time_attack_;
    host_decision_msg.defense_attitude_time = attitude_time_defense_;
    host_decision_msg.move_attitude_time = attitude_time_move_;
    host_decision_msg.last_allowance_17 = last_allowance_17_;
    host_decision_msg.allow_to_get_17mm = allow_to_get_17mm_;
    host_decision_msg.already_allowance_17 = already_allowance_17_;
    host_decision_msg.available_allowance_17 = available_allowance_17_;
    host_decision_msg.remain_time = remain_time_;
    host_decision_msg.manual_target_pose = manual_target_pose_;
    return host_decision_msg;
}

void BlackboardUpdater::ApplyRefereeRawToBlackboard(const sentry_decision_msg::msg::RefereeRaw &msg)
{
    blackboard_->set("referee_raw", msg);
    blackboard_->set("projectile_allowance_17mm", static_cast<int>(msg.projectile_allowance_17mm));
    blackboard_->set("current_hp", static_cast<int>(msg.current_hp));
    blackboard_->set("my_base_hp", static_cast<int>(msg.my_base_hp));
    blackboard_->set("we_outpost_hp", static_cast<int>(msg.we_outpost_hp));
    blackboard_->set("enemy_outpost_hp", static_cast<int>(msg.enemy_outpost_hp));
    blackboard_->set("game_remain_time", static_cast<int>(msg.game_remain_time));
    blackboard_->set("game_state", static_cast<int>(msg.game_state));
    blackboard_->set("if_get_manual_msg", msg.if_get_manual_msg != 0);
    blackboard_->set("if_get_radar_msg", msg.if_get_radar_msg != 0);
    blackboard_->set("enemy_hero_x", static_cast<int>(msg.enemy_hero_x));
    blackboard_->set("enemy_hero_y", static_cast<int>(msg.enemy_hero_y));
    blackboard_->set("real_sentry_attitude_switch", static_cast<int>(msg.real_sentry_attitude_switch));
    blackboard_->set("current_shoot_heat_17mm", static_cast<int>(msg.current_shoot_heat_17mm));
    blackboard_->set("heat_limit_17mm", static_cast<int>(msg.heat_limit_17mm));
    blackboard_->set("heat_cool_rate_17mm", static_cast<int>(msg.heat_cool_rate_17mm));
    blackboard_->set("remaining_energy_flags", static_cast<int>(msg.remaining_energy_flags));
    blackboard_->set("if_energy_below_15", judge_if_energy_below_15());
}

void BlackboardUpdater::UpdateHostDecision()
{
    if (!has_referee_raw_)
    {
        return;
    }

    UpdateMatchLifecycle();

    if (!judge_if_match_started())
    {
        blackboard_->set("if_match_started", false);
        blackboard_->set("if_get_allow_17", false);
        blackboard_->set("if_allowance_less_50", false);
        blackboard_->set("if_allowance_less_100", false);
        blackboard_->set("if_recently_hurt", false);
        blackboard_->set("if_5s_not_hurted", true);
        blackboard_->set("if_3s_not_hurted", true);
        blackboard_->set("desired_sentry_attitude", 3);
        blackboard_->set("real_sentry_attitude_switch", static_cast<int>(referee_raw_msg_.real_sentry_attitude_switch));
        blackboard_->set("attack_attitude_time", 0);
        blackboard_->set("defense_attitude_time", 0);
        blackboard_->set("move_attitude_time", 0);
        blackboard_->set("attack_attitude_weakened", false);
        blackboard_->set("defense_attitude_weakened", false);
        blackboard_->set("move_attitude_weakened", false);
        blackboard_->set("attack_attitude_score", 0);
        blackboard_->set("defense_attitude_score", 0);
        blackboard_->set("move_attitude_score", 0);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "attitude select idle: match_started=0 game_state=%u remain=%u real=%u desired=3",
            static_cast<unsigned>(latest_game_state_),
            static_cast<unsigned>(latest_game_remain_time_),
            static_cast<unsigned>(referee_raw_msg_.real_sentry_attitude_switch));
        return;
    }

    if (!has_seen_we_outpost_hp_) {
        last_we_outpost_hp_ = referee_raw_msg_.we_outpost_hp;
        has_seen_we_outpost_hp_ = true;
    } else {
        if (last_we_outpost_hp_ == 0 && referee_raw_msg_.we_outpost_hp > 0) {
            ++rebuild_outpost_used_count_;
        }
        last_we_outpost_hp_ = referee_raw_msg_.we_outpost_hp;
    }

    judge_if_hp_state();
    judge_if_hurt_state();
    judge_if_need_allow_17();
    UpdateStayHomeState();
    UpdateAttitudeDecision();
    last_game_remain_time_seen_ = latest_game_remain_time_;

    const auto host_decision_msg = BuildHostDecisionMsg(if_get_allow_17_);

    blackboard_->set("if_match_started", judge_if_match_started());
    blackboard_->set("if_hp_less_50", if_hp_less_50_);
    blackboard_->set("if_hp_less_100", if_hp_less_100_);
    blackboard_->set("if_need_hp_recover", judge_if_need_hp_recover());
    blackboard_->set("if_get_allow_17", if_get_allow_17_);
    blackboard_->set("if_force_stay_home", force_stay_home_);
    blackboard_->set("if_allowance_less_50", judge_if_allowance_less_50());
    blackboard_->set("if_allowance_less_100", judge_if_allowance_less_100());
    blackboard_->set("if_enemy_outpost_alive", judge_if_enemy_outpost_alive());
    blackboard_->set("if_radar_outpost_target", judge_if_radar_outpost_target());
    blackboard_->set("if_force_enemy_outpost", judge_if_force_enemy_outpost());
    blackboard_->set("if_base_full_hp", judge_if_base_full_hp());
    blackboard_->set("if_base_low_hp", judge_if_base_low_hp());
    blackboard_->set("if_can_rebuild_outpost", judge_if_can_rebuild_outpost());
    blackboard_->set("if_manual_target_valid", judge_if_manual_target_valid());
    blackboard_->set("if_recently_hurt", if_recently_hurt_);
    blackboard_->set("if_5s_not_hurted", if_5s_not_hurted_);
    blackboard_->set("if_3s_not_hurted", if_3s_not_hurted_);
    refresh_target_state();
    blackboard_->set("if_energy_below_15", judge_if_energy_below_15());
    blackboard_->set(
        "projectile_allowance_17mm",
        static_cast<int>(referee_raw_msg_.projectile_allowance_17mm));
    blackboard_->set("allow_to_get_17mm", static_cast<int>(allow_to_get_17mm_));
    blackboard_->set("already_allowance_17", static_cast<int>(already_allowance_17_));
    blackboard_->set("available_allowance_17", static_cast<int>(available_allowance_17_));
    blackboard_->set("allowance_remain_time", static_cast<int>(remain_time_));
    blackboard_->set("current_shoot_heat_17mm", static_cast<int>(current_shoot_heat_17mm_));
    blackboard_->set("heat_limit_17mm", static_cast<int>(heat_limit_17mm_));
    blackboard_->set("heat_cool_rate_17mm", static_cast<int>(heat_cool_rate_17mm_));

    host_decision_pub_->publish(host_decision_msg);
}

BlackboardUpdater::BlackboardUpdater(BT::Blackboard::Ptr blackboard)
    : Node("blackboard_updater"), blackboard_(blackboard)
{ 
    enemy_pos_scale_ = this->declare_parameter<double>("enemy_pos_scale", enemy_pos_scale_);
    enemy_pos_is_delta_ = this->declare_parameter<bool>("enemy_pos_is_delta", enemy_pos_is_delta_);
    tunnel_monitor_topic_ =
        this->declare_parameter<std::string>("tunnel_monitor_topic", tunnel_monitor_topic_);
    manual_pos_topic_ = this->declare_parameter<std::string>("manual_pos_topic", manual_pos_topic_);
    attackable_armor_ids_ = this->declare_parameter<std::vector<int64_t>>(
        "attackable_armor_ids", std::vector<int64_t>{1, 2, 3, 4, 7});
    target_far_threshold_ = this->declare_parameter<double>("target_far_threshold", target_far_threshold_);
    allowance_return_speed_ = this->declare_parameter<double>("allowance.return_speed", allowance_return_speed_);
    allowance_return_buffer_s_ = this->declare_parameter<double>("allowance.return_buffer_s", allowance_return_buffer_s_);
    {
        const auto init_value = this->declare_parameter<std::vector<double>>(
            "points.0", {init_target_x_, init_target_y_});
        if (init_value.size() >= 2)
        {
            init_target_x_ = init_value[0];
            init_target_y_ = init_value[1];
        }
    }
    {
        const auto home_value = this->declare_parameter<std::vector<double>>(
            "points.1", {home_target_x_, home_target_y_});
        if (home_value.size() >= 2)
        {
            home_target_x_ = home_value[0];
            home_target_y_ = home_value[1];
        }
    }
    opening_outpost_force_time_s_ =
        this->declare_parameter<int>("opening_outpost_force_time_s", opening_outpost_force_time_s_);
    attack_score_need_attack_ = this->declare_parameter<int>("attitude.attack.need_attack", attack_score_need_attack_);
    attack_score_arrived_ = this->declare_parameter<int>("attitude.attack.arrived", attack_score_arrived_);
    attack_score_recently_hurt_penalty_ = this->declare_parameter<int>(
        "attitude.attack.recently_hurt_penalty", attack_score_recently_hurt_penalty_);
    attack_score_far_penalty_ = this->declare_parameter<int>(
        "attitude.attack.target_far_penalty", attack_score_far_penalty_);
    attack_score_near_bonus_ = this->declare_parameter<int>(
        "attitude.attack.target_near_bonus", attack_score_near_bonus_);
    attack_score_weakened_penalty_ = this->declare_parameter<int>(
        "attitude.attack.weakened_penalty", attack_score_weakened_penalty_);

    defense_score_recently_hurt_ = this->declare_parameter<int>(
        "attitude.defense.recently_hurt", defense_score_recently_hurt_);
    defense_score_low_hp_ = this->declare_parameter<int>(
        "attitude.defense.low_hp", defense_score_low_hp_);
    defense_score_far_penalty_ = this->declare_parameter<int>(
        "attitude.defense.target_far_penalty", defense_score_far_penalty_);
    defense_score_near_bonus_ = this->declare_parameter<int>(
        "attitude.defense.target_near_bonus", defense_score_near_bonus_);
    defense_score_weakened_penalty_ = this->declare_parameter<int>(
        "attitude.defense.weakened_penalty", defense_score_weakened_penalty_);

    move_score_target_far_ = this->declare_parameter<int>(
        "attitude.move.target_far", move_score_target_far_);
    move_score_arrived_penalty_ = this->declare_parameter<int>(
        "attitude.move.arrived_penalty", move_score_arrived_penalty_);
    move_score_not_arrived_bonus_ = this->declare_parameter<int>(
        "attitude.move.not_arrived_bonus", move_score_not_arrived_bonus_);
    move_score_recently_hurt_penalty_ = this->declare_parameter<int>(
        "attitude.move.recently_hurt_penalty", move_score_recently_hurt_penalty_);
    move_score_go_home_bonus_ = this->declare_parameter<int>(
        "attitude.move.go_home_bonus", move_score_go_home_bonus_);
    move_score_weakened_penalty_ = this->declare_parameter<int>(
        "attitude.move.weakened_penalty", move_score_weakened_penalty_);
    attack_score_heat_risk_penalty_ = this->declare_parameter<int>(
        "attitude.attack.heat_risk_penalty", attack_score_heat_risk_penalty_);
    defense_score_heat_risk_penalty_ = this->declare_parameter<int>(
        "attitude.defense.heat_risk_penalty", defense_score_heat_risk_penalty_);
    move_score_heat_relief_bonus_ = this->declare_parameter<int>(
        "attitude.move.heat_relief_bonus", move_score_heat_relief_bonus_);

    attitude_weaken_threshold_s_ = static_cast<uint16_t>(this->declare_parameter<int>(
        "attitude.weaken_threshold_s", attitude_weaken_threshold_s_));
    attitude_prediction_horizon_ = this->declare_parameter<int>(
        "attitude.prediction_horizon", attitude_prediction_horizon_);
    attitude_prediction_step_s_ = this->declare_parameter<int>(
        "attitude.prediction_step_s", attitude_prediction_step_s_);
    attitude_switch_penalty_ = this->declare_parameter<int>(
        "attitude.switch_penalty", attitude_switch_penalty_);
    attitude_keep_current_bonus_ = this->declare_parameter<int>(
        "attitude.keep_current_bonus", attitude_keep_current_bonus_);
    attitude_cooldown_s_ = this->declare_parameter<int>(
        "attitude.cooldown_s", attitude_cooldown_s_);

    host_decision_pub_ = this->create_publisher<sentry_decision_msg::msg::HostDecision>(
        "host_decision_msg", 10);

    decision_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { UpdateHostDecision(); });

    referee_raw_sub_ = this->create_subscription<sentry_decision_msg::msg::RefereeRaw>(
        "referee_raw_msg", 10,
        [this](const sentry_decision_msg::msg::RefereeRaw::SharedPtr msg)
        {
            referee_raw_msg_ = *msg;
            has_referee_raw_ = true;
            current_shoot_heat_17mm_ = msg->current_shoot_heat_17mm;
            heat_limit_17mm_ = msg->heat_limit_17mm;
            heat_cool_rate_17mm_ = msg->heat_cool_rate_17mm;
            ApplyRefereeRawToBlackboard(*msg);
            latest_game_remain_time_ = msg->game_remain_time;
            latest_game_state_ = msg->game_state;
            remaining_energy_flags_ = msg->remaining_energy_flags;
            UpdateHostDecision();
        });

    manual_pos_sub_ = this->create_subscription<sentry_decision_msg::msg::ManualPos>(
        manual_pos_topic_, 10,
        [this](const sentry_decision_msg::msg::ManualPos::SharedPtr msg)
        {
            manual_target_pose_.header.stamp = this->now();
            manual_target_pose_.header.frame_id = "map";
            manual_target_pose_.pose.position.x = msg->manual_pos_x;
            manual_target_pose_.pose.position.y = msg->manual_pos_y;
            manual_target_pose_.pose.orientation.w = 1.0;
            has_manual_target_ = true;
            blackboard_->set("manual_target_pose", manual_target_pose_);
            blackboard_->set("if_manual_target_valid", judge_if_manual_target_valid());
            UpdateHostDecision();
        });

    arrived_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "if_arrived", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            blackboard_->set("if_arrived", msg->data);
        });

    auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "global_costmap/costmap", costmap_qos,
        [blackboard](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            std::cout << "cost_map"<<std::endl;
            blackboard->set("nav_globalCostmap", *msg);
        });
    enemypos_sub_ = this->create_subscription<sentry_decision_msg::msg::EnemyPos>(
        "enemy_msg", 1,
        [this, blackboard](const sentry_decision_msg::msg::EnemyPos::SharedPtr msg)
        {
            sentry_decision_msg::msg::EnemyPos enemy_pos_value = *msg;
            blackboard->set("enemy_pos", enemy_pos_value);
            const bool attackable = isAttackableArmorId(msg->armor_id);
            blackboard->set("enemy_armor_id", static_cast<int>(msg->armor_id));
            blackboard->set("if_attack_target_type_allowed", attackable);
            blackboard->set("if_need_to_attack", msg->if_vision_on != 0 && attackable);

            geometry_msgs::msg::PointStamped enemy_pos_point;
            const double ex = static_cast<double>(msg->enemy_pos_x) * enemy_pos_scale_;
            const double ey = static_cast<double>(msg->enemy_pos_y) * enemy_pos_scale_;

            if (enemy_pos_is_delta_)
            {
                if (!has_current_pos_)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "current_pos not ready, skip enemy delta projection");
                    return;
                }
                enemy_pos_point.header = current_msg.header;
                enemy_pos_point.point.x = current_msg.point.x + ex;
                enemy_pos_point.point.y = current_msg.point.y + ey;
            }
            else
            {
                enemy_pos_point.header.stamp = this->now();
                enemy_pos_point.header.frame_id = "map";
                enemy_pos_point.point.x = ex;
                enemy_pos_point.point.y = ey;
            }

            blackboard->set("enemy_pos_point", enemy_pos_point);
        });

    tunnel_monitor_sub_ = this->create_subscription<sentry_decision_msg::msg::TunnelMonitor>(
        tunnel_monitor_topic_, 10,
        [this](const sentry_decision_msg::msg::TunnelMonitor::SharedPtr msg)
        {
            blackboard_->set("tunnel_monitor", *msg);
            blackboard_->set("enable_tunnel_mode", msg->enable_tunnel_mode);
            blackboard_->set("tunnel_has_pose", msg->has_pose);
            blackboard_->set("tunnel_has_plan", msg->has_plan);
            blackboard_->set("will_pass_tunnel", msg->will_pass_tunnel);
            blackboard_->set("in_tunnel", msg->in_tunnel);
            blackboard_->set("disable_spin", msg->disable_spin);
            blackboard_->set("tunnel_recovery_active", msg->tunnel_recovery_active);
            blackboard_->set("target_tunnel_id", static_cast<int>(msg->target_tunnel_id));
            blackboard_->set("tracked_tunnel_id", static_cast<int>(msg->tracked_tunnel_id));
            blackboard_->set("recovery_tunnel_id", static_cast<int>(msg->recovery_tunnel_id));
            blackboard_->set("tunnel_status", static_cast<int>(msg->tunnel_status));
            blackboard_->set("robot_x", msg->robot_x);
            blackboard_->set("robot_y", msg->robot_y);
            blackboard_->set("current_map_yaw", msg->current_map_yaw);
            blackboard_->set("tunnel_target_yaw", msg->tunnel_target_yaw);
            blackboard_->set("tunnel_yaw_error", msg->tunnel_yaw_error);
            blackboard_->set("tunnel_controller_id", msg->controller_id);
            blackboard_->set("tunnel_global_pose", msg->global_pose);
            blackboard_->set("tunnel_recovery_goal", msg->tunnel_recovery_goal);
            tunnel_monitor_msg_ = *msg;
        });

       current_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "current_pos_msg", 1,
       [this, blackboard](const geometry_msgs::msg::PointStamped::SharedPtr msg)
        {
            current_msg = *msg;
            has_current_pos_ = true;
            blackboard->set("current_pos", *msg);
            refresh_target_state();
        });

        target_sub_ = this->create_subscription<algo_master::msg::PLC2Target>(
        "plc2target", 1,
        [this,blackboard](const algo_master::msg::PLC2Target::SharedPtr  msg)
        {    
            latest_target_x_ = msg->target_x;
            latest_target_y_ = msg->target_y;
            has_latest_target_ = true;
            refresh_target_state();
        });





}

// 黑板更新器：订阅 /sentry_decision 话题，将数据写入黑板

#include "sentry_bt/topics2blackboard.hpp"
#include <cmath>

BlackboardUpdater::BlackboardUpdater(BT::Blackboard::Ptr blackboard)
    : Node("blackboard_updater"), blackboard_(blackboard)
{ 
    enemy_pos_scale_ = this->declare_parameter<double>("enemy_pos_scale", enemy_pos_scale_);
    enemy_pos_is_delta_ = this->declare_parameter<bool>("enemy_pos_is_delta", enemy_pos_is_delta_);
    tunnel_monitor_topic_ =
        this->declare_parameter<std::string>("tunnel_monitor_topic", tunnel_monitor_topic_);
    
    sub_ = this->create_subscription<sentry_decision_msg::msg::SentryDecision>(
        "decision_msg", 10,
        [this](const sentry_decision_msg::msg::SentryDecision::SharedPtr msg)
        {
            // 将所有标志写入黑板
            // RCLCPP_INFO(this->get_logger(), "error_1");
            // 将所有标志写入黑板
            // blackboard_->set("if_arrived", msg->if_arrived != 0);
            blackboard_->set("if_3s_not_hurted", msg->if_3s_not_hurted != 0);
            blackboard_->set("if_10s_not_hurted", msg->if_10s_not_hurted != 0);
            blackboard_->set("if_5s_not_hurted", msg->if_5s_not_hurted != 0);
            blackboard_->set("if_3s_not_found", msg->if_3s_not_found != 0);
            blackboard_->set("if_5s_not_found", msg->if_5s_not_found != 0);
            blackboard_->set("if_10s_not_found", msg->if_10s_not_found != 0);
            blackboard_->set("if_hp_less_50", msg->if_hp_less_50 != 0);
            blackboard_->set("if_hp_less_100", msg->if_hp_less_100 != 0);
            blackboard_->set("if_base_armor_spred", msg->if_base_armor_spred != 0);
            blackboard_->set("if_outpost_destroyed", msg->if_outpost_destroyed != 0);
            blackboard_->set("if_fire_lock", msg->if_fire_lock != 0);
            blackboard_->set("if_allowance_less_50", msg->if_allowance_less_50 != 0);
            blackboard_->set("if_allowance_less_100", msg->if_allowance_less_100 != 0);
            blackboard_->set("if_hp_recover", msg->if_hp_recover != 0);
            blackboard_->set("if_on_toss", msg->if_on_toss != 0);
            blackboard_->set("if_need_to_enemy_fortress", msg->if_need_to_enemy_fortress != 0);
            blackboard_->set("if_stop_navi", msg->if_stop_navi != 0);
            blackboard_->set("if_chassis_weak", msg->if_chassis_weak != 0);
            blackboard_->set("if_get_allow_17", msg->if_get_allow_17 != 0);
            blackboard_->set("if_fortress_allow_less_50", msg->if_fortress_allow_less_50 != 0);
            blackboard_->set("if_energy_mechanism", msg->if_energy_mechanism != 0);
            blackboard_->set("if_need_to_protect", msg->if_need_to_protect != 0);
            blackboard_->set("if_fortress_free", msg->if_fortress_free != 0);
            blackboard_->set("if_enemy_outpost_lock", msg->if_enemy_outpost_lock != 0);
            blackboard_->set("if_enemy_outpost_destroyed", msg->if_enemy_outpost_destroyed != 0);
            blackboard_->set("if_moving_v", msg->if_moving_v != 0);
            blackboard_->set("if_chip_base", msg->if_chip_base != 0);
            blackboard_->set("if_hp_less_200", msg->if_hp_less_200 != 0);
            blackboard_->set("if_enemy_small_energy", msg->if_enemy_small_energy != 0);
            blackboard_->set("if_close_to_enemy_out", msg->if_close_to_enemy_out != 0);
            blackboard_->set("game_remain_time", static_cast<int>(msg->game_remain_time));
            blackboard_->set("game_state", static_cast<int>(msg->game_state));
            // blackboard_->set("if_need_to_attack", 1);
            sentry_decision_msg_ = *msg;

        });

    referee_raw_sub_ = this->create_subscription<sentry_decision_msg::msg::RefereeRaw>(
        "referee_raw_msg", 10,
        [this](const sentry_decision_msg::msg::RefereeRaw::SharedPtr msg)
        {
            blackboard_->set("referee_raw", *msg);
            blackboard_->set(
                "projectile_allowance_17mm",
                static_cast<int>(msg->projectile_allowance_17mm));
            blackboard_->set("current_hp", static_cast<int>(msg->current_hp));
            blackboard_->set("my_base_hp", static_cast<int>(msg->my_base_hp));
            blackboard_->set("enemy_hero_x", static_cast<int>(msg->enemy_hero_x));
            blackboard_->set("enemy_hero_y", static_cast<int>(msg->enemy_hero_y));
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
            blackboard->set("if_need_to_attack",msg->if_vision_on);

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
        });

        target_sub_ = this->create_subscription<algo_master::msg::PLC2Target>(
        "plc2target", 1,
        [this,blackboard](const algo_master::msg::PLC2Target::SharedPtr  msg)
        {    
            bool if_arrvied = false;
            algo_master::msg::PLC2Target decisiono_target;
            static algo_master::msg::PLC2Target last_decision_target;
            decisiono_target.target_x = msg->target_x;
            decisiono_target.target_y = msg->target_y; 
            if(std::fabs(last_decision_target.target_x - decisiono_target.target_x)>0.1||
               std::fabs(last_decision_target.target_y - decisiono_target.target_y)>0.1 )
               {
                switch_flag=true;
                if_arrvied=false;
               }

            if (!has_current_pos_)
            {
                blackboard_->set("if_arrived", false);
                last_decision_target.target_x = decisiono_target.target_x;
                last_decision_target.target_y = decisiono_target.target_y;
                return;
            }



            if(std::fabs(decisiono_target.target_x-current_msg.point.x )<0.3&&
               std::fabs(decisiono_target.target_y-current_msg.point.y )<0.3
               )
               {
                
                if_arrvied=true;
               }
               else 
               {
                if_arrvied=false;
               }
               blackboard_->set("if_arrived", if_arrvied);

            last_decision_target.target_x = decisiono_target.target_x;
            last_decision_target.target_y = decisiono_target.target_y;


            // std::cout << current_msg.point.x<<std::endl;
            // blackboard->set("current_pos", *msg);
            // RCLCPP_INFO(this->get_logger(), "error_1");
            
        });





}

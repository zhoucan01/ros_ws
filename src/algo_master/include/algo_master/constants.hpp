#include <nlohmann/json.hpp>
#include <string>
#include <fstream>
#include "algo_master/serialport.hpp"
#include <iostream>
/// msg
// #include "geometry_msgs/TransformStamped.h"
// #include "sensor_msgs/Imu.h"
#include "algo_master/msg/plc2_imu.hpp"
// #include "algo_master/msg/plc2_odom.hpp"
#include "algo_master/msg/plc2_target.hpp"
// #include "/home/ninja/catkin_ws/devel/include/Navigation_tasks/Navigation2PLCMsg.h"
#include "sentry_decision_msg/msg/sentry_decision.hpp"
#include "sentry_decision_msg/msg/enemy_pos.hpp"
#include "sentry_decision_msg/msg/manual_pos.hpp"
#include "sentry_decision_msg/msg/referee_raw.hpp"
/// tf
// #include "tf2_ros/transform_broadcaster.h"
// #include "tf2/LinearMath/Quaternion.h"

using json = nlohmann::json;

const float k_x = -4e-2; // m 平移距离 ,imu robot
const float k_y = -1e-2;
const float k_z = -1e-2;

constexpr double kDegreeToRadCoef = 3.1415926 / 180.0;
constexpr double kRadToDegreeCoef = 180.0 / 3.1415926;

std::string serialflag;

struct SerialPortConfig
{
    std::string portName;
    int baudrate;
    int parity;
    int dataBit;
    int stopBit;
    int synchronize;
    int sendInterval;
} serialPortConfig;

bool InitConfigs(const std::string &pathToConfig)
{
    std::ifstream ifs(pathToConfig);
    if (!ifs.is_open())
        return false;

    json jsonData = json::parse(ifs);
    ifs.close();

    json serialPortNode = jsonData["serialPort"];
    serialPortConfig.baudrate = (int)serialPortNode["baudrate"];
    serialPortConfig.dataBit = (int)serialPortNode["dataBit"];
    serialPortConfig.parity = (int)serialPortNode["parity"];
    serialPortConfig.portName = (std::string)serialPortNode["portName"];
    serialPortConfig.stopBit = (int)serialPortNode["stopBit"];
    serialPortConfig.synchronize = (int)serialPortNode["synchronize"];
    serialPortConfig.sendInterval = (int)serialPortNode["sendInterval"];

    return true;
}

// 此函数中的时间戳无用
algo_master::msg::PLC2Target PLCNavRecv2TargetMsg(const NavSerialMsg &TargetserialRecv)
{
    algo_master::msg::PLC2Target plc2target;
    plc2target.target_x = TargetserialRecv.target_msg.x/10 ; 
    plc2target.target_y = TargetserialRecv.target_msg.y/10 ; 
    // plc2target.header.stamp = ros::Time::now();
    return plc2target;
}

sentry_decision_msg::msg::ManualPos PLCNavRecv2ManualPosMsg(const NavSerialMsg &TargetserialRecv)
{
    sentry_decision_msg::msg::ManualPos manual_pos;
    manual_pos.manual_pos_x = TargetserialRecv.target_msg.x;
    manual_pos.manual_pos_y = TargetserialRecv.target_msg.y;
    return manual_pos;
}




sentry_decision_msg::msg::EnemyPos PLCNavRecv2EnemyMsg(const NavSerialMsg &DecisionserialRecv , double current_yaw)
{
    sentry_decision_msg::msg::EnemyPos enemy_pos;

    enemy_pos.if_vision_on = DecisionserialRecv.Enemy_Pos.If_vision_on;
    enemy_pos.armor_id = DecisionserialRecv.Enemy_Pos.Armor_id;
    enemy_pos.enemy_pos_x  =  DecisionserialRecv.Enemy_Pos.Enemy_x * cos(current_yaw) - DecisionserialRecv.Enemy_Pos.Enemy_y * sin(current_yaw);
    enemy_pos.enemy_pos_y  =  DecisionserialRecv.Enemy_Pos.Enemy_x * sin(current_yaw) + DecisionserialRecv.Enemy_Pos.Enemy_y * cos(current_yaw);

    return enemy_pos;

}

sentry_decision_msg::msg::SentryDecision PLCDecisionRecv2DecisionMsg(const DecisionSerialMsg &DecisionserialRecv)
{
    sentry_decision_msg::msg::SentryDecision sentry_decision;
//     auto logger = rclc:get_logger("dent_logger");
//     // 解析第一个字节（Decision_data_1），对应标志 1~8
//    RCLCPP_INFO(logger, "Received msg");
    // std::cout << "Hello, world!" 
    //       <<  +DecisionserialRecv.Decision_Data.Decision_data_1
    //       <<  "Hello"
    //       <<  +DecisionserialRecv.Decision_Data.Decision_data_2
    //       <<  "Hello"
    //       <<  +DecisionserialRecv.Decision_Data.Decision_data_3 
    //       <<  "Hello"
    //       <<  +DecisionserialRecv.Decision_Data.Decision_data_4 
    //       << std::endl;

    
    sentry_decision.if_get_msg        = (DecisionserialRecv.Decision_Data.Decision_data_1 >> 0) & 1;
    sentry_decision.if_arrived        = (DecisionserialRecv.Decision_Data.Decision_data_1 >> 1) & 1;
    sentry_decision.if_3s_not_hurted  = (DecisionserialRecv.Decision_Data.Decision_data_1 >> 2) & 1;
    sentry_decision.if_10s_not_hurted = (DecisionserialRecv.Decision_Data.Decision_data_1 >> 3) & 1;
    sentry_decision.if_5s_not_hurted  = (DecisionserialRecv.Decision_Data.Decision_data_1 >> 4) & 1;
    sentry_decision.if_3s_not_found   = (DecisionserialRecv.Decision_Data.Decision_data_1 >> 5) & 1;
    sentry_decision.if_5s_not_found   = (DecisionserialRecv.Decision_Data.Decision_data_1 >> 6) & 1;
    sentry_decision.if_10s_not_found  = (DecisionserialRecv.Decision_Data.Decision_data_1 >> 7) & 1;

    // 解析第二个字节（Decision_data_2），对应标志 9~16
    sentry_decision.if_hp_less_50        = (DecisionserialRecv.Decision_Data.Decision_data_2 >> 0) & 1;
    sentry_decision.if_hp_less_100       = (DecisionserialRecv.Decision_Data.Decision_data_2 >> 1) & 1;
    sentry_decision.if_base_armor_spred  = (DecisionserialRecv.Decision_Data.Decision_data_2 >> 2) & 1;
    sentry_decision.if_outpost_destroyed = (DecisionserialRecv.Decision_Data.Decision_data_2 >> 3) & 1;
    sentry_decision.if_fire_lock         = (DecisionserialRecv.Decision_Data.Decision_data_2 >> 4) & 1;
    sentry_decision.if_allowance_less_50 = (DecisionserialRecv.Decision_Data.Decision_data_2 >> 5) & 1;
    sentry_decision.if_allowance_less_100= (DecisionserialRecv.Decision_Data.Decision_data_2 >> 6) & 1;
    sentry_decision.if_hp_recover        = (DecisionserialRecv.Decision_Data.Decision_data_2 >> 7) & 1;

    // 解析第三个字节（Decision_data_3），对应标志 17~24
    sentry_decision.if_on_toss                = (DecisionserialRecv.Decision_Data.Decision_data_3 >> 0) & 1;
    sentry_decision.if_need_to_enemy_fortress = (DecisionserialRecv.Decision_Data.Decision_data_3 >> 1) & 1;
    sentry_decision.if_stop_navi              = (DecisionserialRecv.Decision_Data.Decision_data_3 >> 2) & 1;
    sentry_decision.if_chassis_weak           = (DecisionserialRecv.Decision_Data.Decision_data_3 >> 3) & 1;
    sentry_decision.if_get_allow_17           = (DecisionserialRecv.Decision_Data.Decision_data_3 >> 4) & 1;
    sentry_decision.if_fortress_allow_less_50 = (DecisionserialRecv.Decision_Data.Decision_data_3 >> 5) & 1;
    sentry_decision.if_energy_mechanism       = (DecisionserialRecv.Decision_Data.Decision_data_3 >> 6) & 1;
    sentry_decision.if_need_to_protect        = (DecisionserialRecv.Decision_Data.Decision_data_3 >> 7) & 1;

    // 解析第四个字节（Decision_data_4），对应标志 25~32
    sentry_decision.if_fortress_free           = (DecisionserialRecv.Decision_Data.Decision_data_4 >> 0) & 1;
    sentry_decision.if_enemy_outpost_lock      = (DecisionserialRecv.Decision_Data.Decision_data_4 >> 1) & 1;
    sentry_decision.if_enemy_outpost_destroyed = (DecisionserialRecv.Decision_Data.Decision_data_4 >> 2) & 1;
    sentry_decision.if_moving_v                 = (DecisionserialRecv.Decision_Data.Decision_data_4 >> 3) & 1;
    sentry_decision.if_chip_base                = (DecisionserialRecv.Decision_Data.Decision_data_4 >> 4) & 1;
    sentry_decision.if_hp_less_200              = (DecisionserialRecv.Decision_Data.Decision_data_4 >> 5) & 1;
    sentry_decision.if_enemy_small_energy       = (DecisionserialRecv.Decision_Data.Decision_data_4 >> 6) & 1;
    sentry_decision.if_close_to_enemy_out       = (DecisionserialRecv.Decision_Data.Decision_data_4 >> 7) & 1;
    std::cout << "sentry_decision.if_hp_less_100: " << sentry_decision.if_hp_less_100  << std::endl;

    return sentry_decision;
}

sentry_decision_msg::msg::RefereeRaw PLCDecisionRecv2RefereeRawMsg(
    const DecisionSerialMsg &DecisionserialRecv)
{
    sentry_decision_msg::msg::RefereeRaw referee_raw;
    referee_raw.projectile_allowance_17mm =
        DecisionserialRecv.Referee_Raw_Data.projectile_allowance_17mm;
    referee_raw.current_hp = DecisionserialRecv.Referee_Raw_Data.current_hp;
    referee_raw.my_base_hp = DecisionserialRecv.Referee_Raw_Data.my_base_hp;
    referee_raw.we_outpost_hp = DecisionserialRecv.Referee_Raw_Data.we_outpost_hp;
    referee_raw.enemy_outpost_hp = DecisionserialRecv.Referee_Raw_Data.enemy_outpost_hp;
    referee_raw.game_remain_time = DecisionserialRecv.Decision_Data.game_remain_time;
    referee_raw.game_state = DecisionserialRecv.Decision_Data.game_state;
    referee_raw.if_get_manual_msg =
        DecisionserialRecv.Decision_Update_data.if_get_manual_msg;
    referee_raw.if_get_radar_msg =
        DecisionserialRecv.Decision_Update_data.if_get_radar_msg;
    referee_raw.enemy_hero_x = DecisionserialRecv.Referee_Raw_Data.enemy_hero_x;
    referee_raw.enemy_hero_y = DecisionserialRecv.Referee_Raw_Data.enemy_hero_y;
    referee_raw.real_sentry_attitude_switch =
        DecisionserialRecv.Referee_Raw_Data.real_sentry_attitude_switch;
    referee_raw.remaining_energy_flags =
        DecisionserialRecv.Referee_Raw_Data.remaining_energy_flags;
    referee_raw.current_shoot_heat_17mm =
        DecisionserialRecv.Referee_Raw_Data.current_shoot_heat_17mm;
    referee_raw.heat_limit_17mm =
        DecisionserialRecv.Referee_Raw_Data.heat_limit_17mm;
    referee_raw.heat_cool_rate_17mm =
        DecisionserialRecv.Referee_Raw_Data.heat_cool_rate_17mm;
    return referee_raw;
}


algo_master::msg::PLC2Imu PLCNavRecv2ImuMsg(const NavSerialMsg &ImuDataserialRecv)
{
    algo_master::msg::PLC2Imu plc2imu;
    plc2imu.imu_pitch = ImuDataserialRecv.imu_msg.m_ImuPitch ;   ///yes ro no %1000
    plc2imu.imu_yaw = ImuDataserialRecv.imu_msg.m_ImuYaw ; 
    // std::cout<<plc2imu.imu_yaw<<std::endl;
    // plc2imu.imu_roll = ImuDataserialRecv.imu_msg.m_ImuRoll ;
    // plc2imu.header.stamp = ros::Time::now();my_indepen
    return plc2imu;
}

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sentry_decision_msg/msg/sentry_decision.hpp>
#include <sentry_decision_msg/msg/enemy_pos.hpp>
#include <sentry_decision_msg/msg/attitude_switch.hpp>
#include <map>
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/loggers/groot2_publisher.h> // 添加头文件
#include <geometry_msgs/msg/pose_stamped.hpp>          // For Navigation2 goal
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include "sentry_bt/topics2blackboard.hpp" 
#include "sentry_bt/anti_autoaim.hpp"
#include "algo_master/msg/plc2_target.hpp"

// using namespace pb2025_sentry_behavior
// #include "algo_master/msg/plc2_target.hpp"

// 点枚举（与下位机保持一致）
enum DecisionPoint
{
    INIT_PACK_POINT = 0,
    WE_DEPOT_POINT = 1,
    ENEMY_OUTPOST_POINT = 2,
    WE_OUTPOST_POINT = 3,
    ENEMY_FORTRESS_POINT = 4,
    MANUAL_POINT = 5,
    ENEMY_FLYING_POINT = 6,
    WE_PROTECT_POINT = 7
};

//
    // {INIT_PACK_POINT, {3.76, 8.0}},
    // {WE_DEPOT_POINT, {2.63, 2.33}},
    // {ENEMY_OUTPOST_POINT, {14.50, 11.25}},
    // {WE_OUTPOST_POINT, {11.46, 4.0}},
    // {ENEMY_FORTRESS_POINT, {23.63, 11.69}},
    // {MANUAL_POINT, {3.76, 8.0}},
    // {ENEMY_FLYING_POINT, {23.63, 11.69}},
    // {WE_PROTECT_POINT, {8.4, 8.5}}

static std::map<DecisionPoint, std::pair<float, float>> point_coords = {
    {INIT_PACK_POINT, {3.76, 8.0}},
    {WE_DEPOT_POINT, {2.71, 2.24}},
    {ENEMY_OUTPOST_POINT, {16.80, 14.34}},
    {WE_OUTPOST_POINT, {11.46, 4.0}},
    {ENEMY_FORTRESS_POINT, {23.48, 11.77}},
    {MANUAL_POINT, {3.76, 8.0}},
    {ENEMY_FLYING_POINT, {23.48, 11.77}},
    {WE_PROTECT_POINT, {4.75, 5.67}}
};

static bool g_field_mirror_enabled = false;
static float g_field_length = 28.0f;
static float g_field_width = 15.0f;
static float g_map_origin_x = 3.76f;
static float g_map_origin_y = 8.0f;

static std::pair<float, float> MirrorFieldPoint(float x, float y)
{
    if (!g_field_mirror_enabled) {
        return {x, y};
    }
    return {g_field_length - x, g_field_width - y};
}

static std::pair<float, float> ToInitRelative(float global_x, float global_y)
{
    return {global_x - g_map_origin_x, global_y - g_map_origin_y};
}

static void LoadPointCoords(const rclcpp::Node::SharedPtr &node)
{
    g_field_mirror_enabled = node->declare_parameter<bool>("field_mirror_enable", false);
    g_field_length = static_cast<float>(node->declare_parameter<double>("field_length", 28.0));
    g_field_width = static_cast<float>(node->declare_parameter<double>("field_width", 15.0));
    g_map_origin_x = static_cast<float>(node->declare_parameter<double>("map_origin_x", 3.76));
    g_map_origin_y = static_cast<float>(node->declare_parameter<double>("map_origin_y", 8.0));

    for (const auto &item : point_coords)
    {
        const auto &key = item.first;
        const auto &default_coord = item.second;
        const std::vector<double> def = {
            static_cast<double>(default_coord.first),
            static_cast<double>(default_coord.second)};

        const std::string param_name = "points." + std::to_string(static_cast<int>(key));
        const auto value = node->declare_parameter<std::vector<double>>(param_name, def);
        if (value.size() >= 2)
        {
            point_coords[key] = MirrorFieldPoint(
                static_cast<float>(value[0]),
                static_cast<float>(value[1]));
        }
    }

    RCLCPP_INFO(
        node->get_logger(),
        "decision points use global coordinates; mirror=%d field=(%.3f, %.3f) map_origin=(%.3f, %.3f) decision_init_global=(%.3f, %.3f)",
        g_field_mirror_enabled ? 1 : 0,
        g_field_length,
        g_field_width,
        g_map_origin_x,
        g_map_origin_y,
        point_coords[INIT_PACK_POINT].first,
        point_coords[INIT_PACK_POINT].second);
}
  
static bool ResolveDecisionPoint(int point_id, std::pair<float, float> &coord)
{
    const auto it = point_coords.find(static_cast<DecisionPoint>(point_id));
    if (it == point_coords.end())
    {
        return false;
    }
    coord = it->second;
    return true;
}

static const char *DecisionPointName(int point_id)
{
    switch (static_cast<DecisionPoint>(point_id))
    {
    case INIT_PACK_POINT:
        return "INIT_PACK_POINT";
    case WE_DEPOT_POINT:
        return "WE_DEPOT_POINT";
    case ENEMY_OUTPOST_POINT:
        return "ENEMY_OUTPOST_POINT";
    case WE_OUTPOST_POINT:
        return "WE_OUTPOST_POINT";
    case ENEMY_FORTRESS_POINT:
        return "ENEMY_FORTRESS_POINT";
    case MANUAL_POINT:
        return "MANUAL_POINT";
    case ENEMY_FLYING_POINT:
        return "ENEMY_FLYING_POINT";
    case WE_PROTECT_POINT:
        return "WE_PROTECT_POINT";
    default:
        return "UNKNOWN_POINT";
    }
}

// 动作节点：将目标点编号存入黑板（作为输出端口）
class SetTargetPoint : public BT::SyncActionNode
{
public:
    SetTargetPoint(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("point", "Target point number"),
            BT::OutputPort<int>("target", "Output target point number")};
    }

    BT::NodeStatus tick() override
    {
        int point;
        if (!getInput("point", point))
            return BT::NodeStatus::FAILURE;
        setOutput("target", point);
        return BT::NodeStatus::SUCCESS;
    }
};

class SetManualTarget : public BT::SyncActionNode
{
public:
    SetManualTarget(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("manual_target_pose", "Manual target pose"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("manual_target_output", "Manual target output")};
    }

    BT::NodeStatus tick() override
    {
        auto pose_input = getInput<geometry_msgs::msg::PoseStamped>("manual_target_pose");
        if (!pose_input) {
            return BT::NodeStatus::FAILURE;
        }
        auto pose = pose_input.value();
        const auto mirrored = MirrorFieldPoint(
            static_cast<float>(pose.pose.position.x),
            static_cast<float>(pose.pose.position.y));
        pose.pose.position.x = mirrored.first;
        pose.pose.position.y = mirrored.second;
        setOutput("manual_target_output", pose);
        return BT::NodeStatus::SUCCESS;
    }
};

class ForwardDesiredAttitude : public BT::SyncActionNode
{
public:
    ForwardDesiredAttitude(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("desired_attitude", "Desired attitude from blackboard"),
            BT::OutputPort<int>("attitude_output", "Forwarded desired attitude")};
    }

    BT::NodeStatus tick() override
    {
        auto input = getInput<int>("desired_attitude");
        if (!input) {
            return BT::NodeStatus::FAILURE;
        }
        setOutput("attitude_output", input.value());
        return BT::NodeStatus::SUCCESS;
    }
};

class SelectFinalTarget : public BT::SyncActionNode
{
public:
    SelectFinalTarget(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("normal_target_point", "Normal target point id"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("manual_target_pose", "Manual target pose"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("attack_target_pose", "Attack target pose"),
            BT::InputPort<bool>("attack_target_valid", "Attack target valid"),
            BT::OutputPort<int>("final_target_point", "Final target point id"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_target_pose", "Final target pose"),
            BT::OutputPort<int>("final_target_source", "1 normal, 2 manual, 3 attack")};
    }

    BT::NodeStatus tick() override
    {
        auto normal_point_input = getInput<int>("normal_target_point");
        if (!normal_point_input) {
            return BT::NodeStatus::FAILURE;
        }

        auto manual_pose_input = getInput<geometry_msgs::msg::PoseStamped>("manual_target_pose");
        auto attack_pose_input = getInput<geometry_msgs::msg::PoseStamped>("attack_target_pose");
        auto attack_valid_input = getInput<bool>("attack_target_valid");

        setOutput("final_target_point", normal_point_input.value());
        setOutput("final_target_source", 1);

        if (manual_pose_input)
        {
            setOutput("final_target_pose", manual_pose_input.value());
            setOutput("final_target_source", 2);
            return BT::NodeStatus::SUCCESS;
        }

        if (attack_valid_input && attack_valid_input.value() && attack_pose_input)
        {
            setOutput("final_target_pose", attack_pose_input.value());
            setOutput("final_target_source", 3);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::SUCCESS;
    }
};

// 动作节点：发布导航目标点（根据点编号查坐标以及是否追击进行追击）
// 注意：该类通过黑板获取 ROS 节点指针，因此可以直接用 registerNodeType 注册
// todo 根据当前位置去判断是否需要进行导航，以及限制追击的范围
class PublishNavGoal : public BT::SyncActionNode
{
public:
    PublishNavGoal(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config), node_(nullptr), pub_created_(false) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("final_target_point", "Final target point number"),
                BT::InputPort<int>("final_target_source", "Final target source"),
                BT::InputPort<geometry_msgs::msg::PoseStamped>("final_target_pose", "Final target pose")};
    }

    BT::NodeStatus tick() override
    {
        if (!pub_created_)
        {
            if (!config().blackboard->get("ros_node", node_))
                throw std::runtime_error("ROS node not found in blackboard");
            target_pub_ = node_->create_publisher<algo_master::msg::PLC2Target>("plc2target", 10);
            target_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
                "sentry_bt_target_markers", 10);
            pub_created_ = true;
        }
    
        algo_master::msg::PLC2Target plc2target;
        auto final_target_point_input = getInput<int>("final_target_point");
        auto final_target_source_input = getInput<int>("final_target_source");
        auto final_target_pose_input = getInput<geometry_msgs::msg::PoseStamped>("final_target_pose");

        const int final_source = final_target_source_input ? final_target_source_input.value() : 1;
        const int final_point = final_target_point_input ? final_target_point_input.value() : 0;

        if (final_source == 2 || final_source == 3)
        {
            if (!final_target_pose_input)
            {
                RCLCPP_ERROR(node_->get_logger(), "Missing final_target_pose input");
                return BT::NodeStatus::FAILURE;
            }
            const auto &pose = final_target_pose_input.value();
            if (final_source == 2)
            {
                const auto relative_target = ToInitRelative(
                    static_cast<float>(pose.pose.position.x),
                    static_cast<float>(pose.pose.position.y));
                plc2target.target_x = relative_target.first;
                plc2target.target_y = relative_target.second;
                plc2target.if_on_attack = 0;
                RCLCPP_INFO(
                    node_->get_logger(),
                    "decision target: MANUAL source=%d id=%d global=(%.3f, %.3f) relative=(%.3f, %.3f)",
                    final_source,
                    final_point,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    plc2target.target_x,
                    plc2target.target_y);
            }
            else
            {
                plc2target.target_x = pose.pose.position.x;
                plc2target.target_y = pose.pose.position.y;
                plc2target.if_on_attack = 1;
                RCLCPP_INFO(
                    node_->get_logger(),
                    "decision target: ATTACK source=%d id=%d target=(%.3f, %.3f)",
                    final_source,
                    final_point,
                    pose.pose.position.x,
                    pose.pose.position.y);
            }
        }
        else
        {
            std::pair<float, float> coord;
            if (!ResolveDecisionPoint(final_point, coord))
            {
                RCLCPP_ERROR(node_->get_logger(), "Unknown point: %d", final_point);
                return BT::NodeStatus::FAILURE;
            }
            const auto relative_target = ToInitRelative(coord.first, coord.second);
            plc2target.target_x = relative_target.first;
            plc2target.target_y = relative_target.second;
            plc2target.if_on_attack = 0;
            RCLCPP_INFO(
                node_->get_logger(),
                "decision target: NORMAL source=%d id=%d name=%s global=(%.3f, %.3f) relative=(%.3f, %.3f)",
                final_source,
                final_point,
                DecisionPointName(final_point),
                coord.first,
                coord.second,
                plc2target.target_x,
                plc2target.target_y);
        }

        publishTargetMarkers(final_point, final_source, plc2target, final_target_pose_input);
    
        target_pub_->publish(plc2target);
        return BT::NodeStatus::SUCCESS;
    }

private:
    void publishTargetMarkers(
        int final_point,
        int final_source,
        const algo_master::msg::PLC2Target & plc2target,
        const BT::Expected<geometry_msgs::msg::PoseStamped> & final_target_pose_input)
    {
        if (!target_marker_pub_) {
            return;
        }

        visualization_msgs::msg::MarkerArray markers;

        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "map";
        sphere.header.stamp = node_->now();
        sphere.ns = "sentry_bt_target";
        sphere.id = 0;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.scale.x = 0.45;
        sphere.scale.y = 0.45;
        sphere.scale.z = 0.45;
        sphere.color.a = 0.95;

        if (final_source == 3) {
            sphere.color.r = 1.0;
            sphere.color.g = 0.2;
            sphere.color.b = 0.2;
        } else if (final_source == 2) {
            sphere.color.r = 1.0;
            sphere.color.g = 0.7;
            sphere.color.b = 0.1;
        } else {
            sphere.color.r = 0.1;
            sphere.color.g = 0.9;
            sphere.color.b = 0.2;
        }

        if (final_source == 2 || final_source == 3) {
            if (final_target_pose_input) {
                sphere.pose = final_target_pose_input.value().pose;
                if (final_source == 2) {
                    const auto relative_target = ToInitRelative(
                        static_cast<float>(sphere.pose.position.x),
                        static_cast<float>(sphere.pose.position.y));
                    sphere.pose.position.x = relative_target.first;
                    sphere.pose.position.y = relative_target.second;
                }
            }
        } else {
            std::pair<float, float> coord;
            if (ResolveDecisionPoint(final_point, coord)) {
                const auto relative_target = ToInitRelative(coord.first, coord.second);
                sphere.pose.position.x = relative_target.first;
                sphere.pose.position.y = relative_target.second;
                sphere.pose.position.z = 0.2;
                sphere.pose.orientation.w = 1.0;
            }
        }
        markers.markers.push_back(sphere);

        visualization_msgs::msg::Marker text;
        text.header = sphere.header;
        text.ns = "sentry_bt_target_label";
        text.id = 1;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose = sphere.pose;
        text.pose.position.z += 0.7;
        text.scale.z = 0.45;
        text.color.a = 1.0;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.text = std::string("id=") + std::to_string(final_point) +
                    " src=" + std::to_string(final_source) +
                    " rel=(" + std::to_string(plc2target.target_x) + ", " + std::to_string(plc2target.target_y) + ")";
        markers.markers.push_back(text);

        target_marker_pub_->publish(markers);
    }

    rclcpp::Node::SharedPtr node_;
    // rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
    rclcpp::Publisher<algo_master::msg::PLC2Target>::SharedPtr target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr target_marker_pub_;
    bool pub_created_;
};
// 1为进攻姿态，2为
// 防御姿态，3为移动姿态，默认为3
class SentryAttitudeSwitch : public BT::SyncActionNode
{
public:
    SentryAttitudeSwitch(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config), node_(nullptr), pub_created_(false), last_attitude_(3) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("sentry_attitude", "姿态值：1进攻姿态，2防御姿态，3移动姿态") };
    }

    BT::NodeStatus tick() override
    {
        // 延迟创建发布者（确保黑板中有 ros_node）
        if (!pub_created_)
        {
            if (!config().blackboard->get("ros_node", node_))
                throw std::runtime_error("ROS node not found in blackboard");
            attitude_pub_ = node_->create_publisher<sentry_decision_msg::msg::AttitudeSwitch>(
                "attitude_switch", 10);
            pub_created_ = true;
        }

        // 读取输入端口
        auto attitude_input = getInput<int>("sentry_attitude");
        if (!attitude_input)
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing sentry_attitude input");
            return BT::NodeStatus::FAILURE;
        }

        const auto now = node_->now();
        if (attitude_input.value() == last_attitude_ && last_switch_time_.nanoseconds() > 0)
        {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "attitude switch skipped: desired=%d same_as_last=%d",
                attitude_input.value(), last_attitude_);
            return BT::NodeStatus::SUCCESS;
        }
        if (last_switch_time_.nanoseconds() > 0 && (now - last_switch_time_).seconds() < 5.0)
        {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "attitude switch skipped: desired=%d cooldown_remaining=%.2f",
                attitude_input.value(),
                5.0 - (now - last_switch_time_).seconds());
            return BT::NodeStatus::SUCCESS;
        }

        // 构造消息（仅填充姿态字段，其余保持默认0）
        sentry_decision_msg::msg::AttitudeSwitch msg;
        msg.sentry_attitude_switch = attitude_input.value();
        // 其他字段已默认初始化为0，显式赋值以明确意图
        msg.sentry_if_confirm_alive = 0;
        msg.sentry_if_immediate_alive = 0;
        msg.exchange_projectile_num = 0;
        msg.remote_exchange_projectile_count = 0;
        msg.remote_exchange_hp_count = 0;
        msg.sentry_confirm_big_buff = 0;

        attitude_pub_->publish(msg);
        last_attitude_ = msg.sentry_attitude_switch;
        last_switch_time_ = now;
        RCLCPP_INFO(node_->get_logger(), "已发布姿态切换：%d", msg.sentry_attitude_switch);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sentry_decision_msg::msg::AttitudeSwitch>::SharedPtr attitude_pub_;
    bool pub_created_;
    int last_attitude_;
    rclcpp::Time last_switch_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_options = rclcpp::NodeOptions().use_global_arguments(true);
    auto bt_node = std::make_shared<rclcpp::Node>("sentry_bt_node", node_options);
    // 参数
    const std::string bt_xml_param = bt_node->declare_parameter<std::string>("bt_xml", "");
    const double tick_hz = bt_node->declare_parameter<double>("tick_hz", 1.0);
    const bool use_groot = bt_node->declare_parameter<bool>("use_groot", true);
    const int groot_port = bt_node->declare_parameter<int>("groot_port", 1666);
    LoadPointCoords(bt_node);
    // 创建黑板并初始化默认值（可选）
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    blackboard->set("if_hp_less_50", false);
    blackboard->set("if_get_allow_17", false);
    blackboard->set("if_3s_not_hurted", false);
    blackboard->set("if_hp_less_100", false);
    blackboard->set("if_need_hp_recover", false);
    blackboard->set("if_allowance_less_50", false);
    blackboard->set("if_5s_not_hurted", false);
    blackboard->set("if_arrived", true);
    blackboard->set("last_point", false);
    blackboard->set("if_need_to_attack", false);
    blackboard->set("if_attack_target_type_allowed", false);
    blackboard->set("if_match_started", false);
    blackboard->set("if_enemy_outpost_alive", false);
    blackboard->set("if_radar_outpost_target", false);
    blackboard->set("if_force_enemy_outpost", false);
    blackboard->set("if_base_full_hp", false);
    blackboard->set("if_base_low_hp", false);
    blackboard->set("if_energy_below_15", false);
    blackboard->set("if_can_rebuild_outpost", false);
    blackboard->set("if_manual_target_valid", false);
    blackboard->set("if_recently_hurt", false);
    blackboard->set("if_target_far", true);
    blackboard->set("desired_sentry_attitude", 3);
    blackboard->set("attack_attitude_time", 0);
    blackboard->set("defense_attitude_time", 0);
    blackboard->set("move_attitude_time", 0);
    blackboard->set("attack_attitude_weakened", false);
    blackboard->set("defense_attitude_weakened", false);
    blackboard->set("move_attitude_weakened", false);
    blackboard->set("attack_attitude_score", 0);
    blackboard->set("defense_attitude_score", 0);
    blackboard->set("move_attitude_score", 0);
    blackboard->set("game_state", 0);
    blackboard->set("game_remain_time", 0);
    blackboard->set("if_get_manual_msg", false);
    blackboard->set("if_get_radar_msg", false);
    blackboard->set("projectile_allowance_17mm", 0);
    blackboard->set("current_shoot_heat_17mm", 0);
    blackboard->set("heat_limit_17mm", 0);
    blackboard->set("heat_cool_rate_17mm", 0);
    blackboard->set("allow_to_get_17mm", 0);
    blackboard->set("already_allowance_17", 0);
    blackboard->set("available_allowance_17", 0);
    blackboard->set("allowance_remain_time", 0);
    blackboard->set("current_hp", 0);
    blackboard->set("my_base_hp", 0);
    blackboard->set("we_outpost_hp", 0);
    blackboard->set("enemy_outpost_hp", 0);
    blackboard->set("enemy_hero_x", 0);
    blackboard->set("enemy_hero_y", 0);
    blackboard->set("remaining_energy_flags", 0);
    blackboard->set("enemy_armor_id", 0);
    blackboard->set("real_sentry_attitude_switch", 3);
    blackboard->set("manual_target_pose", geometry_msgs::msg::PoseStamped());
     
    
    // 可继续初始化其他标志...

    // 启动黑板更新器（独立线程）RecoveryFallback
    auto updater = std::make_shared<BlackboardUpdater>(blackboard);
    std::thread updater_thread([updater]()
                               { rclcpp::spin(updater); });

    // 创建ROS节点用于行为树相关发布
    
    // 将 ROS 节点放入黑板，供 PublishNavGoal 使用
    blackboard->set("ros_node", bt_node);

    rclcpp::executors::SingleThreadedExecutor bt_executor;
    bt_executor.add_node(bt_node);
    std::thread bt_executor_thread([&bt_executor]()
                                   { bt_executor.spin(); });

    // 注册自定义节点
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SetTargetPoint>("SetTargetPoint");
    factory.registerNodeType<SetManualTarget>("SetManualTarget");
    factory.registerNodeType<ForwardDesiredAttitude>("ForwardDesiredAttitude");
    factory.registerNodeType<SelectFinalTarget>("SelectFinalTarget");
    factory.registerNodeType<PublishNavGoal>("PublishNavGoal"); // 直接注册，无需 builder
    factory.registerNodeType<SentryAttitudeSwitch>("SentryAttitudeSwitch");
    factory.registerBuilder<AntiAutoAim>("AntiAutoAim",
                                         [bt_node](const std::string &name, const BT::NodeConfig &config)
                                         {
                                             return std::make_unique<AntiAutoAim>(name, config, bt_node);
                                         });

    // 获取 XML 文件路径
    std::string pkg_share = ament_index_cpp::get_package_share_directory("sentry_bt");
    std::string xml_path = bt_xml_param.empty() ? (pkg_share + "/config/sentry_bt.xml") : bt_xml_param;

    // 加载行为树
    auto tree = factory.createTreeFromFile(xml_path, blackboard);
    std::unique_ptr<BT::Groot2Publisher> publisher;
    if (use_groot) {
        publisher = std::make_unique<BT::Groot2Publisher>(tree, static_cast<uint16_t>(groot_port));
    }

    // 以 10Hz tick 树
    rclcpp::Rate rate(tick_hz);
    while (rclcpp::ok())
    {
        tree.tickOnce();
        rate.sleep();
    }

    rclcpp::shutdown();
    bt_executor.cancel();
    if (bt_executor_thread.joinable()) {
        bt_executor_thread.join();
    }
    updater_thread.join();
    return 0;
}

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

static std::map<DecisionPoint, std::pair<float, float>> point_coords = {
    {INIT_PACK_POINT, {0.5, -0.8}},
    {WE_DEPOT_POINT, {1.0, 1.0}},
    {ENEMY_OUTPOST_POINT, {1.0, 1.0}},
    {WE_OUTPOST_POINT, {1.0, 1.0}},
    {ENEMY_FORTRESS_POINT, {1.0, 1.0}},
    {MANUAL_POINT, {1.0, 1.0}},
    {ENEMY_FLYING_POINT, {1.0, 1.0}},
    {WE_PROTECT_POINT, {2.0, 2.0}}
    // ????????????
};

static void LoadPointCoords(const rclcpp::Node::SharedPtr &node)
{
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
            point_coords[key] = {static_cast<float>(value[0]), static_cast<float>(value[1])};
        }
    }
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

// 条件节点：检查黑板中的布尔标志
class CheckBool : public BT::ConditionNode
{
public:
    CheckBool(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("flag_name", "Name of the boolean flag in blackboard"),
            BT::InputPort<bool>("expected", true, "Expected value (true/false)")};
    }

    BT::NodeStatus tick() override
    {
        std::string flag_name;
        bool expected;
        if (!getInput("flag_name", flag_name) || !getInput("expected", expected))
            return BT::NodeStatus::FAILURE;

        bool value;
        if (config().blackboard->get(flag_name, value))
            return (value == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        return BT::NodeStatus::FAILURE;
    }
};

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
        return {BT::InputPort<int>("decision_point_port", "Target point number"),
                BT::InputPort<int>("flag_name", "if_need_to_attack"),
                BT::InputPort<geometry_msgs::msg::PoseStamped>("attack_point_port", "attack point port")};
    }

    BT::NodeStatus tick() override
    {
        if (!pub_created_)
        {
            if (!config().blackboard->get("ros_node", node_))
                throw std::runtime_error("ROS node not found in blackboard");
            target_pub_ = node_->create_publisher<algo_master::msg::PLC2Target>("plc2target", 10);
            pub_created_ = true;
        }
    
        // 1. 获取攻击标志
        auto flag_input = getInput<int>("flag_name");
        if (!flag_input)
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing flag_name input");
            return BT::NodeStatus::FAILURE;
        }
        int if_need_to_attack_msg = flag_input.value();
    
        algo_master::msg::PLC2Target plc2target;
    
        if (if_need_to_attack_msg)
        {
            // 2. 攻击模式：必须存在 attack_point_port
            auto attack_pose_input = getInput<geometry_msgs::msg::PoseStamped>("attack_point_port");
            if (!attack_pose_input)
            {
                RCLCPP_ERROR(node_->get_logger(), "Missing attack_point_port input");
                auto point_input = getInput<int>("decision_point_port");
                if (!point_input)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Missing decision_point_port input");
                    return BT::NodeStatus::FAILURE;
                }
                int point = point_input.value();
                std::pair<float, float> coord;
                if (!ResolveDecisionPoint(point, coord))
                {
                    RCLCPP_ERROR(node_->get_logger(), "Unknown point: %d", point);
                    return BT::NodeStatus::FAILURE;
                }
                plc2target.target_x = coord.first;
                plc2target.target_y = coord.second;

                // return BT::NodeStatus::FAILURE;
            }
            else 
            {

                RCLCPP_INFO(node_->get_logger(), "attack point:");
                const auto& attack_pose = attack_pose_input.value();
                plc2target.target_x = attack_pose.pose.position.x;
              plc2target.target_y = attack_pose.pose.position.y;
            }
            
        }
        else
        {
            // 3. 非攻击模式：必须存在 decision_point_port
            auto point_input = getInput<int>("decision_point_port");
            if (!point_input)
            {
                // RCLCPP_ERROR(node_->get_logger(), "Missing decision_point_port input");
                return BT::NodeStatus::FAILURE;
            }
            int point = point_input.value();
            std::pair<float, float> coord;
            if (!ResolveDecisionPoint(point, coord))
            {
                RCLCPP_ERROR(node_->get_logger(), "Unknown point: %d", point);
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(node_->get_logger(), "decison point:");
            plc2target.target_x = coord.first;
            plc2target.target_y = coord.second;
    
        }
    
        target_pub_->publish(plc2target);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    // rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
    rclcpp::Publisher<algo_master::msg::PLC2Target>::SharedPtr target_pub_;
    bool pub_created_;
};
// 1为进攻姿态，2为
// 防御姿态，3为移动姿态，默认为3
class SentryAttitudeSwitch : public BT::SyncActionNode
{
public:
    SentryAttitudeSwitch(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config), node_(nullptr), pub_created_(false) {}

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
        RCLCPP_INFO(node_->get_logger(), "已发布姿态切换：%d", msg.sentry_attitude_switch);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sentry_decision_msg::msg::AttitudeSwitch>::SharedPtr attitude_pub_;
    bool pub_created_;
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
    blackboard->set("if_hp_less_200", false);
    blackboard->set("if_3s_not_hurted", false);
    blackboard->set("if_hp_less_100", false);
    blackboard->set("if_outpost_destroyed", false);
    blackboard->set("if_enemy_outpost_destroyed", false);
    blackboard->set("if_allowance_less_50", false);
    blackboard->set("if_5s_not_found", false);
    blackboard->set("if_5s_not_hurted", false);
     blackboard->set("if_hp_recover", true);
    blackboard->set("if_arrived", true);
    blackboard->set("last_point", false);
    blackboard->set("if_need_to_attack", 0);
    blackboard->set("game_state", 0);
    blackboard->set("game_remain_time", 0);
    blackboard->set("projectile_allowance_17mm", 0);
    blackboard->set("current_hp", 0);
    blackboard->set("my_base_hp", 0);
    blackboard->set("enemy_hero_x", 0);
    blackboard->set("enemy_hero_y", 0);
     
    
    // 可继续初始化其他标志...

    // 启动黑板更新器（独立线程）
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
    factory.registerNodeType<CheckBool>("CheckBool");
    factory.registerNodeType<SetTargetPoint>("SetTargetPoint");
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

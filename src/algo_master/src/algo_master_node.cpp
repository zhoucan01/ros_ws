
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "algo_master/constants.hpp"
#include "algo_master/serialport.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "algo_master/msg/plc2_imu.hpp"
#include "algo_master/msg/plc2_target.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // For Navigation2 goal
#include <nav2_msgs/action/navigate_to_pose.hpp> // For Navigation2 action client
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp> // For subscribing to odometry
#include <nav_msgs/msg/path.hpp> 

#include "sentry_decision_msg/msg/sentry_decision.hpp"
#include <sentry_decision_msg/msg/enemy_pos.hpp>
#include <sentry_decision_msg/msg/referee_raw.hpp>
#include <sentry_decision_msg/msg/tunnel_monitor.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <atomic>
#include <cmath>
// using namespace 

#define USE_SERIALPORT_RECV

class AlgoMasterNode : public rclcpp::Node
{
public:
    AlgoMasterNode() : Node("algo_master"), running_(true)
    {
        // 参数
        arrived_threshold_ = this->declare_parameter<double>("arrived_threshold", 0.3);
        close_threshold_ = this->declare_parameter<double>("close_threshold", 1.0);
        use_goal_pose_topic_ = this->declare_parameter<bool>("use_goal_pose_topic", true);
        use_action_goal_ = this->declare_parameter<bool>("use_action_goal", true);
        tunnel_monitor_topic_ = this->declare_parameter<std::string>("tunnel_monitor_topic", "tunnel_monitor");
        // 初始化配置
        if (!InitConfigs("src/algo_master/configs/serial_config.json")) {
            RCLCPP_ERROR(this->get_logger(), "Read serial_config.json failed");
            throw std::runtime_error("Config initialization failed");
        }

#ifdef USE_SERIALPORT_RECV
        // 初始化串口
        InitializeSerialPort();
        // 初始化tf监听器(实时位姿（/odom）转换至地图系)


        // 在构造函数中
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



        
        // 创建订阅
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                static u_int8_t buff_seq;
                buff_seq++;

                float cur_x = 0.0f;
                float cur_y = 0.0f;
                {
                    std::lock_guard<std::mutex> pose_lock(odom_mutex_);
                    cur_x = serial_cur_send_.cur_x;
                    cur_y = serial_cur_send_.cur_y;
                }

                std::lock_guard<std::mutex> lock(serial_mutex_);
                serial_send_.linear_vel_x = msg->linear.x;
                serial_send_.linear_vel_y = msg->linear.y;
                serial_send_.control_yaw_diff = msg->angular.z;
                serial_send_.cur_x = cur_x;
                serial_send_.cur_y = cur_y;
                serial_send_.has_path_ = has_path_;
                serial_send_.get_goal = get_goal;
                serial_send_.seq = buff_seq;

                serial_send_.arrive_flag = arrived_flag_;
                serial_send_.close_flag = close_flag_;
                serial_send_.need_tunnel = need_tunnel_.load() ? 1 : 0;
                serial_send_.tunnel_yaw_error = static_cast<float>(tunnel_yaw_error_.load());
                RCLCPP_INFO(this->get_logger(),"seq: = %.2d",buff_seq);

                // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.2f,linear.y=%.2f, angular.z=%.2f", 
                //            msg->linear.x, msg->linear.y, msg->angular.z);
                sp_.Send(serial_send_);
            });
#endif

        // 创建发布者
        imu_pub_ = this->create_publisher<algo_master::msg::PLC2Imu>("plc2imu", 10);
        // target_pub_ = this->create_publisher<algo_master::msg::PLC2Target>("plc2target", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);
        sentry_decision_pub_ = this->create_publisher<sentry_decision_msg::msg::SentryDecision>("decision_msg", 10);
        referee_raw_pub_ = this->create_publisher<sentry_decision_msg::msg::RefereeRaw>("referee_raw_msg", 10);
        enemy_pos_pub_ = this->create_publisher<sentry_decision_msg::msg::EnemyPos>("enemy_msg",10);
        cur_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("current_pos_msg",10);

        // 订阅 IMU 数据并转换
        imu_sub_ = this->create_subscription<algo_master::msg::PLC2Imu>(
            "plc2imu", 10,
            [this](const algo_master::msg::PLC2Imu::SharedPtr msg) {
                sensor_msgs::msg::JointState joint_state_msg;
                
                joint_state_msg.header.stamp = this->now();
                joint_state_msg.name = {"gimbal_yaw_joint", "gimbal_pitch_joint"};
                joint_state_msg.position = {
                    static_cast<double>(msg->imu_yaw * M_PI / 180.0),
                    static_cast<double>(msg->imu_pitch * M_PI / 180.0)
                };

                joint_state_pub_->publish(joint_state_msg);
            });

        auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "global_costmap/costmap",costmap_qos,
            [this](const nav_msgs::msg::OccupancyGrid msg)
            { 
                // 目前仅保存 frame_id，后续如果不用可直接删除此订阅
                global_costmap.header.frame_id = msg.header.frame_id;
            }
        );
        // 订阅 plc2target 并将其发送给 Navigation2
        plc2target_sub_ = this->create_subscription<algo_master::msg::PLC2Target>(
            "plc2target", 10,
            [this](const algo_master::msg::PLC2Target::SharedPtr msg) {
                geometry_msgs::msg::PoseStamped goal_pose;
                goal_pose.header.stamp = this->now();
                goal_pose.header.frame_id = "map"; // 确保 frame_id 正确
                goal_pose.pose.position.x = msg->target_x;
                goal_pose.pose.position.y = msg->target_y;
                goal_pose.pose.orientation.w = 1.0; // 默认朝向（无旋转）

                float cur_x = 0.0f;
                float cur_y = 0.0f;
                {
                    std::lock_guard<std::mutex> pose_lock(odom_mutex_);
                    cur_x = serial_cur_send_.cur_x;
                    cur_y = serial_cur_send_.cur_y;
                }
                const double dx = msg->target_x - cur_x;
                const double dy = msg->target_y - cur_y;
                const double dist = std::hypot(dx, dy);

                arrived_flag_ = (dist < arrived_threshold_);
                close_flag_ = (dist < close_threshold_);

                //到 Navigation2 的目标话题
                if (use_goal_pose_topic_) {
                    nav_goal_pub_->publish(goal_pose);
                }

                // // 启动 Navigation2 的 NavigateToPose 行为
                if (use_action_goal_) {
                    SendGoalToNav2(goal_pose);
                }
            });

        // 订阅 Navigation2 的实时位姿（/odom）
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                geometry_msgs::msg::PointStamped odom_point;
                odom_point.header = msg->header; // 这里 header.frame_id 应为 "odom"
                geometry_msgs::msg::PoseStamped odom_pose;
                odom_pose.header = msg->header;
                odom_point.point = msg->pose.pose.position;
                odom_pose.pose = msg->pose.pose;
                odom_point.header.stamp = msg->header.stamp; // 时间戳必须用消息的，不能用 now()
                odom_point.header.frame_id = "odom";         // 或者 "/odom"

                odom_pose.header.frame_id = "odom";
                

                // RCLCPP_INFO(this->get_logger(), "Odom Msg Frame: [%s]", msg->header.frame_id.c_str());
                try
                {



                    // RCLCPP_INFO(this->get_logger(), "--- Current TF Buffer Status ---\n%s", 
                    // tf_buffer_->allFramesAsString().c_str());
                    // 2. 转换到 map 坐标系
                    geometry_msgs::msg::PointStamped map_point;

                    

                    map_point = tf_buffer_->transform(odom_point, "map", tf2::durationFromSec(0.2));
                    geometry_msgs::msg::PoseStamped map_pose;
                    odom_pose.pose = msg->pose.pose;
                    map_pose = tf_buffer_->transform(odom_pose, "map", tf2::durationFromSec(0.2));
                    // map_point = 
                    tf2::Quaternion q(
                        map_pose.pose.orientation.x,
                        map_pose.pose.orientation.y,
                        map_pose.pose.orientation.z,
                        map_pose.pose.orientation.w);

                    tf2::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);
                    current_map_yaw_.store(yaw, std::memory_order_relaxed);
                    
                    RCLCPP_INFO(this->get_logger(),"current_map_yaw: = %f", yaw);
                    // RCLCPP_INFO(this->get_logger(),current_map_yaw);
                    // 3. 更新串口发送数据（需要保护）
                    {
                        std::lock_guard<std::mutex> lock(odom_mutex_);
                        serial_cur_send_.cur_x = map_point.point.x;
                        serial_cur_send_.cur_y = map_point.point.y;
                    }

                    cur_pos_pub_->publish(map_point);

                    // RCLCPP_INFO(this->get_logger(), "Map Pose: x=%.2f, y=%.2f",
                    //             map_point.point.x, map_point.point.y);
                }
                catch (const tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
                }
            });
        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/red_standard_robot1/odometry", 10,
        //     [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        //         std::lock_guard<std::mutex> lock(odom_mutex_);
        //         serial_cur_send_.cur_x = msg->pose.pose.position.x; // 更新当前位姿
        //         serial_cur_send_.cur_y = msg->pose.pose.position.y;

        //         geometry_msgs::msg::PointStamped map_point;
        //         map_point.header = msg->header;
        //         map_point.point = msg->pose.pose.position;
        //         cur_pos_pub_->publish(map_point);
        //         RCLCPP_INFO(this->get_logger(), "Current Pose: x=%.2f, y=%.2f",
        //             serial_cur_send_.cur_x,
        //             serial_cur_send_.cur_y
        //                     );
        //     });


        // 订阅 Navigation2 的全局路径（/plan）
        global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                if (msg->poses.empty()) {
                    has_path_ = false;
                    RCLCPP_INFO(this->get_logger(), "No path available.");
                } else {
                    has_path_ = true;
                    RCLCPP_INFO(this->get_logger(), "Path detected with %zu points.", msg->poses.size());
                }
            });

        tunnel_monitor_sub_ = this->create_subscription<sentry_decision_msg::msg::TunnelMonitor>(
            tunnel_monitor_topic_, 10,
            [this](const sentry_decision_msg::msg::TunnelMonitor::SharedPtr msg) {
                constexpr int kApproachingTunnel = 2;
                constexpr int kInTunnel = 3;
                constexpr int kRecoveryActive = 5;
                need_tunnel_ = msg->tunnel_status == kApproachingTunnel ||
                               msg->tunnel_status == kInTunnel ||
                               msg->tunnel_status == kRecoveryActive;
                tunnel_yaw_error_ = msg->tunnel_yaw_error;

                std::lock_guard<std::mutex> lock(serial_mutex_);
                serial_send_.need_tunnel = need_tunnel_.load() ? 1 : 0;
                serial_send_.tunnel_yaw_error = static_cast<float>(tunnel_yaw_error_.load());
            });

        // 创建发布者以发送 Navigation2 的目标
        nav_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

        // 初始化 Navigation2 的 Action 客户端
        nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        // 启动处理线程
        processing_thread_ = std::thread(&AlgoMasterNode::ProcessingLoop, this);
    }

    ~AlgoMasterNode() {
        running_ = false;
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
#ifdef USE_SERIALPORT_RECV
        if (serial_recv_producer_.joinable()) serial_recv_producer_.join();
        if (serial_recv_consumer_.joinable()) serial_recv_consumer_.join();
        if (serial_daemon_.joinable()) serial_daemon_.join();
#endif
    }

private:
    void SendGoalToNav2(const geometry_msgs::msg::PoseStamped &goal_pose) {
        using namespace std::placeholders;

        if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&AlgoMasterNode::GoalResponseCallback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&AlgoMasterNode::FeedbackCallback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&AlgoMasterNode::ResultCallback, this, _1);

        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void GoalResponseCallback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            
        }
    }

    void FeedbackCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);


    }

    void ResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }

#ifdef USE_SERIALPORT_RECV
    void InitializeSerialPort() {
        if (!sp_.OpenPort(serialPortConfig.portName, serialPortConfig.baudrate,
                         serialPortConfig.parity, serialPortConfig.dataBit, 
                         serialPortConfig.stopBit, serialPortConfig.synchronize)) {
            RCLCPP_ERROR(this->get_logger(), "Cannot Open Serial Port %s",
                        serialPortConfig.portName.c_str());
            throw std::runtime_error("Serial port initialization failed");
        }

        serial_recv_producer_ = std::thread(&NautilusSerialPort::ReadRawBuf, &sp_);
        serial_recv_consumer_ = std::thread(&NautilusSerialPort::ProcRawBuf, &sp_);
        serial_daemon_ = std::thread(&NautilusSerialPort::CheckAndReconnect, &sp_);
    }
#endif

    void ProcessingLoop() {
        while (rclcpp::ok() && running_) {
            bool got_any = false;
            if (msgSerialNavRecv.Pop(nav_serial_recv_)) {
                got_any = true;
                // target_pub_->publish(PLCNavRecv2TargetMsg(nav_serial_recv_));
                imu_pub_->publish(PLCNavRecv2ImuMsg(nav_serial_recv_));
                const double current_map_yaw = current_map_yaw_.load(std::memory_order_relaxed);
                enemy_pos_pub_->publish(PLCNavRecv2EnemyMsg(nav_serial_recv_, current_map_yaw));
            }

            if (msgSerialDecisionRecv.Pop(decision_serial_recv_)) {
                got_any = true;
                sentry_decision_pub_->publish(PLCDecisionRecv2DecisionMsg(decision_serial_recv_));
                referee_raw_pub_->publish(PLCDecisionRecv2RefereeRawMsg(decision_serial_recv_));
            }

            if (!got_any) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            
        }
    }

    // 成员变量
    NavSerialMsg nav_serial_recv_;
    DecisionSerialMsg decision_serial_recv_;
    NavigationPLCSendMsg serial_send_;
    NavigationPLCSendMsg serial_cur_send_;
    nav_msgs::msg::OccupancyGrid global_costmap;
    NautilusSerialPort sp_;
    std::atomic<bool> running_;
    std::mutex serial_mutex_;
    std::atomic<bool> get_goal{false};
    std::atomic<bool> arrived_flag_{false};
    std::atomic<bool> close_flag_{false};
    // 成员变量
    geometry_msgs::msg::Pose current_pose_;
    

    std::atomic<bool> has_path_{false};
    std::mutex odom_mutex_;
    std::mutex plan_mutex_;

    double arrived_threshold_{0.3};
    double close_threshold_{1.0};
    bool use_goal_pose_topic_{true};
    bool use_action_goal_{true};
    std::string tunnel_monitor_topic_{"tunnel_monitor"};
    std::atomic<bool> need_tunnel_{false};
    std::atomic<double> tunnel_yaw_error_{0.0};
 


    // 线程
    std::thread processing_thread_;
#ifdef USE_SERIALPORT_RECV
    std::thread serial_recv_producer_;
    std::thread serial_recv_consumer_;
    std::thread serial_daemon_;
#endif

    // ROS2接口
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<algo_master::msg::PLC2Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<algo_master::msg::PLC2Target>::SharedPtr target_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    rclcpp::Publisher<sentry_decision_msg::msg::SentryDecision>::SharedPtr sentry_decision_pub_;
    rclcpp::Publisher<sentry_decision_msg::msg::RefereeRaw>::SharedPtr referee_raw_pub_;
    rclcpp::Publisher<sentry_decision_msg::msg::EnemyPos>::SharedPtr enemy_pos_pub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cur_pos_pub_;

    // cur_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("current_pos_msg",10);
    rclcpp::Subscription<algo_master::msg::PLC2Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<algo_master::msg::PLC2Target>::SharedPtr plc2target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
     // ROS2接口
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::TunnelMonitor>::SharedPtr tunnel_monitor_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr test_tf_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

    // Navigation2 Action 客户端
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;

    std::atomic<double> current_map_yaw_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/red_standard_robot1"});

    // 创建节点（节点名称为 "algo_master"，命名空间为 /red_standard_robot1）
    auto node = std::make_shared<AlgoMasterNode>();  // 注意构造函数需要接受 options

    // auto node = std::make_shared<AlgoMasterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

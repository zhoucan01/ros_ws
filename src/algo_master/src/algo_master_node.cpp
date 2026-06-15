
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "algo_master/constants.hpp"
#include "algo_master/serialport.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "algo_master/msg/plc2_imu.hpp"
#include "algo_master/msg/plc2_target.hpp"
#include "algo_master/msg/wheel_raw.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // For Navigation2 goal
#include <nav2_msgs/action/navigate_to_pose.hpp> // For Navigation2 action client
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp> // For subscribing to odometry
#include <nav_msgs/msg/path.hpp> 
#include <std_msgs/msg/bool.hpp>

#include <sentry_decision_msg/msg/attitude_switch.hpp>
#include <sentry_decision_msg/msg/enemy_pos.hpp>
#include <sentry_decision_msg/msg/manual_pos.hpp>
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
        // åæ°
        arrived_threshold_ = this->declare_parameter<double>("arrived_threshold", 0.3);
        close_threshold_ = this->declare_parameter<double>("close_threshold", 1.0);
        use_goal_pose_topic_ = this->declare_parameter<bool>("use_goal_pose_topic", true);
        use_action_goal_ = this->declare_parameter<bool>("use_action_goal", true);
        tunnel_monitor_topic_ = this->declare_parameter<std::string>("tunnel_monitor_topic", "tunnel_monitor");
        // åå§åéç½?        if (!InitConfigs("src/algo_master/configs/serial_config.json")) {
            RCLCPP_ERROR(this->get_logger(), "Read serial_config.json failed");
            throw std::runtime_error("Config initialization failed");
        }

#ifdef USE_SERIALPORT_RECV
        // åå§åä¸²å?        InitializeSerialPort();
        // åå§åtfçå¬å?å®æ¶ä½å§¿ï¼?odomï¼è½¬æ¢è³å°å¾ç³?


        // å¨æé å½æ°ä¸­
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



        
        // åå»ºè®¢é
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
                const double current_map_yaw = current_map_yaw_.load(std::memory_order_relaxed);
                const double tunnel_target_yaw = tunnel_target_yaw_.load(std::memory_order_relaxed);
                const double tunnel_yaw_error = std::atan2(
                    std::sin(tunnel_target_yaw - current_map_yaw),
                    std::cos(tunnel_target_yaw - current_map_yaw));
                tunnel_yaw_error_.store(tunnel_yaw_error, std::memory_order_relaxed);
                serial_send_.current_map_yaw = static_cast<float>(current_map_yaw);
                serial_send_.tunnel_yaw_error = static_cast<float>(tunnel_yaw_error);
                serial_send_.if_on_attack = if_on_attack_.load() ? 1 : 0;
                serial_send_.sentry_attitude_switch = static_cast<uint8_t>(sentry_attitude_switch_.load());
                serial_send_.m_FrameTail = 0xAA;
                // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.2f,linear.y=%.2f, angular.z=%.2f", 
                //            msg->linear.x, msg->linear.y, msg->angular.z);
                sp_.Send(serial_send_);
            });
#endif

        // åå»ºåå¸è?        imu_pub_ = this->create_publisher<algo_master::msg::PLC2Imu>("plc2imu", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);
        referee_raw_pub_ = this->create_publisher<sentry_decision_msg::msg::RefereeRaw>("referee_raw_msg", 10);
        enemy_pos_pub_ = this->create_publisher<sentry_decision_msg::msg::EnemyPos>("enemy_msg",10);
        manual_pos_pub_ = this->create_publisher<sentry_decision_msg::msg::ManualPos>("manual_pos_msg", 10);
        cur_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("current_pos_msg",10);
        wheel_raw_pub_ = this->create_publisher<algo_master::msg::WheelRaw>("wheel_raw", 10);
        arrived_pub_ = this->create_publisher<std_msgs::msg::Bool>("if_arrived", 10);
        wheel_odom_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/wheel_odom", 10);

        // è®¢é IMU æ°æ®å¹¶è½¬æ?        imu_sub_ = this->create_subscription<algo_master::msg::PLC2Imu>(
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
                // ç®åä»ä¿å­?frame_idï¼åç»­å¦æä¸ç¨å¯ç´æ¥å é¤æ­¤è®¢é?                global_costmap.header.frame_id = msg.header.frame_id;
            }
        );
        // è®¢é plc2target å¹¶å°å¶åéç» Navigation2
        plc2target_sub_ = this->create_subscription<algo_master::msg::PLC2Target>(
            "plc2target", 10,
            [this](const algo_master::msg::PLC2Target::SharedPtr msg) {
                geometry_msgs::msg::PoseStamped goal_pose;
                goal_pose.header.stamp = this->now();
                goal_pose.header.frame_id = "map"; // ç¡®ä¿ frame_id æ­£ç¡®
                goal_pose.pose.position.x = msg->target_x;
                goal_pose.pose.position.y = msg->target_y;
                goal_pose.pose.orientation.w = 1.0; // é»è®¤æåï¼æ æè½¬ï¼?
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

                current_target_x_.store(msg->target_x, std::memory_order_relaxed);
                current_target_y_.store(msg->target_y, std::memory_order_relaxed);
                current_target_valid_.store(true);

                arrived_flag_ = (dist < arrived_threshold_);
                close_flag_ = (dist < close_threshold_);
                if_on_attack_ = msg->if_on_attack != 0;

                std_msgs::msg::Bool arrived_msg;
                arrived_msg.data = arrived_flag_.load();
                arrived_pub_->publish(arrived_msg);

                {
                    std::lock_guard<std::mutex> lock(serial_mutex_);
                    serial_send_.if_on_attack = if_on_attack_.load() ? 1 : 0;
                }

                //å?Navigation2 çç®æ è¯é¢?                if (use_goal_pose_topic_) {
                    nav_goal_pub_->publish(goal_pose);
                }

                // // å¯å¨ Navigation2 ç?NavigateToPose è¡ä¸º
                if (use_action_goal_) {
                    SendGoalToNav2(goal_pose);
                }
            });

        // è®¢é Navigation2 çå®æ¶ä½å§¿ï¼/odomï¼?        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                geometry_msgs::msg::PointStamped odom_point;
                odom_point.header = msg->header; // è¿é header.frame_id åºä¸º "odom"
                geometry_msgs::msg::PoseStamped odom_pose;
                odom_pose.header = msg->header;
                odom_point.point = msg->pose.pose.position;
                odom_pose.pose = msg->pose.pose;
                odom_point.header.stamp = msg->header.stamp; // æ¶é´æ³å¿é¡»ç¨æ¶æ¯çï¼ä¸è½ç?now()
                odom_point.header.frame_id = "odom";         // æè?"/odom"

                odom_pose.header.frame_id = "odom";
                

                // RCLCPP_INFO(this->get_logger(), "Odom Msg Frame: [%s]", msg->header.frame_id.c_str());
                try
                {



                    // RCLCPP_INFO(this->get_logger(), "--- Current TF Buffer Status ---\n%s", 
                    // tf_buffer_->allFramesAsString().c_str());
                    // 2. è½¬æ¢å?map åæ ç³?                    geometry_msgs::msg::PointStamped map_point;

                    

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

                    auto lookup_map_yaw = [this](const std::string & frame, double & out_yaw) -> bool {
                        try {
                            const auto tf = tf_buffer_->lookupTransform(
                                "map", frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
                            tf2::Quaternion q_tf(
                                tf.transform.rotation.x,
                                tf.transform.rotation.y,
                                tf.transform.rotation.z,
                                tf.transform.rotation.w);
                            tf2::Matrix3x3 m_tf(q_tf);
                            double r, p, y;
                            m_tf.getRPY(r, p, y);
                            out_yaw = y;
                            return true;
                        } catch (const tf2::TransformException &) {
                            return false;
                        }
                    };

                    double base_footprint_yaw = 0.0;
                    double gimbal_yaw_yaw = 0.0;
                    double gimbal_yaw_fake_yaw = 0.0;
                    const bool has_base_footprint_yaw = lookup_map_yaw("base_footprint", base_footprint_yaw);
                    const bool has_gimbal_yaw_yaw = lookup_map_yaw("gimbal_yaw", gimbal_yaw_yaw);
                    const bool has_gimbal_yaw_fake_yaw = lookup_map_yaw("gimbal_yaw_fake", gimbal_yaw_fake_yaw);

                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(), *this->get_clock(), 1000,
                        "yaw debug | odom->map: %.6f | base_footprint: %s%.6f | gimbal_yaw: %s%.6f | gimbal_yaw_fake: %s%.6f | tunnel_target_yaw: %.6f | tunnel_yaw_error(stored): %.6f",
                        yaw,
                        has_base_footprint_yaw ? "" : "N/A ", base_footprint_yaw,
                        has_gimbal_yaw_yaw ? "" : "N/A ", gimbal_yaw_yaw,
                        has_gimbal_yaw_fake_yaw ? "" : "N/A ", gimbal_yaw_fake_yaw,
                        tunnel_target_yaw_.load(std::memory_order_relaxed),
                        tunnel_yaw_error_.load(std::memory_order_relaxed));
                    // RCLCPP_INFO(this->get_logger(),current_map_yaw);
                    // 3. æ´æ°ä¸²å£åéæ°æ®ï¼éè¦ä¿æ¤ï¼
                    {
                        std::lock_guard<std::mutex> lock(odom_mutex_);
                        serial_cur_send_.cur_x = map_point.point.x;
                        serial_cur_send_.cur_y = map_point.point.y;
                    }

                    cur_pos_pub_->publish(map_point);

                    if (current_target_valid_.load()) {
                        const double dx = current_target_x_.load(std::memory_order_relaxed) - map_point.point.x;
                        const double dy = current_target_y_.load(std::memory_order_relaxed) - map_point.point.y;
                        const double dist = std::hypot(dx, dy);
                        arrived_flag_ = (dist < arrived_threshold_);
                        close_flag_ = (dist < close_threshold_);
                        std_msgs::msg::Bool arrived_msg;
                        arrived_msg.data = arrived_flag_.load();
                        arrived_pub_->publish(arrived_msg);
                    }

                    // RCLCPP_INFO(this->get_logger(), "Map Pose: x=%.2f, y=%.2f",
                    //             map_point.point.x, map_point.point.y);
                }
                catch (const tf2::TransformException &ex)
                {
                    const bool can_transform = tf_buffer_->canTransform(
                        "map", "odom", msg->header.stamp, tf2::durationFromSec(0.2));
                    RCLCPP_WARN(
                        this->get_logger(),
                        "TF transform failed: %s | odom stamp: %u.%u | now: %u.%u | odom frame: %s | canTransform(map<-odom): %s",
                        ex.what(),
                        msg->header.stamp.sec,
                        msg->header.stamp.nanosec,
                        this->now().seconds() >= 0 ? static_cast<unsigned>(this->now().seconds()) : 0U,
                        this->now().nanoseconds() >= 0 ? static_cast<unsigned>(this->now().nanoseconds() % 1000000000LL) : 0U,
                        msg->header.frame_id.c_str(),
                        can_transform ? "true" : "false");
                }
            });
        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/red_standard_robot1/odometry", 10,
        //     [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        //         std::lock_guard<std::mutex> lock(odom_mutex_);
        //         serial_cur_send_.cur_x = msg->pose.pose.position.x; // æ´æ°å½åä½å§¿
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


        // è®¢é Navigation2 çå¨å±è·¯å¾ï¼?planï¼?        global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                if (msg->poses.empty()) {
                    has_path_ = false;
                } else {
                    has_path_ = true;
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
                tunnel_target_yaw_ = msg->tunnel_target_yaw;
                tunnel_yaw_error_ = msg->tunnel_yaw_error;

                std::lock_guard<std::mutex> lock(serial_mutex_);
                serial_send_.need_tunnel = need_tunnel_.load() ? 1 : 0;
                serial_send_.current_map_yaw = static_cast<float>(current_map_yaw_.load(std::memory_order_relaxed));
                serial_send_.tunnel_yaw_error = static_cast<float>(tunnel_yaw_error_.load());
            });

        attitude_switch_sub_ = this->create_subscription<sentry_decision_msg::msg::AttitudeSwitch>(
            "attitude_switch", 10,
            [this](const sentry_decision_msg::msg::AttitudeSwitch::SharedPtr msg) {
                sentry_attitude_switch_ = msg->sentry_attitude_switch;
                std::lock_guard<std::mutex> lock(serial_mutex_);
                serial_send_.sentry_attitude_switch =
                    static_cast<uint8_t>(sentry_attitude_switch_.load());
            });

        // åå»ºåå¸èä»¥åé?Navigation2 çç®æ ?        nav_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

        // åå§å?Navigation2 ç?Action å®¢æ·ç«?        nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        // å¯å¨å¤ççº¿ç¨
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
        }
    }

    void FeedbackCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
        (void)feedback;
    }

    void ResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
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
                manual_pos_pub_->publish(PLCNavRecv2ManualPosMsg(nav_serial_recv_));
                imu_pub_->publish(PLCNavRecv2ImuMsg(nav_serial_recv_));
                const double current_map_yaw = current_map_yaw_.load(std::memory_order_relaxed);
                enemy_pos_pub_->publish(PLCNavRecv2EnemyMsg(nav_serial_recv_, current_map_yaw));


                /* Wheel odometry (already rotated to IMU frame by STM32) */
                {
                    geometry_msgs::msg::Twist twist_msg;
                    twist_msg.linear.x = nav_serial_recv_.vx_wheel / 10000.0;
                    twist_msg.linear.y = nav_serial_recv_.vy_wheel / 10000.0;
                    twist_msg.linear.z = 0.0;
                    twist_msg.angular.z = 0.0;
                    wheel_odom_pub_->publish(twist_msg);
                }
                /* Publish raw wheel speeds + gimbal yaw */
                {
                    algo_master::msg::WheelRaw raw_msg;
                    raw_msg.vlf = nav_serial_recv_.wheel_vlf;
                    raw_msg.vlb = nav_serial_recv_.wheel_vlb;
                    raw_msg.vrb = nav_serial_recv_.wheel_vrb;
                    raw_msg.vrf = nav_serial_recv_.wheel_vrf;
                    raw_msg.gimbal_yaw = nav_serial_recv_.gimbal_yaw;
                    wheel_raw_pub_->publish(raw_msg);
                }
                /* Publish gimbal joint state */

            }

            if (msgSerialDecisionRecv.Pop(decision_serial_recv_)) {
                got_any = true;
                referee_raw_pub_->publish(PLCDecisionRecv2RefereeRawMsg(decision_serial_recv_));
            }

            if (!got_any) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            
        }
    }

    // æååé
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
    std::atomic<bool> if_on_attack_{false};
    std::atomic<bool> current_target_valid_{false};
    std::atomic<int> sentry_attitude_switch_{3};
    // æååé
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
    std::atomic<double> tunnel_target_yaw_{0.0};
    std::atomic<double> tunnel_yaw_error_{0.0};
    std::atomic<double> current_target_x_{0.0};
    std::atomic<double> current_target_y_{0.0};
 


    // çº¿ç¨
    std::thread processing_thread_;
#ifdef USE_SERIALPORT_RECV
    std::thread serial_recv_producer_;
    std::thread serial_recv_consumer_;
    std::thread serial_daemon_;
#endif

    // ROS2æ¥å£
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<algo_master::msg::PLC2Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    rclcpp::Publisher<sentry_decision_msg::msg::RefereeRaw>::SharedPtr referee_raw_pub_;
    rclcpp::Publisher<sentry_decision_msg::msg::EnemyPos>::SharedPtr enemy_pos_pub_;
    rclcpp::Publisher<sentry_decision_msg::msg::ManualPos>::SharedPtr manual_pos_pub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cur_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<algo_master::msg::WheelRaw>::SharedPtr wheel_raw_pub_;

    // cur_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("current_pos_msg",10);
        wheel_raw_pub_ = this->create_publisher<algo_master::msg::WheelRaw>("wheel_raw", 10);
    rclcpp::Subscription<algo_master::msg::PLC2Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<algo_master::msg::PLC2Target>::SharedPtr plc2target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
     // ROS2æ¥å£
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::TunnelMonitor>::SharedPtr tunnel_monitor_sub_;
    rclcpp::Subscription<sentry_decision_msg::msg::AttitudeSwitch>::SharedPtr attitude_switch_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr test_tf_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

    // Navigation2 Action å®¢æ·ç«?    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;

    std::atomic<double> current_map_yaw_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AlgoMasterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

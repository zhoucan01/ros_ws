// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <mutex>
#include <string>

#ifndef BT_PLUGIN_EXPORT
#if defined(_WIN32)
#define BT_PLUGIN_EXPORT __declspec(dllexport)
#else
#define BT_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif
#endif

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace pb_nav2_bt_nodes
{

class SelectPoseByTopic : public BT::SyncActionNode
{
public:
  SelectPoseByTopic(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    active_topic_ = getInput<std::string>("active_topic").value();
    pose_topic_ = getInput<std::string>("pose_topic").value();

    active_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      active_topic_, qos,
      std::bind(&SelectPoseByTopic::activeCallback, this, std::placeholders::_1));
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, qos,
      std::bind(&SelectPoseByTopic::poseCallback, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("active_topic", "Boolean topic used to enable override pose"),
      BT::InputPort<std::string>("pose_topic", "Pose topic used as the override goal"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("default_pose", "Default navigation goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose", "Selected navigation goal")
    };
  }

  BT::NodeStatus tick() override
  {
    auto default_pose = getInput<geometry_msgs::msg::PoseStamped>("default_pose");
    if (!default_pose) {
      throw BT::RuntimeError(default_pose.error());
    }

    geometry_msgs::msg::PoseStamped selected_pose = default_pose.value();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // When tunnel recovery is active, temporarily replace the external nav goal
      // with the locally generated retreat pose.
      if (recovery_active_ && recovery_pose_received_) {
        selected_pose = recovery_pose_;
      }
    }

    setOutput("output_pose", selected_pose);
    return BT::NodeStatus::SUCCESS;
  }

private:
  void activeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recovery_active_ = msg->data;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recovery_pose_ = *msg;
    recovery_pose_received_ = true;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::string active_topic_;
  std::string pose_topic_;
  bool recovery_active_{false};
  bool recovery_pose_received_{false};
  geometry_msgs::msg::PoseStamped recovery_pose_;
  std::mutex mutex_;
};

}  // namespace pb_nav2_bt_nodes

extern "C" BT_PLUGIN_EXPORT void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<pb_nav2_bt_nodes::SelectPoseByTopic>("SelectPoseByTopic");
}

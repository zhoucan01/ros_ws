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

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace pb_nav2_bt_nodes
{

class TopicBoolCondition : public BT::ConditionNode
{
public:
  TopicBoolCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    topic_name_ = getInput<std::string>("topic_name").value();
    expected_value_ = getInput<bool>("expected_value").value();

    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic_name_, qos,
      std::bind(&TopicBoolCondition::callback, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "Boolean topic to observe"),
      BT::InputPort<bool>("expected_value", true, "Expected boolean value")
    };
  }

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_) {
      return BT::NodeStatus::FAILURE;
    }
    return value_ == expected_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  void callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = msg->data;
    received_ = true;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::string topic_name_;
  bool expected_value_{true};
  bool value_{false};
  bool received_{false};
  std::mutex mutex_;
};

}  // namespace pb_nav2_bt_nodes

extern "C" BT_PLUGIN_EXPORT void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<pb_nav2_bt_nodes::TopicBoolCondition>("TopicBoolCondition");
}

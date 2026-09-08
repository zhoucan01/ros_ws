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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace pb_nav2_bt_nodes
{

class WaitForTunnelReopen : public BT::StatefulActionNode
{
public:
  WaitForTunnelReopen(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    topic_name_ = getInput<std::string>("topic_name").value();
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic_name_, qos,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        waiting_ = message->data;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("topic_name", "Tunnel-reopen wait topic")};
  }

  BT::NodeStatus onStart() override
  {
    return isWaiting() ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning() override
  {
    // Failure on reopen invalidates the retreat path and triggers a fresh plan.
    return isWaiting() ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }

  void onHalted() override {}

private:
  bool isWaiting()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return waiting_;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::string topic_name_;
  bool waiting_{false};
  std::mutex mutex_;
};

}  // namespace pb_nav2_bt_nodes

extern "C" BT_PLUGIN_EXPORT void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<pb_nav2_bt_nodes::WaitForTunnelReopen>("WaitForTunnelReopen");
}

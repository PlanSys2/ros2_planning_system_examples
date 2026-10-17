// Copyright 2019 Intelligent Robotics Lab
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

#ifndef PLANSYS2_BT_EXAMPLE__BEHAVIOR_TREE_NODES__MOVE_HPP_
#define PLANSYS2_BT_EXAMPLE__BEHAVIOR_TREE_NODES__MOVE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "easynav_interfaces/msg/goal_manager_info.hpp"
#include "easynav_system/GoalManagerClient.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2_bt_tests
{

// Node and EasyNav client of a performer, kept between the trees of its actions
struct Navigator
{
  Navigator(const std::string & name, const std::string & ns);

  // The goal manager is matched both ways; is_connected() also counts other clients
  bool connected() const;
  // Every client gets the feedback of all robots: take all the queued messages
  void spin();

  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  easynav::GoalManagerClient::SharedPtr client;
  // Only the goal manager publishes it
  rclcpp::Subscription<easynav_interfaces::msg::GoalManagerInfo>::SharedPtr manager_info;
};

// Navigates to a waypoint with EasyNav
class Move : public BT::StatefulActionNode
{
public:
  Move(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal"),
      BT::InputPort<double>(
        "server_timeout", 5.0, "Seconds to wait for EasyNav to be connected and to accept")
    };
  }

private:
  // A new client, with a new id, so that late messages to the old one are ignored
  void renew_navigator();
  void wait_while(easynav::GoalManagerClient::State state, double seconds);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string navigator_key_;
  std::shared_ptr<Navigator> navigator_;
  std::map<std::string, geometry_msgs::msg::Pose> waypoints_;

  geometry_msgs::msg::PoseStamped goal_;
  bool goal_sent_ {false};
  std::chrono::steady_clock::time_point phase_start_;
  double server_timeout_ {5.0};
};

}  // namespace plansys2_bt_tests

#endif  // PLANSYS2_BT_EXAMPLE__BEHAVIOR_TREE_NODES__MOVE_HPP_

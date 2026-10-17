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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_bt_example/behavior_tree_nodes/Move.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2_bt_tests
{

using State = easynav::GoalManagerClient::State;

// Without global arguments, so that the node name given in the launch does not apply to it
Navigator::Navigator(const std::string & name, const std::string & ns)
: node(rclcpp::Node::make_shared(
      name, ns, rclcpp::NodeOptions().use_global_arguments(false)))
{
  executor.add_node(node);
  client = easynav::GoalManagerClient::make_shared(node);
  manager_info = node->create_subscription<easynav_interfaces::msg::GoalManagerInfo>(
    "easynav_manager_info", 10, [](easynav_interfaces::msg::GoalManagerInfo::UniquePtr) {});
}

void
Navigator::spin()
{
  executor.spin_all(std::chrono::milliseconds(100));
}

bool
Navigator::connected() const
{
  return manager_info->get_publisher_count() > 0 && client->is_connected();
}

Move::Move(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  if (!config().blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError("Move: no 'node' in the blackboard");
  }

  // In the root blackboard, so that it outlives this tree
  navigator_key_ = std::string("@easynav_navigator_") + node_->get_name();
  if (!config().blackboard->get(navigator_key_, navigator_) || !navigator_) {
    renew_navigator();
  }

  if (!node_->has_parameter("waypoints")) {
    node_->declare_parameter<std::vector<std::string>>("waypoints", std::vector<std::string>{});
  }

  std::vector<std::string> wp_names;
  node_->get_parameter("waypoints", wp_names);

  for (const auto & wp : wp_names) {
    const auto param = "waypoint_coords." + wp;
    if (!node_->has_parameter(param)) {
      node_->declare_parameter<std::vector<double>>(param, std::vector<double>{});
    }

    std::vector<double> coords;
    node_->get_parameter(param, coords);
    if (coords.size() < 3) {
      RCLCPP_ERROR(node_->get_logger(), "Waypoint [%s] needs [x, y, yaw]", wp.c_str());
      continue;
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = coords[0];
    pose.position.y = coords[1];
    pose.orientation = tf2::toMsg(tf2::Quaternion({0.0, 0.0, 1.0}, coords[2]));
    waypoints_[wp] = pose;
  }
}

void
Move::renew_navigator()
{
  if (!rclcpp::ok()) {  // Shutting down: no node can be created
    return;
  }

  static std::atomic<int> generation {0};
  const int n = generation++;
  auto name = std::string(node_->get_name()) + "_nav" + (n > 0 ? "_" + std::to_string(n) : "");

  try {
    navigator_ = std::make_shared<Navigator>(name, node_->get_namespace());
    config().blackboard->set(navigator_key_, navigator_);
  } catch (const rclcpp::exceptions::RCLError & e) {  // Shutdown in between
    RCLCPP_WARN(node_->get_logger(), "No EasyNav client: %s", e.what());
  }
}

BT::NodeStatus
Move::onStart()
{
  std::string goal;
  if (!getInput<std::string>("goal", goal) || waypoints_.find(goal) == waypoints_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "No coordinates for waypoint [%s]", goal.c_str());
    return BT::NodeStatus::FAILURE;
  }
  getInput<double>("server_timeout", server_timeout_);
  if (!navigator_) {
    return BT::NodeStatus::FAILURE;
  }

  navigator_->spin();
  switch (navigator_->client->get_state()) {
    case State::IDLE:
      break;
    case State::NAVIGATION_FINISHED:
    case State::NAVIGATION_REJECTED:
    case State::NAVIGATION_FAILED:
    case State::NAVIGATION_CANCELLED:
    case State::ERROR:
      navigator_->client->reset();
      break;
    default:  // A navigation of a previous tree still running
      renew_navigator();
  }

  goal_.header.frame_id = "map";
  goal_.header.stamp = node_->now();
  goal_.pose = waypoints_[goal];

  goal_sent_ = false;
  phase_start_ = std::chrono::steady_clock::now();

  return onRunning();
}

BT::NodeStatus
Move::onRunning()
{
  navigator_->spin();

  auto & client = navigator_->client;
  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - phase_start_).count();

  if (!goal_sent_) {
    if (navigator_->manager_info->get_publisher_count() > 1) {
      RCLCPP_ERROR(node_->get_logger(), "More than one EasyNav goal manager in easynav_control");
      return BT::NodeStatus::FAILURE;
    }
    if (navigator_->connected()) {
      client->send_goal(goal_);
      goal_sent_ = true;
      phase_start_ = std::chrono::steady_clock::now();
    } else if (elapsed > server_timeout_) {
      RCLCPP_ERROR(node_->get_logger(), "No EasyNav goal manager in easynav_control");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  switch (client->get_state()) {
    case State::SENT_GOAL:
      if (elapsed > server_timeout_) {
        RCLCPP_ERROR(node_->get_logger(), "EasyNav did not answer the navigation goal");
        renew_navigator();
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    case State::SENT_PREEMPT:
    case State::ACCEPTED_AND_NAVIGATING:
      return BT::NodeStatus::RUNNING;
    case State::NAVIGATION_FINISHED:
      client->reset();
      return BT::NodeStatus::SUCCESS;
    case State::IDLE:  // Not expected after sending
      return BT::NodeStatus::FAILURE;
    default:
      client->reset();
      return BT::NodeStatus::FAILURE;
  }
}

void
Move::wait_while(State state, double seconds)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(seconds));

  while (navigator_->client->get_state() == state &&
    std::chrono::steady_clock::now() < deadline && rclcpp::ok())
  {
    navigator_->spin();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void
Move::onHalted()
{
  // Shutting down: nothing can be sent anymore
  if (!goal_sent_ || !rclcpp::ok()) {
    goal_sent_ = false;
    return;
  }

  auto & client = navigator_->client;
  navigator_->spin();

  // Accepted or not, the goal must not keep the robot moving
  wait_while(State::SENT_GOAL, 1.0);
  if (client->get_state() == State::ACCEPTED_AND_NAVIGATING) {
    client->cancel();
    wait_while(State::ACCEPTED_AND_NAVIGATING, 1.0);
  }

  switch (client->get_state()) {
    case State::IDLE:
      break;
    case State::NAVIGATION_FINISHED:
    case State::NAVIGATION_REJECTED:
    case State::NAVIGATION_FAILED:
    case State::NAVIGATION_CANCELLED:
    case State::ERROR:
      client->reset();
      break;
    default:
      RCLCPP_WARN(node_->get_logger(), "EasyNav did not confirm the cancellation");
      renew_navigator();
  }
  goal_sent_ = false;
}

}  // namespace plansys2_bt_tests

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_bt_tests::Move>("Move");
}

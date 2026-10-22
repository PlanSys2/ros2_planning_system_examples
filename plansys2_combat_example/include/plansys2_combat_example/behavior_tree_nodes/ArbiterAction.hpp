// Copyright 2026 Intelligent Robotics Lab
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

#ifndef PLANSYS2_COMBAT_EXAMPLE__BEHAVIOR_TREE_NODES__ARBITERACTION_HPP_
#define PLANSYS2_COMBAT_EXAMPLE__BEHAVIOR_TREE_NODES__ARBITERACTION_HPP_

#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "plansys2_combat_example/srv/cancel_action.hpp"
#include "plansys2_combat_example/srv/finish_action.hpp"
#include "plansys2_combat_example/srv/start_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2_combat_example
{

// One action of a robot, as the arbiter resolves it: it asks for its duration, waits it, and
// asks for its result. Fails if the action did not do what the plan expected; tells the
// arbiter if it is halted before its end
class ArbiterAction : public BT::StatefulActionNode
{
public:
  ArbiterAction(const std::string & xml_tag_name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("action", "Action of the domain"),
      BT::InputPort<int>("nargs", "Number of its arguments: arg0 .. arg(nargs-1)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus ask_to_start();

  using StartAction = plansys2_combat_example::srv::StartAction;
  using FinishAction = plansys2_combat_example::srv::FinishAction;
  using CancelAction = plansys2_combat_example::srv::CancelAction;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Client<StartAction>::SharedPtr start_client_;
  rclcpp::Client<FinishAction>::SharedPtr finish_client_;
  rclcpp::Client<CancelAction>::SharedPtr cancel_client_;
  std::string side_;

  std::string action_;
  std::vector<std::string> args_;
  std::optional<rclcpp::Client<StartAction>::SharedFuture> start_future_;
  std::optional<rclcpp::Client<FinishAction>::SharedFuture> finish_future_;
  rclcpp::Time asked_;
  rclcpp::Time started_;
  double duration_ = 0.0;
  bool under_way_ = false;  // started and not finished: a halt must tell the arbiter
};

}  // namespace plansys2_combat_example

#endif  // PLANSYS2_COMBAT_EXAMPLE__BEHAVIOR_TREE_NODES__ARBITERACTION_HPP_

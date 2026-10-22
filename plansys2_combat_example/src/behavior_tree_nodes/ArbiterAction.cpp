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

#include "plansys2_combat_example/behavior_tree_nodes/ArbiterAction.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace plansys2_combat_example
{

using namespace std::chrono_literals;  // NOLINT

ArbiterAction::ArbiterAction(const std::string & xml_tag_name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
  start_client_ = node_->create_client<StartAction>("/combat/start_action");
  finish_client_ = node_->create_client<FinishAction>("/combat/finish_action");
  cancel_client_ = node_->create_client<CancelAction>("/combat/cancel_action");
  // The side is the namespace the performer runs in: /blue -> blue
  side_ = node_->get_namespace();
  side_.erase(0, side_.find_first_not_of('/'));
}

BT::NodeStatus
ArbiterAction::onStart()
{
  action_ = getInput<std::string>("action").value_or("");
  const int nargs = getInput<int>("nargs").value_or(0);
  args_.clear();
  for (int i = 0; i < nargs; i++) {
    args_.push_back(config().blackboard->get<std::string>("arg" + std::to_string(i)));
  }
  start_future_.reset();
  finish_future_.reset();
  asked_ = node_->now();
  config().blackboard->set<float>("completion", 0.0f);
  return ask_to_start();
}

// Asks the arbiter to start the action, once it is there
BT::NodeStatus
ArbiterAction::ask_to_start()
{
  if (!start_client_->service_is_ready()) {
    if ((node_->now() - asked_).seconds() > 5.0) {
      RCLCPP_ERROR(node_->get_logger(), "no arbiter: /combat/start_action is not available");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  auto request = std::make_shared<StartAction::Request>();
  request->side = side_;
  request->action = action_;
  request->arguments = args_;
  start_future_ = start_client_->async_send_request(request).future.share();
  under_way_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
ArbiterAction::onRunning()
{
  if (!under_way_ && !finish_future_) {
    return ask_to_start();
  }

  // Waiting for the arbiter to accept the start
  if (start_future_) {
    if (start_future_->wait_for(0s) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    auto response = start_future_->get();
    start_future_.reset();
    if (!response->accepted) {
      under_way_ = false;
      RCLCPP_ERROR(node_->get_logger(), "%s: %s", action_.c_str(), response->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    duration_ = response->duration;
    started_ = node_->now();
  }

  // Doing it
  if (!finish_future_) {
    const double elapsed = (node_->now() - started_).seconds();
    config().blackboard->set<float>(
      "completion", static_cast<float>(duration_ > 0.0 ? std::min(1.0, elapsed / duration_) : 1.0));
    if (elapsed < duration_) {
      return BT::NodeStatus::RUNNING;
    }
    auto request = std::make_shared<FinishAction::Request>();
    request->side = side_;
    request->action = action_;
    request->arguments = args_;
    finish_future_ = finish_client_->async_send_request(request).future.share();
    under_way_ = false;  // from now on, the arbiter decides
    return BT::NodeStatus::RUNNING;
  }

  // Waiting for what really happened
  if (finish_future_->wait_for(0s) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }
  auto response = finish_future_->get();
  finish_future_.reset();
  if (!response->success) {
    RCLCPP_WARN(node_->get_logger(), "%s: %s", action_.c_str(), response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

void
ArbiterAction::onHalted()
{
  // Cancelled by PlanSys2 (a replan, another action failed): a robot on the road turns back
  if (under_way_) {
    auto request = std::make_shared<CancelAction::Request>();
    request->side = side_;
    request->action = action_;
    request->arguments = args_;
    cancel_client_->async_send_request(request);
    under_way_ = false;
  }
  start_future_.reset();
  finish_future_.reset();
}

}  // namespace plansys2_combat_example

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_combat_example::ArbiterAction>("ArbiterAction");
}

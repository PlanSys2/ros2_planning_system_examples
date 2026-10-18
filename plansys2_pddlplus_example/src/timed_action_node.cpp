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

// A performer of any action: it lasts what the plan gives it, so a durative action acts
// for all of its planned duration, and a non-durative one ends at once

#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

class TimedAction : public plansys2::ActionExecutorClient
{
public:
  // Its action comes from action_name; launched once per action
  TimedAction()
  : plansys2::ActionExecutorClient("timed_action")
  {
    set_parameter({"rate", 10.0});
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override
  {
    started_ = now();
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  void do_work() override
  {
    const double planned = get_planned_duration();
    const double elapsed = (now() - started_).seconds();
    if (elapsed >= planned) {
      finish(true, 1.0, "done");
    } else {
      send_feedback(elapsed / planned, "running");
    }
  }

  rclcpp::Time started_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimedAction>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

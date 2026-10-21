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

// Performer of any action that just takes `duration` seconds (approach, arm motions, assemble)

#include <memory>

#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

class TimedAction : public plansys2::ActionExecutorClient
{
public:
  TimedAction()
  : plansys2::ActionExecutorClient("timed_action")
  {
    declare_parameter("duration", 1.0);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override
  {
    started_ = now();
    return ActionExecutorClient::on_activate(state);
  }

private:
  void do_work() override
  {
    double duration = get_parameter("duration").as_double();
    double elapsed = (now() - started_).seconds();
    if (elapsed >= duration) {
      finish(true, 1.0, "done");
    } else {
      send_feedback(elapsed / duration, "running");
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

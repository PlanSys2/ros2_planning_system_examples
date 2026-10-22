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

// Every performer of a side in one process: one BTAction node per robot and action, named after
// the arguments, with its parameters (action_name, specialized_arguments, bt_xml_file, plugins)
// in the parameters file under its full name. Far fewer DDS participants than a process each

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_bt_actions/BTAction.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto names = rclcpp::remove_ros_arguments(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  std::vector<std::shared_ptr<plansys2::BTAction>> performers;
  for (size_t i = 1; i < names.size(); i++) {
    auto performer = std::make_shared<plansys2::BTAction>(names[i]);
    performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exec.add_node(performer->get_node_base_interface());
    performers.push_back(performer);
  }
  // Each lifecycle node ends its life before the context goes, as a node of its own would
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [&performers]() {
      for (auto & performer : performers) {
        performer->shutdown();
      }
    });
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

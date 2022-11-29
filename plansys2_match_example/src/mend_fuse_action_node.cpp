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

#include <memory>
#include <algorithm>
#include <cmath>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MendFuseAction : public plansys2::ActionExecutorClient
{
public:
  MendFuseAction()
  : plansys2::ActionExecutorClient("mend_fuse", 250ms)
  {
    progress_ = 0.0;
    duration_ = 5.0;
  }

private:

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state)
  {
    start_time_ = now();
    return ActionExecutorClient::on_activate(state);
  }

  void do_work()
  {
    auto elapsed_time = now() - start_time_;
    progress_ = elapsed_time.seconds() / duration_;
    if (progress_ < 1.0) {
      send_feedback(progress_, "Mend fuse running");
    } else {
      finish(true, 1.0, "Mend fuse completed");
      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Mend fuse ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      "  elapsed_time: " << elapsed_time.seconds() <<
      std::flush;
  }

  float progress_;
  double duration_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MendFuseAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "mend_fuse"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

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

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class LightMatchAction : public plansys2::ActionExecutorClient
{
public:
  LightMatchAction()
  : plansys2::ActionExecutorClient("light_match", 250ms)
  {
    progress_ = 0.0;
    duration_ = 8.0;
  }

  private:
  void do_work()
  {
    auto elapsed_time = now() - get_start_time();
    progress_ = elapsed_time.seconds() / duration_;
    if (progress_ < 1.0) {
      send_feedback(progress_, "Light match running");
    } else {
      finish(true, 1.0, "Light match completed");
      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Light match ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;
  double duration_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LightMatchAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "light_match"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

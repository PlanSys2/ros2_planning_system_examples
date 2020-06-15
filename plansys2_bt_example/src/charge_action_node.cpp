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


#include "behavior_tree_nodes/ApproachObject.hpp"
#include "behavior_tree_nodes/OpenGripper.hpp"
#include "behavior_tree_nodes/CloseGripper.hpp"

#include "plansys2_executor/ActionBTExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class ChargeAction : public plansys2::ActionBTExecutorClient
{
public:
  ChargeAction()
  : plansys2::ActionBTExecutorClient(
      "charge",
      ament_index_cpp::get_package_share_directory("plansys2_bt_example") +
      "/behavior_trees_xml/charge.xml")
  {
    factory_.registerNodeType<plansys2_bt_example::ApproachObject>("ApproachObject");
    factory_.registerNodeType<plansys2_bt_example::OpenGripper>("OpenGripper");
    factory_.registerNodeType<plansys2_bt_example::CloseGripper>("CloseGripper");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChargeAction>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

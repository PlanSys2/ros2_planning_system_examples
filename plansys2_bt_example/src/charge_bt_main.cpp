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

#include <string>

#include "plansys2_bt_example/behavior_tree_nodes/ApproachObject.hpp"
#include "plansys2_bt_example/behavior_tree_nodes/CloseGripper.hpp"
#include "plansys2_bt_example/behavior_tree_nodes/OpenGripper.hpp"
#include "plansys2_bt_example/behavior_tree_nodes/Move.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<plansys2_bt_example::ApproachObject>("ApproachObject");
  factory.registerNodeType<plansys2_bt_example::OpenGripper>("OpenGripper");
  factory.registerNodeType<plansys2_bt_example::CloseGripper>("CloseGripper");
  factory.registerNodeType<plansys2_bt_example::Move>("Move");

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_bt_example");
  std::string xml_file = pkgpath + "/behavior_trees_xml/charge.xml";

  BT::Tree tree = factory.createTreeFromFile(xml_file);

  rclcpp::Rate rate(1);
  bool finish = false;

  while (!finish && rclcpp::ok()) {
    finish = tree.root_node->executeTick() == BT::NodeStatus::SUCCESS;

    rate.sleep();
  }

  std::cout << "BT execution finished" << std::endl;
  return 0;
}

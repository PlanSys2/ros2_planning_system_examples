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
#include <string>
#include <map>

#include "plansys2_bt_example/behavior_tree_nodes/ApproachObject.hpp"
#include "plansys2_bt_example/behavior_tree_nodes/CloseGripper.hpp"
#include "plansys2_bt_example/behavior_tree_nodes/Move.hpp"
#include "plansys2_bt_example/behavior_tree_nodes/OpenGripper.hpp"

#include "plansys2_msgs/action/execute_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "plansys2_executor/ActionBTExecutorClient.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class TransportAction : public plansys2::ActionBTExecutorClient
{
public:
  TransportAction()
  : plansys2::ActionBTExecutorClient(
      "transport",
      ament_index_cpp::get_package_share_directory("plansys2_bt_example") +
      "/behavior_trees_xml/transport.xml")
  {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.pose.position.x = 0.0;
    wp.pose.position.y = -2.0;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 0.0;
    wp.pose.orientation.w = 1.0;
    waypoints_["wheels_zone"] = wp;

    wp.pose.position.x = 1.8;
    wp.pose.position.y = 0.0;
    waypoints_["steering_wheels_zone"] = wp;

    wp.pose.position.x = 0.0;
    wp.pose.position.y = 2.0;
    waypoints_["body_car_zone"] = wp;

    wp.pose.position.x = -0.5;
    wp.pose.position.y = -0.5;
    waypoints_["wp4"] = wp;

    wp.pose.position.x = -2.0;
    wp.pose.position.y = -0.4;
    waypoints_["assembly_zone"] = wp;

    getFeedback()->progress = 0.0;

    factory_.registerNodeType<plansys2_bt_example::Move>("Move");
    factory_.registerNodeType<plansys2_bt_example::OpenGripper>("ApproachObject");
    factory_.registerNodeType<plansys2_bt_example::OpenGripper>("OpenGripper");
    factory_.registerNodeType<plansys2_bt_example::CloseGripper>("CloseGripper");
  }

private:
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  void atStart()
  {
    auto wp_to_navigate = getArguments()[3];
    auto goal_pos = waypoints_[wp_to_navigate];

    RCLCPP_INFO(get_logger(), "Start transport to [%s]", wp_to_navigate.c_str());

    getBackboard()->set("goal", goal_pos);
    getBackboard()->set("node", rclcpp::Node::make_shared("move_aux"));

    ActionBTExecutorClient::atStart();
  }

  bool isFinished()
  {
    return ActionBTExecutorClient::isFinished();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TransportAction>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

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
#include <iostream>

#include "plansys2_bt_example/behavior_tree_nodes/Move.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace plansys2_bt_example
{

Move::Move(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BtActionNode(xml_tag_name, "navigate_to_pose", config)
{
}

void
Move::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput<geometry_msgs::msg::PoseStamped>("goal", goal);

  goal_.pose = goal;
}

void
Move::on_success()
{
}

}  // namespace plansys2_bt_example

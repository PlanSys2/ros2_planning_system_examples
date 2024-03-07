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

#ifndef PLANSYS2_BT_EXAMPLE__BEHAVIOR_TREE_NODES__OPENGRIPPER_HPP_
#define PLANSYS2_BT_EXAMPLE__BEHAVIOR_TREE_NODES__OPENGRIPPER_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace plansys2_bt_example
{

class OpenGripper : public BT::ActionNodeBase
{
public:
  explicit OpenGripper(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  int counter_;
};

}  // namespace plansys2_bt_example

#endif  // PLANSYS2_BT_EXAMPLE__BEHAVIOR_TREE_NODES__OPENGRIPPER_HPP_

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

// Patrols the waypoints of `waypoints` once, in order, and ends: 0 if every plan succeeded

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PatrolWaypointsController : public rclcpp::Node
{
public:
  PatrolWaypointsController()
  : rclcpp::Node("patrol_waypoints_controller")
  {
    declare_parameter("waypoints", std::vector<std::string>{"wp1", "wp2"});
  }

  // Whether the node is active, asked through its lifecycle
  bool active(const std::string & node)
  {
    auto client = create_client<lifecycle_msgs::srv::GetState>(node + "/get_state");
    if (!client->wait_for_service(1s)) {
      return false;
    }
    auto future = client->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if (rclcpp::spin_until_future_complete(shared_from_this(), future, 1s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }
    return future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  int run()
  {
    auto domain = std::make_shared<plansys2::DomainExpertClient>();
    auto problem = std::make_shared<plansys2::ProblemExpertClient>();
    auto planner = std::make_shared<plansys2::PlannerClient>();
    auto executor = std::make_shared<plansys2::ExecutorClient>();
    for (const auto & node : {"domain_expert", "problem_expert", "planner", "executor"}) {
      while (rclcpp::ok() && !active(node)) {
        RCLCPP_INFO(get_logger(), "Waiting for %s...", node);
        rclcpp::sleep_for(500ms);
      }
    }

    // The control waypoint is connected to every other one
    problem->addInstance(plansys2::Instance{"r2d2", "robot"});
    problem->addInstance(plansys2::Instance{"wp_control", "waypoint"});
    problem->addPredicate(plansys2::Predicate("(robot_at r2d2 wp_control)"));
    for (const auto & wp : {"wp1", "wp2", "wp3", "wp4"}) {
      problem->addInstance(plansys2::Instance{wp, "waypoint"});
      problem->addPredicate(plansys2::Predicate(std::string("(connected wp_control ") + wp + ")"));
      problem->addPredicate(plansys2::Predicate(std::string("(connected ") + wp + " wp_control)"));
    }

    const auto waypoints = get_parameter("waypoints").as_string_array();
    for (const auto & wp : waypoints) {
      problem->setGoal(plansys2::Goal("(and (patrolled " + wp + "))"));
      auto plan = planner->getPlan(domain->getDomain(), problem->getProblem());
      if (!plan.has_value()) {
        RCLCPP_ERROR(get_logger(), "No plan to patrol %s", wp.c_str());
        return 1;
      }
      if (!executor->start_plan_execution(plan.value())) {
        RCLCPP_ERROR(get_logger(), "The plan to patrol %s could not start", wp.c_str());
        return 1;
      }
      while (rclcpp::ok() && executor->execute_and_check_plan()) {
        rclcpp::sleep_for(200ms);
      }
      auto result = executor->getResult();
      if (!result.has_value() ||
        result->result != plansys2_msgs::action::ExecutePlan::Result::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "Patrolling %s failed", wp.c_str());
        return 1;
      }
      RCLCPP_INFO(get_logger(), "%s patrolled", wp.c_str());
    }
    return 0;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolWaypointsController>();
  const int result = node->run();
  rclcpp::shutdown();
  return result;
}

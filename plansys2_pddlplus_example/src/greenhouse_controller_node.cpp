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

// Plans the watering of the greenhouse with Kairos and executes it, showing what the dynamics
// do meanwhile: the battery, the level of the cistern and the temperature of the nursery.
// Ends with 0 if the goal holds at the end of the plan

#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class GreenhouseController : public rclcpp::Node
{
public:
  GreenhouseController()
  : rclcpp::Node("greenhouse_controller")
  {
  }

  // The result: 0 if the plan reached the goal
  int run()
  {
    auto domain = std::make_shared<plansys2::DomainExpertClient>();
    auto problem = std::make_shared<plansys2::ProblemExpertClient>();
    auto planner = std::make_shared<plansys2::PlannerClient>();
    auto executor = std::make_shared<plansys2::ExecutorClient>();

    // The world evolves from the moment the problem is there: it starts now, as it is in the file
    const auto file = ament_index_cpp::get_package_share_path("plansys2_pddlplus_example") /
      "pddl" / "watering.pddl";
    std::ifstream in(file);
    std::stringstream text;
    text << in.rdbuf();
    while (rclcpp::ok() && !(problem->clearKnowledge() && problem->addProblem(text.str()))) {
      RCLCPP_INFO(get_logger(), "Waiting for the problem expert...");
      rclcpp::sleep_for(500ms);
    }

    auto plan = planner->getPlan(domain->getDomain(), problem->getProblem());
    if (!plan.has_value()) {
      RCLCPP_ERROR(get_logger(), "No plan for the watering");
      return 1;
    }
    RCLCPP_INFO(get_logger(), "Plan (it ends at %.3f s):", plan->end);
    for (const auto & item : plan->items) {
      RCLCPP_INFO(
        get_logger(), "  %7.3f: %s [%.3f]", item.time, item.action.c_str(), item.duration);
    }

    if (!executor->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "The plan could not start");
      return 1;
    }
    auto value = [&problem](const std::string & function) {
        auto f = problem->getFunction(function);
        return f ? f->value : 0.0;
      };
    while (rclcpp::ok() && executor->execute_and_check_plan()) {
      RCLCPP_INFO(
        get_logger(), "battery %5.1f  cistern %5.2f  nursery %5.2f C",
        value("(battery rover)"), value("(level cistern)"), value("(temperature nursery)"));
      rclcpp::sleep_for(1s);
    }

    auto result = executor->getResult();
    const bool reached = result.has_value() &&
      result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS &&
      problem->isGoalSatisfied(problem->getGoal());
    RCLCPP_INFO(
      get_logger(), "battery %5.1f  cistern %5.2f  nursery %5.2f C",
      value("(battery rover)"), value("(level cistern)"), value("(temperature nursery)"));
    if (reached) {
      RCLCPP_INFO(get_logger(), "Watered, not too hot, and back at the dock");
      return 0;
    }
    RCLCPP_ERROR(get_logger(), "The plan did not reach the goal");
    return 1;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GreenhouseController>();
  const int ret = node->run();
  rclcpp::shutdown();
  return ret;
}

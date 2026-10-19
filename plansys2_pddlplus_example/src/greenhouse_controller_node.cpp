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
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class GreenhouseController : public rclcpp::Node
{
public:
  static constexpr int kAttempts = 3;

  GreenhouseController()
  : rclcpp::Node("greenhouse_controller")
  {
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

  // The result: 0 if the plan reached the goal
  int run()
  {
    auto domain = std::make_shared<plansys2::DomainExpertClient>();
    auto problem = std::make_shared<plansys2::ProblemExpertClient>();
    auto planner = std::make_shared<plansys2::PlannerClient>();
    auto executor = std::make_shared<plansys2::ExecutorClient>();
    // PlanSys2 up, before the world starts
    for (const auto & node : {"domain_expert", "problem_expert", "planner", "executor"}) {
      while (rclcpp::ok() && !active(node)) {
        RCLCPP_INFO(get_logger(), "Waiting for %s...", node);
        rclcpp::sleep_for(500ms);
      }
    }

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

    auto value = [&problem](const std::string & function) {
        auto f = problem->getFunction(function);
        return f ? f->value : 0.0;
      };

    // A plan that fails is made again from where the world is now, a few times
    bool reached = false;
    for (int attempt = 1; attempt <= kAttempts && !reached && rclcpp::ok(); attempt++) {
      auto plan = planner->getPlan(domain->getDomain(), problem->getProblem());
      if (!plan.has_value()) {
        RCLCPP_ERROR(get_logger(), "No plan for the watering from here");
        return 1;
      }
      RCLCPP_INFO(get_logger(), "Plan %d (it ends at %.3f s):", attempt, plan->end);
      for (const auto & item : plan->items) {
        RCLCPP_INFO(
          get_logger(), "  %7.3f: %s [%.3f]", item.time, item.action.c_str(), item.duration);
      }

      if (!executor->start_plan_execution(plan.value())) {
        RCLCPP_ERROR(get_logger(), "The plan could not start");
        return 1;
      }
      while (rclcpp::ok() && executor->execute_and_check_plan()) {
        RCLCPP_INFO(
          get_logger(), "battery %5.1f  cistern %5.2f  nursery %5.2f C",
          value("(battery rover)"), value("(level cistern)"), value("(temperature nursery)"));
        rclcpp::sleep_for(1s);
      }

      auto result = executor->getResult();
      reached = result.has_value() &&
        result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS &&
        problem->isGoalSatisfied(problem->getGoal());
      if (!reached && result.has_value()) {
        for (const auto & action : result->action_execution_status) {
          if (action.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
            RCLCPP_WARN(
              get_logger(), "%s failed: %s", action.action_full_name.c_str(),
              action.message_status.c_str());
          }
        }
        RCLCPP_WARN(get_logger(), "Replanning from where it is");
      }
    }
    RCLCPP_INFO(
      get_logger(), "battery %5.1f  cistern %5.2f  nursery %5.2f C",
      value("(battery rover)"), value("(level cistern)"), value("(temperature nursery)"));
    if (!reached) {
      RCLCPP_ERROR(get_logger(), "The plan did not reach the goal");
      return 1;
    }

    // What each part of the domain did, seen in the final state
    auto holds = [&problem](const std::string & predicate) {
        return problem->existPredicate(parser::pddl::fromStringPredicate(predicate));
      };
    auto tool = problem->getFunction("(tool rover)");
    const std::vector<std::pair<std::string, bool>> evidence = {
      {"continuous effects: the battery moved with recharge and move",
        value("(battery rover)") != 10.0},
      {"processes: the cistern filled and the nursery warmed up",
        value("(level cistern)") >= 6.0 && value("(temperature nursery)") > 20.0},
      {"events: the float switch closed the full cistern",
        holds("(full cistern)") && !holds("(valve_open cistern)")},
      {"timed literal: the feeder came at 6 s and the rover carries it",
        tool.has_value() && tool->object == "feeder"},
      {"timed fluent: the water pressure went up", value("(flow cistern)") == 2.0},
      {"recursive derived: the cable plugged powers the nursery", holds("(powered nursery)")},
      {"conditional effect: tended with the feeder, the orchid is fertilized",
        holds("(fertilized orchid)")},
      {"forall goal: every plant tended", holds("(tended orchid)") && holds("(tended fern)")},
      {"constraints: no heat alarm, a battery reserve",
        !holds("(heat_alarm nursery)") && value("(battery rover)") >= 5.0},
    };
    bool all = true;
    for (const auto & [what, ok] : evidence) {
      if (ok) {
        RCLCPP_INFO(get_logger(), "  ok  %s", what.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "  NO  %s", what.c_str());
        all = false;
      }
    }
    // Soft: the planner may leave it
    RCLCPP_INFO(
      get_logger(), "  preference home: %s",
      holds("(robot_at rover dock)") ? "kept, back at the dock" : "given up, it stays");
    if (!all) {
      return 1;
    }
    RCLCPP_INFO(get_logger(), "Watered, fertilized and not too hot");
    return 0;
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

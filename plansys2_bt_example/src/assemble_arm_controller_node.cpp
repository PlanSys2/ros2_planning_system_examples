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

// Controller of the assembly with arms: sets the problem for `cars` cars, executes the plan and
// reports how long it took, to compare how the BT builders parallelize the plan

#include <algorithm>
#include <memory>
#include <thread>
#include <string>


#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"

class AssembleArm : public rclcpp::Node
{
public:
  AssembleArm()
  : rclcpp::Node("assembling_arm_controller")
  {
    declare_parameter("cars", 3);
  }


  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>(*this);
    planner_client_ = std::make_shared<plansys2::PlannerClient>(*this);
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(*this);
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(*this);

    init_knowledge(get_parameter("cars").as_int());

    auto plan = planner_client_->getPlan(domain_expert_->getDomain(),
        problem_expert_->getProblem());
    if (!plan.has_value()) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Could not find plan to reach goal " <<
          parser::pddl::toString(problem_expert_->getGoal()));
      return false;
    }

    double makespan = 0.0;
    for (const auto & item : plan->items) {
      makespan = std::max(makespan, static_cast<double>(item.time + item.duration));
    }
    RCLCPP_INFO(
      get_logger(), "Plan with %zu actions, %.1f s planned", plan->items.size(), makespan);
    for (const auto & item : plan->items) {
      RCLCPP_INFO(
        get_logger(), "  %7.3f: %s [%.3f]", item.time, item.action.c_str(), item.duration);
    }

    start_ = now();
    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting the plan");
      return false;
    }
    return true;
  }

  // False when the plan has finished
  bool step()
  {
    if (executor_client_->execute_and_check_plan()) {
      return true;
    }
    auto result = executor_client_->getResult();
    double elapsed = (now() - start_).seconds();
    succeeded_ = result && result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
    if (succeeded_) {
      RCLCPP_INFO(get_logger(), "Plan succesfully finished in %.1f s", elapsed);
    } else {
      RCLCPP_ERROR(get_logger(), "Plan finished with error after %.1f s", elapsed);
    }
    return false;
  }

  bool succeeded() const {return succeeded_;}

private:
  void init_knowledge(int cars)
  {
    for (auto robot : {"r2d2", "c3po", "bb8"}) {
      problem_expert_->addInstance(plansys2::Instance{robot, "robot"});
    }
    for (auto zone : {"wheels_zone", "steering_wheels_zone", "body_car_zone", "assembly_zone"}) {
      problem_expert_->addInstance(plansys2::Instance{zone, "zone"});
    }
    problem_expert_->addPredicate(plansys2::Predicate("(is_assembly_zone assembly_zone)"));

    // Each robot takes care of one kind of piece
    problem_expert_->addPredicate(plansys2::Predicate("(assigned r2d2 wheels_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(assigned c3po body_car_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(assigned bb8 steering_wheels_zone)"));
    for (std::string robot : {"r2d2", "c3po", "bb8"}) {
      for (auto predicate : {"robot_at", "heading"}) {
        problem_expert_->addPredicate(
          plansys2::Predicate("(" + std::string(predicate) + " " + robot + " assembly_zone)"));
      }
      problem_expert_->addPredicate(plansys2::Predicate("(robot_available " + robot + ")"));
      problem_expert_->addPredicate(plansys2::Predicate("(arm_free " + robot + ")"));
    }

    std::string goal = "(and";
    for (int i = 1; i <= cars; i++) {
      auto n = std::to_string(i);
      problem_expert_->addInstance(plansys2::Instance{"car_" + n, "car"});
      add_piece("wheel_" + n, "piece_is_wheel", "wheels_zone");
      add_piece("body_car_" + n, "piece_is_body_car", "body_car_zone");
      add_piece("steering_wheel_" + n, "piece_is_steering_wheel", "steering_wheels_zone");
      goal += "(car_assembled car_" + n + ")";
    }
    problem_expert_->setGoal(plansys2::Goal(goal + ")"));
  }

  void add_piece(const std::string & piece, const std::string & kind, const std::string & zone)
  {
    problem_expert_->addInstance(plansys2::Instance{piece, "piece"});
    problem_expert_->addPredicate(plansys2::Predicate("(" + kind + " " + piece + ")"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at " + piece + " " + zone + ")"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used " + piece + ")"));
  }

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  rclcpp::Time start_;
  bool succeeded_ {false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AssembleArm>();
  // Spun from the start in a thread of its own: the clients get their answers from it
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  std::thread spinner([&exec]() {exec.spin();});

  if (!node->init()) {
    exec.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::Rate rate(5);
  while (rclcpp::ok() && node->step()) {
    rate.sleep();
  }

  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  // A test tells by it whether the plan succeeded
  return node->succeeded() ? 0 : 1;
}

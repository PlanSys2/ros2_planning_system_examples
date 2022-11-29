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

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Assemble : public rclcpp::Node
{
public:
  Assemble()
  : rclcpp::Node("assembling_controller")
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    init_knowledge();

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return false;
    }

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }

    return true;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"wheels_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"steering_wheels_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"body_car_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"assembly_zone", "zone"});


    problem_expert_->addInstance(plansys2::Instance{"car_1", "car"});
    problem_expert_->addInstance(plansys2::Instance{"car_2", "car"});
    problem_expert_->addInstance(plansys2::Instance{"car_3", "car"});
    problem_expert_->addInstance(plansys2::Instance{"wheel_1", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"wheel_2", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"wheel_3", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"body_car_1", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"body_car_2", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"body_car_3", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"steering_wheel_1", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"steering_wheel_2", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"steering_wheel_3", "piece"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 assembly_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_assembly_zone assembly_zone)"));

    problem_expert_->addPredicate(plansys2::Predicate("(piece_at_zone wheel_1 wheels_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at_zone body_car_1 body_car_zone)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_at_zone steering_wheel_1 steering_wheels_zone)"));

    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_1)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_is_steering_wheel steering_wheel_1)"));

    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used wheel_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used wheel_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used wheel_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used body_car_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used body_car_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used body_car_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used steering_wheel_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used steering_wheel_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used steering_wheel_3)"));

    problem_expert_->addPredicate(plansys2::Predicate("(not_in_place wheel_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(not_in_place wheel_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(not_in_place wheel_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(not_in_place body_car_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(not_in_place body_car_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(not_in_place body_car_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(not_in_place steering_wheel_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(not_in_place steering_wheel_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(not_in_place steering_wheel_3)"));

    problem_expert_->addPredicate(plansys2::Predicate("(gripper_available r2d2)"));





/*
    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at_zone wheel_2 wheels_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at_zone wheel_3 wheels_zone)"));

    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at_zone body_car_1 body_car_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at_zone body_car_2 body_car_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at_zone body_car_3 body_car_zone)"));

    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_is_steering_wheel steering_wheel_2)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_is_steering_wheel steering_wheel_3)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_at_zone steering_wheel_2 steering_wheels_zone)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_at_zone steering_wheel_3 steering_wheels_zone)"));

*/

    problem_expert_->setGoal(
      plansys2::Goal(std::string("(and(car_assembled car_1))")));
        // "(and(ready_to_pick r2d2))"));
        // "(and(robot_at r2d2 assembly_zone))"));
        // std::string("(and(piece_at_zone steering_wheel_1 assembly_zone) ") +
        //"(piece_at_zone body_car_1 assembly_zone) )"));
       //"(and(car_assembled car_1)"));// (car_assembled car_2) (car_assembled car_3))"));
  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Assemble>();

  if (!node->init()) {
    return 0;
  }

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}

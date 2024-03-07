// Copyright 2020 Intelligent Robotics Lab
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <random>

using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Sim : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

  Nav2Sim()
  : Node("navigate_to_pose_server")
  {
  }

  void start_server()
  {
    using namespace std::placeholders;

    repeat_sentence_action_server_ = rclcpp_action::create_server<NavigateToPose>(
      shared_from_this(),
      "navigate_to_pose",
      std::bind(&Nav2Sim::handle_goal, this, _1, _2),
      std::bind(&Nav2Sim::handle_cancel, this, _1),
      std::bind(&Nav2Sim::handle_accepted, this, _1));
    
    compute_path_to_pose_server_ = rclcpp_action::create_server<ComputePathToPose>(
      shared_from_this(),
      "compute_path_to_pose",
      std::bind(&Nav2Sim::handle_goal_compute_path, this, _1, _2),
      std::bind(&Nav2Sim::handle_cancel_compute_path, this, _1),
      std::bind(&Nav2Sim::handle_accepted_compute_path, this, _1));

    RCLCPP_INFO(get_logger(), "Ready.");
  }

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr repeat_sentence_action_server_;
  rclcpp_action::Server<ComputePathToPose>::SharedPtr compute_path_to_pose_server_;
  
  NavigateToPose::Goal current_goal_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse handle_goal_compute_path(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathToPose::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
    

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp_action::CancelResponse handle_cancel_compute_path(
  const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute_compute_path(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<ComputePathToPose::Feedback>();
    auto result = std::make_shared<ComputePathToPose::Result>();

    // auto pose_cmd = goal_handle->get_goal()->pose.pose;
    auto start_pose = goal_handle->get_goal()->start.pose;
    auto goal_pose = goal_handle->get_goal()->goal.pose;

    tf2::Quaternion q;
    tf2::fromMsg(start_pose.orientation, q);

    RCLCPP_INFO(
      this->get_logger(), "Starting compute path to %lf, %lf, %lf",
      start_pose.position.x, start_pose.position.y, q.getAngle());

    auto start = now();
    int current_times = 0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 10);

    while (rclcpp::ok() && current_times++ < dis(gen)) {
      RCLCPP_INFO(this->get_logger(), "Computing path %d ", current_times);

      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);

        RCLCPP_INFO(this->get_logger(), "Action Canceled");

        return;
      }

      loop_rate.sleep();
    }
  
    // Fake Path
    nav_msgs::msg::Path fake_path;
    fake_path.header.frame_id = "map"; 

    geometry_msgs::msg::PoseStamped start_pose_msg;
    start_pose_msg.pose = start_pose;
    fake_path.poses.push_back(start_pose_msg);

    geometry_msgs::msg::PoseStamped goal_pose_msg;
    goal_pose_msg.pose = goal_pose;
    fake_path.poses.push_back(goal_pose_msg);

    result->path = fake_path;


    if (rclcpp::ok()) {
      goal_handle->succeed(result);

      RCLCPP_INFO(this->get_logger(), "Compute Path Succeeded");
    }
  }

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();

    auto pose_cmd = goal_handle->get_goal()->pose.pose;
    tf2::Quaternion q;
    tf2::fromMsg(pose_cmd.orientation, q);

    RCLCPP_INFO(
      this->get_logger(), "Starting navigation to %lf, %lf, %lf",
      pose_cmd.position.x, pose_cmd.position.y, q.getAngle());

    auto start = now();
    int current_times = 0;
    while (rclcpp::ok() && current_times++ < 10) {
      RCLCPP_INFO(this->get_logger(), "Navigating %d ", current_times);

      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);

        RCLCPP_INFO(this->get_logger(), "Action Canceled");

        return;
      }

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      goal_handle->succeed(result);

      RCLCPP_INFO(this->get_logger(), "Navigation Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&Nav2Sim::execute, this, _1), goal_handle}.detach();
  }
  void handle_accepted_compute_path(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&Nav2Sim::execute_compute_path, this, _1), goal_handle}.detach();
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Nav2Sim>();

  node->start_server();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

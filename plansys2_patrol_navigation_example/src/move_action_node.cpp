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

#include <math.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "easynav_interfaces/msg/goal_manager_info.hpp"
#include "easynav_system/GoalManagerClient.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using State = easynav::GoalManagerClient::State;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move")
  {
    // GoalManagerClient needs a plain node; without global arguments, so that the
    // node name given in the launch does not apply to it
    nav_node_ = rclcpp::Node::make_shared(
      std::string(get_name()) + "_nav", get_namespace(),
      rclcpp::NodeOptions().use_global_arguments(false));
    navigation_ = easynav::GoalManagerClient::make_shared(nav_node_);
    // Only the goal manager publishes it: is_connected() also counts other clients
    manager_info_ = nav_node_->create_subscription<easynav_interfaces::msg::GoalManagerInfo>(
      "easynav_manager_info", 10, [](easynav_interfaces::msg::GoalManagerInfo::UniquePtr) {});

    server_timeout_ = declare_parameter("server_timeout", 5.0);
    auto wp_names = declare_parameter("waypoints", std::vector<std::string>{});

    for (const auto & wp : wp_names) {
      auto coords = declare_parameter("waypoint_coords." + wp, std::vector<double>{});
      if (coords.size() < 2) {
        RCLCPP_ERROR(get_logger(), "Waypoint [%s] needs [x, y] or [x, y, yaw]", wp.c_str());
        continue;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = coords[0];
      pose.pose.position.y = coords[1];
      const double yaw = coords.size() > 2 ? coords[2] : 0.0;
      pose.pose.orientation.z = sin(yaw / 2.0);
      pose.pose.orientation.w = cos(yaw / 2.0);
      waypoints_[wp] = pose;
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
    if (waypoints_.find(wp_to_navigate) == waypoints_.end()) {
      RCLCPP_ERROR(get_logger(), "No coordinates for waypoint [%s]", wp_to_navigate.c_str());
      goal_pos_.reset();
    } else {
      RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());
      goal_pos_ = waypoints_[wp_to_navigate];
      goal_pos_->header.stamp = now();
    }

    // A navigation left by a previous action
    switch (navigation_->get_state()) {
      case State::IDLE:
        break;
      case State::SENT_GOAL:
      case State::SENT_PREEMPT:
      case State::ACCEPTED_AND_NAVIGATING:
        renew_client();
        break;
      default:
        navigation_->reset();
    }
    goal_sent_ = false;
    initial_distance_ = -1.0;
    phase_start_ = std::chrono::steady_clock::now();

    send_feedback(0.0, "Move starting");
    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    // Cancelled by the executor: the robot must stop
    if (navigation_->get_state() == State::ACCEPTED_AND_NAVIGATING) {
      navigation_->cancel();
    }
    return ActionExecutorClient::on_deactivate(previous_state);
  }

  rclcpp::Node::SharedPtr get_nav_node() {return nav_node_;}

private:
  // A new client: reset() does not leave a goal still in course
  void renew_client()
  {
    navigation_ = easynav::GoalManagerClient::make_shared(nav_node_);
  }

  void do_work()
  {
    if (!goal_pos_) {
      finish(false, 0.0, "Unknown waypoint");
      return;
    }

    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - phase_start_).count();

    if (!goal_sent_) {
      if (manager_info_->get_publisher_count() > 0 && navigation_->is_connected()) {
        navigation_->send_goal(*goal_pos_);
        goal_sent_ = true;
        phase_start_ = std::chrono::steady_clock::now();
      } else if (elapsed > server_timeout_) {
        finish(false, 0.0, "No EasyNav goal manager in easynav_control");
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for EasyNav...");
      }
      return;
    }

    switch (navigation_->get_state()) {
      case State::SENT_GOAL:
        if (elapsed > server_timeout_) {
          renew_client();
          finish(false, 0.0, "EasyNav did not answer the navigation goal");
        }
        break;
      case State::SENT_PREEMPT:
      case State::ACCEPTED_AND_NAVIGATING:
        {
          const double distance = navigation_->get_feedback().distance_to_goal;
          if (initial_distance_ < 0.0 && distance > 0.0) {
            initial_distance_ = distance;
          }
          const double progress = initial_distance_ > 0.0 ?
            1.0 - distance / initial_distance_ : 0.0;
          send_feedback(std::clamp(progress, 0.0, 1.0), "Move running");
        }
        break;
      case State::NAVIGATION_FINISHED:
        navigation_->reset();
        finish(true, 1.0, "Move completed");
        break;
      default:
        navigation_->reset();
        finish(false, 0.0, "Navigation failed");
    }
  }

  rclcpp::Node::SharedPtr nav_node_;
  easynav::GoalManagerClient::SharedPtr navigation_;
  rclcpp::Subscription<easynav_interfaces::msg::GoalManagerInfo>::SharedPtr manager_info_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  std::optional<geometry_msgs::msg::PoseStamped> goal_pos_;
  bool goal_sent_ {false};
  double initial_distance_ {-1.0};
  double server_timeout_ {5.0};
  std::chrono::steady_clock::time_point phase_start_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->set_parameter(rclcpp::Parameter("rate", 2.0));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(node->get_nav_node());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}

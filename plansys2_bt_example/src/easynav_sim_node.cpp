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

// Simulated EasyNav goal manager: each goal is reached after navigation_time seconds.
// Unlike EasyNav, it serves several clients at once, one simulated robot per client.

#include <map>
#include <memory>
#include <string>

#include "easynav_interfaces/msg/goal_manager_info.hpp"
#include "easynav_interfaces/msg/navigation_control.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using GoalManagerInfo = easynav_interfaces::msg::GoalManagerInfo;
using NavigationControl = easynav_interfaces::msg::NavigationControl;

class EasyNavSim : public rclcpp::Node
{
public:
  EasyNavSim()
  : Node("easynav_sim_node")
  {
    navigation_time_ = declare_parameter("navigation_time", 10.0);

    control_pub_ = create_publisher<NavigationControl>("easynav_control", 100);
    // Clients know by it that a goal manager is there
    info_pub_ = create_publisher<GoalManagerInfo>("easynav_manager_info", 100);
    // Another goal manager would answer every goal twice
    info_sub_ = create_subscription<GoalManagerInfo>(
      "easynav_manager_info", 10, [](GoalManagerInfo::UniquePtr) {});
    control_sub_ = create_subscription<NavigationControl>(
      "easynav_control", 100,
      [this](NavigationControl::UniquePtr msg) {control_callback(std::move(msg));});
    timer_ = create_wall_timer(100ms, [this]() {update();});

    RCLCPP_INFO(get_logger(), "Ready.");
  }

private:
  struct Navigation
  {
    NavigationControl request;
    double elapsed {0.0};
  };

  void control_callback(NavigationControl::UniquePtr msg)
  {
    if (msg->user_id == id_) {return;}

    switch (msg->type) {
      case NavigationControl::REQUEST:
        if (msg->goals.goals.empty()) {
          reply(*msg, NavigationControl::REJECT, "Goals are empty");
        } else {
          // A new goal of the same client replaces its current one
          navigations_[msg->user_id] = Navigation{*msg, 0.0};
          reply(*msg, NavigationControl::ACCEPT, "Goal accepted");
          RCLCPP_INFO(
            get_logger(), "%s: navigating to %lf, %lf", msg->user_id.c_str(),
            msg->goals.goals.back().pose.position.x, msg->goals.goals.back().pose.position.y);
        }
        break;
      case NavigationControl::CANCEL:
        if (navigations_.erase(msg->user_id) > 0) {
          reply(*msg, NavigationControl::CANCELLED, "Goal cancelled");
          RCLCPP_INFO(get_logger(), "%s: navigation cancelled", msg->user_id.c_str());
        } else {
          reply(*msg, NavigationControl::ERROR, "Nothing to cancel");
        }
        break;
      case NavigationControl::PAUSE:
        paused_ = true;
        reply(*msg, NavigationControl::PAUSED, "Navigation paused");
        break;
      case NavigationControl::RESUME:
        paused_ = false;
        reply(*msg, NavigationControl::RESUMED, "Navigation resumed");
        break;
      case NavigationControl::REJECT:
      case NavigationControl::ACCEPT:
      case NavigationControl::FEEDBACK:
      case NavigationControl::FINISHED:
      case NavigationControl::FAILED:
      case NavigationControl::CANCELLED:
      case NavigationControl::ERROR:
      case NavigationControl::PAUSED:
      case NavigationControl::RESUMED:
        break;  // Answers of other goal managers
      default:
        reply(*msg, NavigationControl::ERROR, "Unable to process message");
    }
  }

  void update()
  {
    if (info_sub_->get_publisher_count() > 1) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Another EasyNav goal manager is running: clients get every answer twice");
    }

    for (auto it = navigations_.begin(); it != navigations_.end(); ) {
      auto & nav = it->second;
      if (!paused_) {
        nav.elapsed += 0.1;
      }

      if (nav.elapsed >= navigation_time_) {
        reply(nav.request, NavigationControl::FINISHED, "Goal reached");
        RCLCPP_INFO(get_logger(), "%s: navigation finished", it->first.c_str());
        GoalManagerInfo info;
        info.status = GoalManagerInfo::IDLE;
        info.goals = nav.request.goals;
        info_pub_->publish(info);
        it = navigations_.erase(it);
        continue;
      }

      // 1 m/s along the way
      const double remaining = navigation_time_ - nav.elapsed;
      auto feedback = make_reply(nav.request, NavigationControl::FEEDBACK, "Navigating");
      feedback.navigation_time = rclcpp::Duration::from_seconds(nav.elapsed);
      feedback.estimated_time_remaining = rclcpp::Duration::from_seconds(remaining);
      feedback.distance_covered = nav.elapsed;
      feedback.distance_to_goal = remaining;
      control_pub_->publish(feedback);
      ++it;
    }
  }

  NavigationControl make_reply(
    const NavigationControl & request, uint8_t type, const std::string & status)
  {
    NavigationControl msg;
    msg.type = type;
    msg.header.stamp = now();
    msg.header.frame_id = request.header.frame_id;
    msg.seq = request.seq + 1;
    msg.user_id = id_;
    msg.nav_current_user_id = request.user_id;
    msg.goals = request.goals;
    msg.status_message = status;
    return msg;
  }

  void reply(const NavigationControl & request, uint8_t type, const std::string & status)
  {
    control_pub_->publish(make_reply(request, type, status));
  }

  const std::string id_ {"easynav_sim_node"};
  double navigation_time_;
  bool paused_ {false};
  std::map<std::string, Navigation> navigations_;

  rclcpp::Publisher<NavigationControl>::SharedPtr control_pub_;
  rclcpp::Publisher<GoalManagerInfo>::SharedPtr info_pub_;
  rclcpp::Subscription<GoalManagerInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<NavigationControl>::SharedPtr control_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EasyNavSim>());
  rclcpp::shutdown();
  return 0;
}

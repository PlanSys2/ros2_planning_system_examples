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

#ifndef PLANSYS2_COMBAT_EXAMPLE__BATTLEVIEW_HPP_
#define PLANSYS2_COMBAT_EXAMPLE__BATTLEVIEW_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_combat_example/World.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace plansys2_combat_example
{

// The battle in RViz: one TF frame per robot under "map", markers for the map, the robots,
// shots and explosions. Robots move along the roads for as long as their actions last
class BattleView
{
public:
  BattleView(rclcpp::Node * node, const World & world);

  void set_zone_position(const std::string & zone, double x, double y);
  // Water on the map, only drawn: a stretch from (x1, y1) to (x2, y2), width meters wide
  void add_water(double x1, double y1, double x2, double y2, double width);
  // The side whose knowledge the view shows: enemies it has not seen are faint. Empty: all
  void set_viewpoint(const std::string & side) {viewpoint_ = side;}

  // A robot leaves one zone for another after delay seconds, arriving duration seconds later
  void start_motion(
    const std::string & robot, const std::string & from, const std::string & to,
    double duration, double delay = 0.0);
  // Its move was cancelled: from where it is, back to its zone in duration seconds
  void retreat(const std::string & robot, double duration);
  void end_motion(const std::string & robot);

  // A shot, and an explosion if it destroyed the target
  void shot(const std::string & attacker, const std::string & target, bool destroyed);

  void publish_frames();   // TF of every robot alive, periodically
  void publish_markers();  // the map and the robots, whenever they change

private:
  // A stretch of road between two points
  struct Leg
  {
    std::pair<double, double> from;
    std::pair<double, double> to;
    rclcpp::Time start;
    double duration;
    bool hold;  // stays at its end until end_motion; a retreat ends in the zone
  };

  std::pair<double, double> zone_xy(const std::string & zone) const;
  std::pair<double, double> robot_xy(const std::string & robot);  // now
  bool on_the_road(const std::string & robot);
  double robot_z(const std::string & robot) const;
  std_msgs::msg::ColorRGBA side_color(const std::string & side, double alpha = 1.0) const;

  rclcpp::Node * node_;
  const World & world_;
  std::map<std::string, std::pair<double, double>> zones_;
  struct Water
  {
    double x1, y1, x2, y2, width;
  };
  std::vector<Water> water_;
  std::map<std::string, std::vector<Leg>> legs_;
  std::map<std::string, int> robot_ids_;
  std::string viewpoint_;
  int effect_id_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr effects_pub_;
};

}  // namespace plansys2_combat_example

#endif  // PLANSYS2_COMBAT_EXAMPLE__BATTLEVIEW_HPP_

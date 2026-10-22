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

#include "plansys2_combat_example/BattleView.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace plansys2_combat_example
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{
constexpr double kDroneHeight = 4.0;   // drones fly over the map
constexpr double kZoneSize = 6.0;
constexpr double kFlagHeight = 4.0;
constexpr double kSlotRadius = 2.2;    // robots in the same zone, around its center
constexpr double kStaleMotion = 3.0;   // seconds after which a motion with no end is ignored

std_msgs::msg::ColorRGBA color(double r, double g, double b, double a = 1.0)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

Marker base_marker(const std::string & frame, const std::string & ns, int id, int type)
{
  Marker m;
  m.header.frame_id = frame;  // stamp 0: the latest transform
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.frame_locked = frame != "map";  // follows its robot's TF without being republished
  return m;
}
}  // namespace

BattleView::BattleView(rclcpp::Node * node, const World & world)
: node_(node), world_(world)
{
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
  // The scene changes only on events: latched for late subscribers. Effects are transient
  markers_pub_ = node->create_publisher<MarkerArray>(
    "/combat/markers", rclcpp::QoS(1).transient_local().reliable());
  effects_pub_ = node->create_publisher<MarkerArray>("/combat/effects", 10);
}

void
BattleView::set_zone_position(const std::string & zone, double x, double y)
{
  zones_[zone] = {x, y};
}

void
BattleView::add_water(double x1, double y1, double x2, double y2, double width)
{
  water_.push_back({x1, y1, x2, y2, width});
}

void
BattleView::start_motion(
  const std::string & robot, const std::string & from, const std::string & to, double duration,
  double delay)
{
  // After the retreat it may be doing, if any
  auto & legs = legs_[robot];
  legs.push_back(
    {zone_xy(from), zone_xy(to), node_->now() + rclcpp::Duration::from_seconds(delay),
      std::max(duration, 1e-3), true});
}

void
BattleView::retreat(const std::string & robot, double duration)
{
  const auto here = robot_xy(robot);
  legs_[robot] = {{here, zone_xy(world_.robots().at(robot).zone), node_->now(),
    std::max(duration, 1e-3), false}};
}

void
BattleView::end_motion(const std::string & robot)
{
  legs_.erase(robot);
}

std::pair<double, double>
BattleView::zone_xy(const std::string & zone) const
{
  auto it = zones_.find(zone);
  return it != zones_.end() ? it->second : std::make_pair(0.0, 0.0);
}

bool
BattleView::on_the_road(const std::string & robot)
{
  auto it = legs_.find(robot);
  if (it == legs_.end()) {
    return false;
  }
  // Retreats end in the zone; a leg with no end_motion is forgotten after a while
  const auto now = node_->now();
  auto & legs = it->second;
  while (!legs.empty()) {
    const double left = legs.front().duration - (now - legs.front().start).seconds();
    const bool next_started = legs.size() > 1 && legs[1].start <= now;
    if (next_started || (left < 0.0 && !legs.front().hold) || left < -kStaleMotion) {
      legs.erase(legs.begin());
    } else {
      break;
    }
  }
  if (legs.empty()) {
    legs_.erase(it);
    return false;
  }
  return true;
}

std::pair<double, double>
BattleView::robot_xy(const std::string & robot)
{
  // On the road while its action lasts
  if (on_the_road(robot)) {
    const auto & leg = legs_.at(robot).front();
    const double s = std::clamp((node_->now() - leg.start).seconds() / leg.duration, 0.0, 1.0);
    return {leg.from.first + s * (leg.to.first - leg.from.first),
      leg.from.second + s * (leg.to.second - leg.from.second)};
  }

  // In its zone, on a circle with the others there so they do not overlap
  const auto & here = world_.robots().at(robot).zone;
  std::vector<std::string> mates;
  for (const auto & [name, r] : world_.robots()) {
    if (r.alive && r.zone == here && !legs_.count(name)) {
      mates.push_back(name);
    }
  }
  auto [x, y] = zone_xy(here);
  if (mates.size() > 1) {
    const auto i = std::find(mates.begin(), mates.end(), robot) - mates.begin();
    const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(mates.size());
    x += kSlotRadius * std::cos(angle);
    y += kSlotRadius * std::sin(angle);
  }
  return {x, y};
}

std_msgs::msg::ColorRGBA
BattleView::side_color(const std::string & side, double alpha) const
{
  // The first side blue, the second red
  const auto & sides = world_.sides();
  const auto i = std::find(sides.begin(), sides.end(), side) - sides.begin();
  if (i == 0) {
    return color(0.1, 0.45, 1.0, alpha);
  } else if (i == 1) {
    return color(1.0, 0.15, 0.1, alpha);
  }
  return color(0.7, 0.3, 0.9, alpha);
}

double
BattleView::robot_z(const std::string & robot) const
{
  return world_.robots().at(robot).type == "drone" ? kDroneHeight : 0.0;
}

void
BattleView::publish_frames()
{
  const auto now = node_->now();
  for (const auto & [name, r] : world_.robots()) {
    if (!r.alive) {
      continue;
    }
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "map";
    t.child_frame_id = name;
    auto [x, y] = robot_xy(name);
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = robot_z(name);
    t.transform.rotation.w = 1.0;
    tf_->sendTransform(t);
  }
}

void
BattleView::publish_markers()
{
  MarkerArray array;

  // Zones: gray, control points yellow, bases and captured points in their side's color
  int id = 0;
  for (const auto & zone : world_.zones()) {
    auto [x, y] = zone_xy(zone);
    auto m = base_marker("map", "zones", id, Marker::CUBE);
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = -0.05;
    m.scale.x = kZoneSize;
    m.scale.y = kZoneSize;
    m.scale.z = 0.1;
    const auto & cps = world_.control_points();
    std::string base_of;
    for (const auto & side : world_.sides()) {
      if (world_.base(side) == zone) {
        base_of = side;
      }
    }
    // The flag says who holds the zone; the ground, a softer tone, so robots stand out on it
    std_msgs::msg::ColorRGBA flag;
    if (!world_.owner(zone).empty()) {
      flag = side_color(world_.owner(zone));
    } else if (std::find(cps.begin(), cps.end(), zone) != cps.end()) {
      flag = color(0.95, 0.8, 0.1);
    } else if (!base_of.empty()) {
      flag = side_color(base_of);
    } else {
      flag = color(0.6, 0.6, 0.6);
    }
    m.color = color(
      0.35 + 0.4 * (flag.r - 0.35), 0.35 + 0.4 * (flag.g - 0.35), 0.35 + 0.4 * (flag.b - 0.35));
    array.markers.push_back(m);

    // A flag in a corner of the zone
    const double fx = x + kZoneSize / 2.0 - 0.6;
    const double fy = y + kZoneSize / 2.0 - 0.6;
    auto pole = base_marker("map", "flag_poles", id, Marker::CYLINDER);
    pole.pose.position.x = fx;
    pole.pose.position.y = fy;
    pole.pose.position.z = kFlagHeight / 2.0;
    pole.scale.x = pole.scale.y = 0.15;
    pole.scale.z = kFlagHeight;
    pole.color = color(0.85, 0.85, 0.85);
    array.markers.push_back(pole);

    auto cloth = base_marker("map", "flags", id, Marker::CUBE);
    cloth.pose.position.x = fx + 0.8;
    cloth.pose.position.y = fy;
    cloth.pose.position.z = kFlagHeight - 0.5;
    cloth.scale.x = 1.6;
    cloth.scale.y = 0.05;
    cloth.scale.z = 1.0;
    cloth.color = flag;
    array.markers.push_back(cloth);

    auto label = base_marker("map", "zone_labels", id, Marker::TEXT_VIEW_FACING);
    label.pose.position.x = x;
    label.pose.position.y = y - kZoneSize / 2.0 - 0.6;
    label.pose.position.z = 0.3;
    label.scale.z = 0.9;
    label.color = color(1.0, 1.0, 1.0);
    label.text = zone;
    array.markers.push_back(label);
    id++;
  }

  // Water, under the roads: bridges cross it
  for (size_t i = 0; i < water_.size(); i++) {
    const auto & w = water_[i];
    auto m = base_marker("map", "water", static_cast<int>(i), Marker::CUBE);
    m.pose.position.x = (w.x1 + w.x2) / 2.0;
    m.pose.position.y = (w.y1 + w.y2) / 2.0;
    m.pose.position.z = -0.08;
    const double yaw = std::atan2(w.y2 - w.y1, w.x2 - w.x1);
    m.pose.orientation.z = std::sin(yaw / 2.0);
    m.pose.orientation.w = std::cos(yaw / 2.0);
    m.scale.x = std::hypot(w.x2 - w.x1, w.y2 - w.y1);
    m.scale.y = w.width;
    m.scale.z = 0.05;
    m.color = color(0.15, 0.35, 0.75);
    array.markers.push_back(m);
  }

  // Roads between connected zones
  auto roads = base_marker("map", "roads", 0, Marker::LINE_LIST);
  roads.scale.x = 0.4;
  roads.color = color(0.3, 0.3, 0.3);
  for (const auto & [ab, d] : world_.connections()) {
    if (ab.first < ab.second) {
      for (const auto & z : {ab.first, ab.second}) {
        geometry_msgs::msg::Point p;
        auto [x, y] = zone_xy(z);
        p.x = x;
        p.y = y;
        roads.points.push_back(p);
      }
    }
  }
  array.markers.push_back(roads);

  // Robots, in their own frames, in their side's color; faint if the viewpoint has not seen them
  for (const auto & [name, r] : world_.robots()) {
    if (!robot_ids_.count(name)) {
      robot_ids_[name] = static_cast<int>(robot_ids_.size());
    }
    const int rid = robot_ids_[name];
    auto body = base_marker(name, "robots", rid, Marker::CUBE);
    auto label = base_marker(name, "robot_labels", rid, Marker::TEXT_VIEW_FACING);
    if (!r.alive) {
      body.action = Marker::DELETE;
      label.action = Marker::DELETE;
      array.markers.push_back(body);
      array.markers.push_back(label);
      continue;
    }
    if (r.type == "drone") {
      body.type = Marker::SPHERE;
      body.scale.x = body.scale.y = body.scale.z = 1.2;
    } else if (r.type == "infantry") {
      body.type = Marker::CYLINDER;
      body.scale.x = body.scale.y = 0.8;
      body.scale.z = 1.8;
    } else if (r.type == "tank") {
      body.scale.x = 2.2;
      body.scale.y = 1.6;
      body.scale.z = 1.0;
    } else {  // artillery
      body.type = Marker::CYLINDER;
      body.scale.x = body.scale.y = 2.0;
      body.scale.z = 0.6;
    }
    if (r.type != "drone") {
      body.pose.position.z = body.scale.z / 2.0;  // on the ground
    }
    const bool seen = viewpoint_.empty() || r.side == viewpoint_ ||
      world_.known_enemies(viewpoint_).count(name);
    const double alpha = seen ? 1.0 : 0.35;
    body.color = side_color(r.side, alpha);
    array.markers.push_back(body);

    label.pose.position.z = 2.4;
    label.scale.z = 0.7;
    label.color = color(1.0, 1.0, 1.0, alpha);
    label.text = name;
    array.markers.push_back(label);
  }

  markers_pub_->publish(array);
}

void
BattleView::shot(const std::string & attacker, const std::string & target, bool destroyed)
{
  MarkerArray array;
  auto [ax, ay] = robot_xy(attacker);
  auto [tx, ty] = robot_xy(target);

  auto line = base_marker("map", "shots", effect_id_++, Marker::LINE_LIST);
  line.scale.x = 0.15;
  line.color = color(1.0, 1.0, 0.6);
  line.lifetime = rclcpp::Duration::from_seconds(0.6);
  geometry_msgs::msg::Point from, to;
  from.x = ax;
  from.y = ay;
  from.z = robot_z(attacker) + 0.8;
  to.x = tx;
  to.y = ty;
  to.z = 0.8;
  line.points = {from, to};
  array.markers.push_back(line);

  if (destroyed) {
    auto boom = base_marker("map", "explosions", effect_id_++, Marker::SPHERE);
    boom.pose.position.x = tx;
    boom.pose.position.y = ty;
    boom.pose.position.z = 1.0;
    boom.scale.x = boom.scale.y = boom.scale.z = 3.5;
    boom.color = color(1.0, 0.5, 0.0, 0.85);
    boom.lifetime = rclcpp::Duration::from_seconds(1.5);
    array.markers.push_back(boom);
  }
  effects_pub_->publish(array);
}

}  // namespace plansys2_combat_example

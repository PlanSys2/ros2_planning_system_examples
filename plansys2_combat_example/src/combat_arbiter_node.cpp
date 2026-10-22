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

// The arbiter: the true state of the battle. It decides what the actions of every side's robots
// really do and tells each side what it sees. It knows nothing of PlanSys2: each side's
// controller turns its view into knowledge. Everything here is outside the sides' namespaces

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "plansys2_combat_example/BattleView.hpp"
#include "plansys2_combat_example/World.hpp"
#include "plansys2_combat_example/msg/side_view.hpp"
#include "plansys2_combat_example/srv/cancel_action.hpp"
#include "plansys2_combat_example/srv/finish_action.hpp"
#include "plansys2_combat_example/srv/start_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2_combat_example
{

using StartAction = plansys2_combat_example::srv::StartAction;
using FinishAction = plansys2_combat_example::srv::FinishAction;
using CancelAction = plansys2_combat_example::srv::CancelAction;
using SideView = plansys2_combat_example::msg::SideView;
using RobotState = plansys2_combat_example::msg::RobotState;
using namespace std::chrono_literals;  // NOLINT

namespace
{
// Actions that take a robot from args[1] to args[2], and those that shoot args[1]
const std::set<std::string> kTravels = {"move", "advance", "assault_tank", "assault_infantry",
  "fly"};
const std::set<std::string> kShots = {"attack_infantry", "attack_tank", "bombard"};

std::vector<std::string> words(const std::string & s)
{
  std::istringstream in(s);
  std::vector<std::string> ret;
  for (std::string w; in >> w; ) {
    ret.push_back(w);
  }
  return ret;
}

std::string fact(const std::string & name, const std::vector<std::string> & args)
{
  std::string ret = "(" + name;
  for (const auto & a : args) {
    ret += " " + a;
  }
  return ret + ")";
}

RobotState state(const Robot & r, const std::string & zone)
{
  RobotState s;
  s.name = r.name;
  s.type = r.type;
  s.zone = zone;
  s.alive = r.alive;
  s.ammo = r.ammo;
  s.battery = r.battery;
  return s;
}
}  // namespace

class CombatArbiter : public rclcpp::Node
{
public:
  CombatArbiter()
  : rclcpp::Node("combat_arbiter"),
    world_(static_cast<unsigned int>(declare_parameter<int64_t>("seed", 0)))
  {
    time_scale_ = declare_parameter<double>("time_scale", 0.2);
    using Strings = std::vector<std::string>;
    for (const auto & z : declare_parameter<Strings>("zones", Strings{})) {
      world_.add_zone(z);
    }
    for (const auto & c : declare_parameter<Strings>("connections", Strings{})) {
      auto w = words(c);  // "a b distance"
      if (w.size() == 3) {
        world_.connect(w[0], w[1], std::stod(w[2]));
      }
    }
    for (const auto & z : declare_parameter<Strings>("control_points", Strings{})) {
      world_.add_control_point(z);
    }
    for (const auto & h : declare_parameter<Strings>("hit_probability", Strings{})) {
      auto w = words(h);  // "type probability"
      if (w.size() == 2) {
        world_.set_hit_probability(w[0], std::stod(w[1]));
      }
    }

    // Every side: its base, its army, and the zones it sees from the start
    std::map<std::string, Strings> explored;
    for (const auto & side : declare_parameter<Strings>("sides", Strings{"blue", "red"})) {
      world_.add_side(side, declare_parameter<std::string>(side + ".base", ""));
      for (const auto & r : declare_parameter<Strings>(side + ".robots", Strings{})) {
        auto w = words(r);  // "name type zone"
        if (w.size() == 3) {
          world_.add_robot(w[0], w[1], w[2], side);
        }
      }
      explored[side] = declare_parameter<Strings>(side + ".explored", Strings{world_.base(side)});
    }
    for (const auto & [side, zones] : explored) {
      for (const auto & z : zones) {
        world_.explore(side, z);
      }
    }

    // The battle in RViz, from the true state
    view_ = std::make_unique<BattleView>(this, world_);
    for (const auto & z : declare_parameter<Strings>("zone_positions", Strings{})) {
      auto w = words(z);  // "zone x y"
      if (w.size() == 3) {
        view_->set_zone_position(w[0], std::stod(w[1]), std::stod(w[2]));
      }
    }
    for (const auto & water : declare_parameter<Strings>("water", Strings{})) {
      auto w = words(water);  // "x1 y1 x2 y2 width", only drawn
      if (w.size() == 5) {
        view_->add_water(
          std::stod(w[0]), std::stod(w[1]), std::stod(w[2]), std::stod(w[3]), std::stod(w[4]));
      }
    }
    view_->set_viewpoint(
      declare_parameter<std::string>(
        "viewpoint", world_.sides().empty() ? "" : world_.sides().front()));

    for (const auto & side : world_.sides()) {
      view_pubs_[side] = create_publisher<SideView>(
        "/combat/" + side + "/view", rclcpp::QoS(1).transient_local().reliable());
    }
    publish();
    frames_timer_ = create_wall_timer(
      50ms, [this]() {
        if (rclcpp::ok()) {  // not while shutting down
          view_->publish_frames();
        }
      });

    start_srv_ = create_service<StartAction>(
      "/combat/start_action",
      [this](const std::shared_ptr<StartAction::Request> req,
      std::shared_ptr<StartAction::Response> res) {
        on_start(*req, *res);
      });
    finish_srv_ = create_service<FinishAction>(
      "/combat/finish_action",
      [this](const std::shared_ptr<FinishAction::Request> req,
      std::shared_ptr<FinishAction::Response> res) {
        on_finish(*req, *res);
      });
    cancel_srv_ = create_service<CancelAction>(
      "/combat/cancel_action",
      [this](const std::shared_ptr<CancelAction::Request> req,
      std::shared_ptr<CancelAction::Response>) {
        on_cancel(*req);
      });
  }

private:
  // An action under way, as the arbiter times it
  struct Ongoing
  {
    std::string action;
    rclcpp::Time leaves;   // when it sets off, after getting back from a cancelled move
    double duration;       // of the action itself, real seconds
  };

  void on_start(const StartAction::Request & req, StartAction::Response & res)
  {
    const auto & args = req.arguments;
    if (!world_.winner().empty()) {
      res.accepted = false;
      res.message = "the battle is over";
      return;
    }
    res.accepted = world_.start(req.side, req.action, args, res.message);
    if (res.accepted) {
      // A robot turning back from a cancelled move first gets to its zone
      const auto & robot = args[0];
      const auto now = this->now();
      double delay = 0.0;
      auto back = back_at_.find(robot);
      if (back != back_at_.end()) {
        delay = std::max(0.0, (back->second - now).seconds());
        back_at_.erase(back);
      }
      const double duration = *world_.duration(req.action, args) * time_scale_;
      ongoing_[robot] = {req.action, now + rclcpp::Duration::from_seconds(delay), duration};
      res.duration = delay + duration;
      if (kTravels.count(req.action) && args.size() == 3) {
        view_->start_motion(robot, args[1], args[2], duration, delay);
      }
      publish(req.side);  // ammo and battery
    }
    RCLCPP_INFO(
      get_logger(), "[%s] start %s: %s", req.side.c_str(), fact(req.action, args).c_str(),
      res.accepted ? "ok" : res.message.c_str());
  }

  void on_finish(const FinishAction::Request & req, FinishAction::Response & res)
  {
    const auto & args = req.arguments;
    const bool shot = kShots.count(req.action) && args.size() >= 2 &&
      world_.robots().count(args[1]);
    const bool target_alive = shot && world_.robots().at(args[1]).alive;
    auto out = world_.finish(req.side, req.action, args);
    if (!args.empty()) {
      ongoing_.erase(args[0]);
      view_->end_motion(args[0]);
    }
    if (target_alive) {
      view_->shot(args[0], args[1], !world_.robots().at(args[1]).alive);
    }
    res.success = out.success;
    res.message = out.message;
    for (const auto & e : out.discovered) {
      RCLCPP_WARN(
        get_logger(), "[%s] enemy %s (%s) seen in %s", req.side.c_str(), e.c_str(),
        world_.robots().at(e).type.c_str(), world_.robots().at(e).zone.c_str());
    }
    RCLCPP_INFO(
      get_logger(), "[%s] %s%s", req.side.c_str(), out.success ? "" : "FAILED: ",
      out.message.c_str());
    publish();  // a robot seen or destroyed changes what every side knows
    const auto winner = world_.winner();
    if (!winner.empty() && announced_.empty()) {
      announced_ = winner;
      RCLCPP_WARN(get_logger(), "THE BATTLE IS OVER: %s wins", winner.c_str());
    }
  }

  // PlanSys2 cancelled the action: a robot on the road turns back, as fast as it came
  void on_cancel(const CancelAction::Request & req)
  {
    if (req.arguments.empty()) {
      return;
    }
    const auto & robot = req.arguments[0];
    auto it = ongoing_.find(robot);
    auto r = world_.robots().find(robot);
    if (it == ongoing_.end() || it->second.action != req.action || r == world_.robots().end() ||
      r->second.side != req.side)
    {
      return;
    }
    const auto now = this->now();
    double back = -1.0;  // seconds to get back to its zone, if it is out of it
    if (it->second.leaves > now) {
      // Cancelled while still coming back from a previous cancel
      back = (it->second.leaves - now).seconds();
    } else if (kTravels.count(req.action)) {
      // On the road: back as fast as it came
      back = std::min((now - it->second.leaves).seconds(), it->second.duration);
    }
    if (back >= 0.0) {
      back_at_[robot] = now + rclcpp::Duration::from_seconds(back);
      view_->retreat(robot, back);
    }
    std::string what = fact(req.action, req.arguments) + " cancelled";
    if (back >= 0.0) {
      what += ", back to " + r->second.zone + " in " + std::to_string(back) + " s";
    }
    RCLCPP_WARN(get_logger(), "[%s] %s", req.side.c_str(), what.c_str());
    ongoing_.erase(it);
  }

  // What each side knows, latched for its controller
  void publish(const std::string & only = "")
  {
    for (const auto & side : world_.sides()) {
      if (!only.empty() && side != only) {
        continue;
      }
      SideView v;
      v.side = side;
      v.winner = world_.winner();
      v.base = world_.base(side);
      v.zones = world_.zones();
      for (const auto & [ab, d] : world_.connections()) {
        v.connection_from.push_back(ab.first);
        v.connection_to.push_back(ab.second);
        v.connection_distance.push_back(d);
      }
      v.control_points = world_.control_points();
      for (const auto & [name, r] : world_.robots()) {
        if (r.side == side) {
          v.robots.push_back(state(r, r.zone));
        }
      }
      for (const auto & [name, zone] : world_.known_enemies(side)) {
        v.enemies.push_back(state(world_.robots().at(name), zone));
      }
      for (const auto & z : world_.explored(side)) {
        v.explored.push_back(z);
      }
      for (const auto & z : world_.zones()) {
        const auto owner = world_.owner(z);
        if (owner == side) {
          v.owned.push_back(z);
        } else if (!owner.empty()) {
          v.enemy_owned.push_back(z);
        }
      }
      view_pubs_[side]->publish(v);
    }
    view_->publish_markers();
  }

  World world_;
  std::string announced_;  // the winner, once told
  double time_scale_;
  std::unique_ptr<BattleView> view_;
  std::map<std::string, Ongoing> ongoing_;      // robot -> action under way
  std::map<std::string, rclcpp::Time> back_at_;  // robot -> back in its zone after a cancel
  std::map<std::string, rclcpp::Publisher<SideView>::SharedPtr> view_pubs_;
  rclcpp::TimerBase::SharedPtr frames_timer_;
  rclcpp::Service<StartAction>::SharedPtr start_srv_;
  rclcpp::Service<FinishAction>::SharedPtr finish_srv_;
  rclcpp::Service<CancelAction>::SharedPtr cancel_srv_;
};

}  // namespace plansys2_combat_example

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plansys2_combat_example::CombatArbiter>());
  rclcpp::shutdown();
  return 0;
}

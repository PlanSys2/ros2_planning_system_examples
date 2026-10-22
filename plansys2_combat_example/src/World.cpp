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

#include "plansys2_combat_example/World.hpp"

#include <algorithm>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace plansys2_combat_example
{

namespace
{
const std::set<std::string> kMoves = {"move", "advance", "assault_tank", "assault_infantry"};
}  // namespace

World::World(unsigned int seed)
: rng_(seed)
{
}

const std::map<std::string, Kind> &
World::kinds()
{
  // speed, ammo, value, op_cost, shot_cost, hit
  static const std::map<std::string, Kind> table = {
    {"drone", {0.0, 0, 5000.0, 50.0, 0.0, 0.0}},
    {"infantry", {1.0, 5, 20000.0, 100.0, 10.0, 0.7}},
    {"tank", {0.5, 4, 150000.0, 500.0, 300.0, 0.8}},
    {"artillery", {0.4, 3, 80000.0, 300.0, 1000.0, 0.6}},
  };
  return table;
}


void
World::add_zone(const std::string & zone)
{
  zones_.push_back(zone);
}

void
World::connect(const std::string & a, const std::string & b, double distance)
{
  connections_[{a, b}] = distance;
  connections_[{b, a}] = distance;
}

void
World::add_side(const std::string & side, const std::string & base)
{
  if (std::find(sides_.begin(), sides_.end(), side) == sides_.end()) {
    sides_.push_back(side);
  }
  bases_[side] = base;
}

void
World::add_robot(
  const std::string & name, const std::string & type, const std::string & zone,
  const std::string & side)
{
  Robot r;
  r.name = name;
  r.type = type;
  r.zone = zone;
  r.side = side;
  r.ammo = kinds().at(type).ammo;
  robots_[name] = r;
}

std::string
World::base(const std::string & side) const
{
  auto it = bases_.find(side);
  return it != bases_.end() ? it->second : "";
}

double
World::hit_probability(const std::string & type) const
{
  auto it = hit_.find(type);
  return it != hit_.end() ? it->second : kinds().at(type).hit;
}

std::vector<std::string>
World::enemies_at(const std::string & side, const std::string & zone) const
{
  std::vector<std::string> ret;
  for (const auto & [name, r] : robots_) {
    if (r.side != side && r.alive && r.zone == zone) {
      ret.push_back(name);
    }
  }
  return ret;
}

std::vector<std::string>
World::explore(const std::string & side, const std::string & zone)
{
  explored_[side].insert(zone);
  std::vector<std::string> discovered;
  auto & known = known_[side];
  // Those believed here that are gone: where they are now is not known
  for (auto & [enemy, where] : known) {
    if (where == zone && robots_.at(enemy).zone != zone) {
      where.clear();
    }
  }
  for (const auto & e : enemies_at(side, zone)) {
    if (!known.count(e)) {
      discovered.push_back(e);
    }
    known[e] = zone;
  }
  return discovered;
}

void
World::seen_arriving(const Robot & r)
{
  for (const auto & [name, other] : robots_) {
    if (other.side != r.side && other.alive && other.zone == r.zone) {
      known_[other.side][r.name] = r.zone;
    }
  }
}

std::set<std::string>
World::explored(const std::string & side) const
{
  auto it = explored_.find(side);
  return it != explored_.end() ? it->second : std::set<std::string>{};
}

std::map<std::string, std::string>
World::known_enemies(const std::string & side) const
{
  auto it = known_.find(side);
  return it != known_.end() ? it->second : std::map<std::string, std::string>{};
}

std::set<std::string>
World::owned(const std::string & side) const
{
  std::set<std::string> ret;
  for (const auto & [zone, s] : owner_) {
    if (s == side) {
      ret.insert(zone);
    }
  }
  return ret;
}

std::string
World::owner(const std::string & zone) const
{
  auto it = owner_.find(zone);
  return it != owner_.end() ? it->second : "";
}

std::string
World::winner() const
{
  // Every control point in the hands of one side
  if (!control_points_.empty()) {
    const auto first = owner(control_points_.front());
    bool all = !first.empty();
    for (const auto & z : control_points_) {
      all = all && owner(z) == first;
    }
    if (all) {
      return first;
    }
  }
  // Only one side with robots left
  std::set<std::string> alive;
  for (const auto & [name, r] : robots_) {
    if (r.alive) {
      alive.insert(r.side);
    }
  }
  if (alive.size() == 1 && sides_.size() > 1) {
    return *alive.begin();
  }
  return "";
}

std::optional<double>
World::duration(const std::string & action, const std::vector<std::string> & args) const
{
  if (args.empty()) {
    return std::nullopt;
  }
  auto robot = robots_.find(args[0]);
  if (robot == robots_.end()) {
    return std::nullopt;
  }
  if (kMoves.count(action) && args.size() == 3) {
    auto c = connections_.find({args[1], args[2]});
    const double speed = kinds().at(robot->second.type).speed;
    if (c == connections_.end() || speed <= 0.0) {
      return std::nullopt;
    }
    return c->second / speed;
  }
  static const std::map<std::string, double> fixed = {
    {"fly", 5.0}, {"survey", 3.0}, {"spot", 12.0}, {"recharge", 20.0},
    {"attack_infantry", 3.0}, {"attack_tank", 3.0}, {"bombard", 4.0}, {"capture", 5.0}};
  auto f = fixed.find(action);
  if (f == fixed.end()) {
    return std::nullopt;
  }
  return f->second;
}

bool
World::start(
  const std::string & side, const std::string & action, const std::vector<std::string> & args,
  std::string & why)
{
  if (!duration(action, args)) {
    why = "not an action of a robot";
    return false;
  }
  Robot & r = robots_.at(args[0]);
  if (r.side != side) {
    why = r.name + " is not a robot of " + side;
    return false;
  }
  if (!r.alive) {
    why = r.name + " is destroyed";
    return false;
  }
  // The robot must be where the action starts: its plan may be older than the world
  static const std::map<std::string, size_t> where = {
    {"move", 1}, {"advance", 1}, {"assault_tank", 1}, {"assault_infantry", 1}, {"fly", 1},
    {"survey", 1}, {"spot", 1}, {"recharge", 1}, {"capture", 1},
    {"attack_infantry", 2}, {"attack_tank", 2}, {"bombard", 2}};
  auto w = where.find(action);
  if (w != where.end() && w->second < args.size() && r.zone != args[w->second]) {
    why = r.name + " is in " + r.zone + ", not in " + args[w->second];
    return false;
  }
  // Resources go at the start, as in the domain
  static const std::map<std::string, double> battery = {
    {"fly", 10.0}, {"survey", 5.0}, {"spot", 10.0}};
  auto b = battery.find(action);
  if (b != battery.end()) {
    if (r.battery < b->second) {
      why = r.name + " has not enough battery";
      return false;
    }
    r.battery -= b->second;
  }
  if (action == "attack_infantry" || action == "attack_tank" || action == "bombard") {
    if (r.ammo < 1) {
      why = r.name + " has no ammunition";
      return false;
    }
    r.ammo--;
  }
  return true;
}

Outcome
World::shoot(const Robot & attacker, const std::string & target, const std::string & zone)
{
  Outcome out;
  auto t = robots_.find(target);
  if (t == robots_.end() || t->second.side == attacker.side) {
    out.success = false;
    out.message = target + " is not an enemy of " + attacker.name;
    return out;
  }
  if (!t->second.alive) {
    out.message = target + " was already destroyed";
    return out;
  }
  if (t->second.zone != zone) {
    // The side sees it is not there: where it is now is not known
    auto & known = known_[attacker.side];
    if (known.count(target) && known[target] == zone) {
      known[target].clear();
    }
    out.success = false;
    out.message = target + " is not in " + zone;
    return out;
  }
  std::uniform_real_distribution<double> roll(0.0, 1.0);
  if (roll(rng_) < hit_probability(attacker.type)) {
    t->second.alive = false;
    out.message = attacker.name + " destroyed " + target;
  } else {
    out.success = false;
    out.message = attacker.name + " missed " + target;
  }
  return out;
}

Outcome
World::finish(
  const std::string & side, const std::string & action, const std::vector<std::string> & args)
{
  Outcome out;
  if (!duration(action, args)) {
    out.success = false;
    out.message = "not an action of a robot";
    return out;
  }
  Robot & r = robots_.at(args[0]);
  if (r.side != side) {
    out.success = false;
    out.message = r.name + " is not a robot of " + side;
    return out;
  }
  if (!r.alive) {
    out.success = false;
    out.message = r.name + " is destroyed";
    return out;
  }

  if (kMoves.count(action)) {
    // A ground robot sees the zone it arrives at, and is seen by the enemies there
    r.zone = args[2];
    out.discovered = explore(side, args[2]);
    seen_arriving(r);
    out.message = r.name + " reached " + args[2];
  } else if (action == "fly") {
    r.zone = args[2];
    seen_arriving(r);
    out.message = r.name + " flew to " + args[2];
  } else if (action == "survey") {
    out.discovered = explore(side, args[1]);
    out.message = r.name + " surveyed " + args[1];
  } else if (action == "recharge") {
    r.battery = 100.0;
    out.message = r.name + " recharged";
  } else if (action == "attack_infantry" || action == "attack_tank") {
    out = shoot(r, args[1], args[2]);
  } else if (action == "bombard") {
    out = shoot(r, args[1], args[3]);
  } else if (action == "capture") {
    // Only enemies on the ground hold a zone: drones fly over it
    std::vector<std::string> defenders;
    for (const auto & e : enemies_at(side, args[1])) {
      if (robots_.at(e).type != "drone") {
        defenders.push_back(e);
      }
    }
    if (!defenders.empty()) {
      // The zone was not clear: the capture fails and the enemies there are seen
      out.success = false;
      out.discovered = explore(side, args[1]);
      out.message = "enemies in " + args[1];
    } else {
      owner_[args[1]] = side;
      out.message = r.name + " captured " + args[1];
    }
  } else {
    out.message = r.name + " " + action;
  }
  return out;
}

}  // namespace plansys2_combat_example

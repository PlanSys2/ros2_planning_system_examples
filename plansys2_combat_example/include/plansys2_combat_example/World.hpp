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

#ifndef PLANSYS2_COMBAT_EXAMPLE__WORLD_HPP_
#define PLANSYS2_COMBAT_EXAMPLE__WORLD_HPP_

#include <map>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace plansys2_combat_example
{

// What a kind of robot is like, as in pddl/combat.pddl and the problems
struct Kind
{
  double speed;       // meters per second; 0 for drones, that fly at a fixed time
  int ammo;
  double value;       // euros
  double op_cost;     // euros per action
  double shot_cost;   // euros per shot
  double hit;         // probability that one of its shots destroys the target
};

struct Robot
{
  std::string name;
  std::string type;   // drone, infantry, tank, artillery
  std::string zone;
  std::string side;   // the army it belongs to
  bool alive = true;
  int ammo = 0;
  double battery = 100.0;
};

// What an action really did
struct Outcome
{
  bool success = true;
  std::string message;
  std::vector<std::string> discovered;  // enemies its side sees for the first time
};

// The true state of the battle, and what each side knows of its enemies. No ROS here
class World
{
public:
  explicit World(unsigned int seed = 0);

  static const std::map<std::string, Kind> & kinds();

  void add_zone(const std::string & zone);
  void connect(const std::string & a, const std::string & b, double distance);  // both ways
  void add_control_point(const std::string & zone) {control_points_.push_back(zone);}
  void add_side(const std::string & side, const std::string & base);
  void add_robot(
    const std::string & name, const std::string & type, const std::string & zone,
    const std::string & side);
  void set_hit_probability(const std::string & type, double p) {hit_[type] = p;}

  // A side looks at a zone: the enemies there become known, where they are now.
  // Returns the ones it had never seen
  std::vector<std::string> explore(const std::string & side, const std::string & zone);

  // Seconds of the action as the domain defines them; empty if it is not an action of a robot
  std::optional<double> duration(const std::string & action, const std::vector<std::string> & args)
  const;

  // Resources the action uses at its start; false (and why) if the side's robot cannot do it
  bool start(
    const std::string & side, const std::string & action, const std::vector<std::string> & args,
    std::string & why);

  // The end of the action: where robots are, what they see, if shots hit, if zones are taken
  Outcome finish(
    const std::string & side, const std::string & action, const std::vector<std::string> & args);

  const std::vector<std::string> & zones() const {return zones_;}
  const std::map<std::pair<std::string, std::string>, double> & connections() const
  {
    return connections_;
  }
  const std::vector<std::string> & control_points() const {return control_points_;}
  const std::vector<std::string> & sides() const {return sides_;}
  std::string base(const std::string & side) const;
  const std::map<std::string, Robot> & robots() const {return robots_;}

  std::set<std::string> explored(const std::string & side) const;
  // Enemies the side has seen, and the zone where it saw them last; empty once it knows they
  // left it
  std::map<std::string, std::string> known_enemies(const std::string & side) const;
  std::set<std::string> owned(const std::string & side) const;
  std::string owner(const std::string & zone) const;  // empty if nobody holds it
  std::vector<std::string> enemies_at(const std::string & side, const std::string & zone) const;

  // The side that has won: it holds every control point, or it is the only one with robots left.
  // Empty while the battle goes on
  std::string winner() const;

private:
  double hit_probability(const std::string & type) const;
  Outcome shoot(const Robot & attacker, const std::string & target, const std::string & zone);
  void seen_arriving(const Robot & r);  // the other sides with robots there see it

  std::vector<std::string> zones_;
  std::map<std::pair<std::string, std::string>, double> connections_;
  std::vector<std::string> control_points_;
  std::vector<std::string> sides_;
  std::map<std::string, std::string> bases_;
  std::map<std::string, Robot> robots_;
  std::map<std::string, double> hit_;
  std::map<std::string, std::set<std::string>> explored_;                   // side -> zones
  std::map<std::string, std::map<std::string, std::string>> known_;         // side -> enemy -> zone
  std::map<std::string, std::string> owner_;                                // zone -> side
  std::mt19937 rng_;
};

}  // namespace plansys2_combat_example

#endif  // PLANSYS2_COMBAT_EXAMPLE__WORLD_HPP_

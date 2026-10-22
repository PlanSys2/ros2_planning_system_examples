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

#include <set>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_combat_example/World.hpp"

using plansys2_combat_example::World;

namespace
{
// The 3x3 map of the scenarios, our army at c_west, the enemy defense of s2
World make_world(double hit = -1.0)
{
  World w(1);
  const std::vector<std::vector<std::string>> grid = {
    {"n_west", "n_mid", "n_east"}, {"c_west", "c_mid", "c_east"}, {"s_west", "s_mid", "s_east"}};
  for (const auto & row : grid) {
    for (const auto & z : row) {
      w.add_zone(z);
    }
  }
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      if (c < 2) {w.connect(grid[r][c], grid[r][c + 1], 10.0);}
      if (r < 2) {w.connect(grid[r][c], grid[r + 1][c], 10.0);}
    }
  }
  for (const auto & z : {"n_mid", "c_mid", "s_mid"}) {
    w.add_control_point(z);
  }
  w.add_side("blue", "c_west");
  w.add_side("red", "c_east");
  w.add_robot("d1", "drone", "c_west", "blue");
  w.add_robot("i1", "infantry", "c_west", "blue");
  w.add_robot("t1", "tank", "c_west", "blue");
  w.add_robot("a1", "artillery", "c_west", "blue");
  w.add_robot("e_inf1", "infantry", "c_mid", "red");
  w.add_robot("e_tank1", "tank", "n_mid", "red");
  if (hit >= 0.0) {
    for (const auto & k : {"infantry", "tank", "artillery"}) {
      w.set_hit_probability(k, hit);
    }
  }
  return w;
}

bool start(World & w, const std::string & action, const std::vector<std::string> & args)
{
  std::string why;
  return w.start("blue", action, args, why);
}
}  // namespace

TEST(world, durations_follow_the_domain)
{
  auto w = make_world();
  EXPECT_DOUBLE_EQ(*w.duration("move", {"i1", "c_west", "c_mid"}), 10.0);   // 10 m at 1 m/s
  EXPECT_DOUBLE_EQ(*w.duration("advance", {"t1", "c_west", "c_mid"}), 20.0);  // 0.5 m/s
  EXPECT_DOUBLE_EQ(*w.duration("assault_tank", {"t1", "c_west", "n_west"}), 20.0);
  EXPECT_DOUBLE_EQ(*w.duration("move", {"a1", "c_west", "c_mid"}), 25.0);   // 0.4 m/s
  EXPECT_DOUBLE_EQ(*w.duration("fly", {"d1", "c_west", "s_east"}), 5.0);    // anywhere
  EXPECT_DOUBLE_EQ(*w.duration("survey", {"d1", "c_mid"}), 3.0);
  EXPECT_DOUBLE_EQ(*w.duration("spot", {"d1", "c_mid"}), 12.0);
  EXPECT_DOUBLE_EQ(*w.duration("recharge", {"d1", "c_west"}), 20.0);
  EXPECT_DOUBLE_EQ(*w.duration("attack_tank", {"t1", "e_tank1", "n_mid"}), 3.0);
  EXPECT_DOUBLE_EQ(*w.duration("bombard", {"a1", "e_tank1", "n_west", "n_mid"}), 4.0);
  EXPECT_DOUBLE_EQ(*w.duration("capture", {"i1", "c_mid"}), 5.0);

  EXPECT_FALSE(w.duration("move", {"i1", "c_west", "c_east"}));  // not connected
  EXPECT_FALSE(w.duration("move", {"d1", "c_west", "c_mid"}));   // drones fly
  EXPECT_DOUBLE_EQ(*w.duration("move", {"e_inf1", "c_mid", "c_west"}), 10.0);  // any side
  EXPECT_FALSE(w.duration("move", {"x9", "c_west", "c_mid"}));   // unknown robot
  EXPECT_FALSE(w.duration("dance", {"i1"}));                     // unknown action
  EXPECT_FALSE(w.duration("move", {}));
}

TEST(world, resources_go_at_the_start)
{
  auto w = make_world();
  // The drone: 100 of battery, 10 per flight, 10 per spot, 5 per survey
  EXPECT_TRUE(start(w, "fly", {"d1", "c_west", "c_mid"}));
  EXPECT_DOUBLE_EQ(w.robots().at("d1").battery, 90.0);
  w.finish("blue", "fly", {"d1", "c_west", "c_mid"});
  EXPECT_TRUE(start(w, "survey", {"d1", "c_mid"}));
  EXPECT_DOUBLE_EQ(w.robots().at("d1").battery, 85.0);
  for (int i = 0; i < 8; i++) {
    EXPECT_TRUE(start(w, "spot", {"d1", "c_mid"})) << i;
  }
  EXPECT_DOUBLE_EQ(w.robots().at("d1").battery, 5.0);
  std::string why;
  EXPECT_FALSE(w.start("blue", "spot", {"d1", "c_mid"}, why));  // 5 left, 10 needed
  EXPECT_NE(why.find("battery"), std::string::npos);
  EXPECT_TRUE(start(w, "survey", {"d1", "c_mid"}));      // exactly 5
  EXPECT_DOUBLE_EQ(w.robots().at("d1").battery, 0.0);
  w.finish("blue", "recharge", {"d1", "c_west"});
  EXPECT_DOUBLE_EQ(w.robots().at("d1").battery, 100.0);

  // The tank: 4 shots, from where the enemy tank is
  w.finish("blue", "advance", {"t1", "c_west", "n_west"});
  w.finish("blue", "move", {"t1", "n_west", "n_mid"});
  for (int i = 0; i < 4; i++) {
    EXPECT_TRUE(start(w, "attack_tank", {"t1", "e_tank1", "n_mid"})) << i;
  }
  EXPECT_FALSE(w.start("blue", "attack_tank", {"t1", "e_tank1", "n_mid"}, why));
  EXPECT_NE(why.find("ammunition"), std::string::npos);
  EXPECT_EQ(w.robots().at("t1").ammo, 0);
}

TEST(world, arriving_and_surveying_reveal_enemies_once)
{
  auto w = make_world();
  EXPECT_TRUE(w.known_enemies("blue").empty());

  // Flying does not reveal; surveying does
  auto out = w.finish("blue", "fly", {"d1", "c_west", "c_mid"});
  EXPECT_TRUE(out.success);
  EXPECT_TRUE(out.discovered.empty());
  EXPECT_EQ(w.robots().at("d1").zone, "c_mid");
  out = w.finish("blue", "survey", {"d1", "c_mid"});
  EXPECT_EQ(out.discovered, (std::vector<std::string>{"e_inf1"}));
  EXPECT_TRUE(w.explored("blue").count("c_mid"));

  // Arriving reveals what is there, but only what was not known
  out = w.finish("blue", "advance", {"i1", "c_west", "c_mid"});
  EXPECT_TRUE(out.success);  // the robot gets there, enemies or not
  EXPECT_TRUE(out.discovered.empty());
  EXPECT_EQ(w.robots().at("i1").zone, "c_mid");
  out = w.finish("blue", "advance", {"t1", "c_west", "n_west"});
  EXPECT_TRUE(out.discovered.empty());  // nobody in n_west
  out = w.finish("blue", "move", {"t1", "n_west", "n_mid"});
  EXPECT_EQ(out.discovered, (std::vector<std::string>{"e_tank1"}));
  EXPECT_EQ(w.known_enemies("blue").size(), 2u);

  // explore() from the start, as the arbiter does with the zones known at the beginning
  auto w2 = make_world();
  EXPECT_EQ(w2.explore("blue", "n_mid"), (std::vector<std::string>{"e_tank1"}));
  EXPECT_TRUE(w2.explore("blue", "n_mid").empty());
}

TEST(world, shots_hit_or_miss)
{
  // Always hit
  auto w = make_world(1.0);
  auto out = w.finish("blue", "attack_tank", {"t1", "e_tank1", "n_mid"});
  EXPECT_TRUE(out.success);
  EXPECT_FALSE(w.robots().at("e_tank1").alive);
  EXPECT_TRUE(w.enemies_at("blue", "n_mid").empty());
  // Shooting what is already destroyed is not a failure
  out = w.finish("blue", "attack_tank", {"t1", "e_tank1", "n_mid"});
  EXPECT_TRUE(out.success);
  EXPECT_NE(out.message.find("already"), std::string::npos);

  // Never hit: the action fails, the target lives
  auto w0 = make_world(0.0);
  out = w0.finish("blue", "bombard", {"a1", "e_inf1", "c_west", "c_mid"});
  EXPECT_FALSE(out.success);
  EXPECT_NE(out.message.find("missed"), std::string::npos);
  EXPECT_TRUE(w0.robots().at("e_inf1").alive);

  // A target that is not where the plan believes
  out = w.finish("blue", "attack_infantry", {"i1", "e_inf1", "s_mid"});
  EXPECT_FALSE(out.success);
  EXPECT_TRUE(w.robots().at("e_inf1").alive);

  // Bombard aims at the zone it fires at, the last argument
  out = w.finish("blue", "bombard", {"a1", "e_inf1", "c_west", "c_mid"});
  EXPECT_TRUE(out.success);
  EXPECT_FALSE(w.robots().at("e_inf1").alive);
}

TEST(world, shots_follow_the_probability)
{
  auto w = make_world(0.5);
  int hits = 0;
  for (int i = 0; i < 400; i++) {
    w.add_robot("target", "infantry", "c_mid", "red");
    hits += w.finish("blue", "attack_infantry", {"i1", "target", "c_mid"}).success ? 1 : 0;
  }
  EXPECT_GT(hits, 150);
  EXPECT_LT(hits, 250);
}

TEST(world, capture_needs_a_clear_zone)
{
  auto w = make_world(1.0);
  // Not clear: fails and shows the enemy
  auto out = w.finish("blue", "capture", {"i1", "c_mid"});
  EXPECT_FALSE(out.success);
  EXPECT_EQ(out.discovered, (std::vector<std::string>{"e_inf1"}));
  EXPECT_FALSE(w.owned("blue").count("c_mid"));

  // Clear after the enemy is destroyed
  EXPECT_TRUE(w.finish("blue", "attack_infantry", {"i1", "e_inf1", "c_mid"}).success);
  out = w.finish("blue", "capture", {"i1", "c_mid"});
  EXPECT_TRUE(out.success);
  EXPECT_TRUE(w.owned("blue").count("c_mid"));

  // A zone nobody defends
  EXPECT_TRUE(w.finish("blue", "capture", {"i1", "s_mid"}).success);
  EXPECT_EQ(w.owned("blue").size(), 2u);
}

TEST(world, a_side_only_commands_its_robots)
{
  auto w = make_world(1.0);
  std::string why;
  EXPECT_FALSE(w.start("blue", "move", {"e_inf1", "c_mid", "c_west"}, why));
  EXPECT_NE(why.find("not a robot of blue"), std::string::npos);
  EXPECT_FALSE(w.finish("blue", "move", {"e_inf1", "c_mid", "c_west"}).success);
  EXPECT_EQ(w.robots().at("e_inf1").zone, "c_mid");
  EXPECT_FALSE(w.start("red", "move", {"i1", "c_west", "c_mid"}, why));
  EXPECT_FALSE(w.finish("red", "capture", {"i1", "c_west"}).success);
  EXPECT_FALSE(w.start("green", "move", {"i1", "c_west", "c_mid"}, why));  // no such side
  // Unknown robots and actions fail without changing anything
  EXPECT_FALSE(w.start("blue", "move", {"x9", "c_west", "c_mid"}, why));
  EXPECT_FALSE(w.finish("blue", "move", {"x9", "c_west", "c_mid"}).success);
  EXPECT_FALSE(w.finish("blue", "dance", {"i1"}).success);
  EXPECT_EQ(w.robots().at("i1").zone, "c_west");
  // Nothing was spent by the rejected starts
  EXPECT_EQ(w.robots().at("i1").ammo, 5);
}

TEST(world, no_friendly_fire)
{
  auto w = make_world(1.0);
  w.add_robot("i9", "infantry", "c_west", "blue");
  auto out = w.finish("blue", "attack_infantry", {"i1", "i9", "c_west"});
  EXPECT_FALSE(out.success);
  EXPECT_TRUE(w.robots().at("i9").alive);
  // Unknown targets are not hit either
  EXPECT_FALSE(w.finish("blue", "attack_infantry", {"i1", "x9", "c_west"}).success);
}

TEST(world, both_sides_fight)
{
  auto w = make_world(1.0);
  EXPECT_EQ(w.sides(), (std::vector<std::string>{"blue", "red"}));
  EXPECT_EQ(w.base("red"), "c_east");
  EXPECT_EQ(w.base("green"), "");

  // Red moves into c_west, where the blue army is: it sees blue, and blue sees it
  auto out = w.finish("red", "move", {"e_inf1", "c_mid", "c_west"});
  EXPECT_TRUE(out.success);
  EXPECT_EQ(out.discovered.size(), 4u);  // d1, i1, t1, a1
  EXPECT_TRUE(w.known_enemies("red").count("t1"));
  EXPECT_EQ(w.known_enemies("blue").at("e_inf1"), "c_west");
  EXPECT_TRUE(w.explored("red").count("c_west"));
  EXPECT_EQ(w.enemies_at("red", "c_west").size(), 4u);
  EXPECT_EQ(w.enemies_at("blue", "c_west"), (std::vector<std::string>{"e_inf1"}));

  // It walks away: blue keeps the zone where it saw it last
  EXPECT_TRUE(w.finish("red", "move", {"e_inf1", "c_west", "s_west"}).success);
  EXPECT_EQ(w.known_enemies("blue").at("e_inf1"), "c_west");
  // A blue survey of s_west finds it again, but it is not new
  w.add_robot("d2", "drone", "s_west", "blue");
  out = w.finish("blue", "survey", {"d2", "s_west"});
  EXPECT_TRUE(out.discovered.empty());
  EXPECT_EQ(w.known_enemies("blue").at("e_inf1"), "s_west");

  // Red shoots back
  out = w.finish("red", "attack_infantry", {"e_inf1", "d2", "s_west"});
  EXPECT_TRUE(out.success);
  EXPECT_FALSE(w.robots().at("d2").alive);
  EXPECT_FALSE(w.start("blue", "survey", {"d2", "s_west"}, out.message));
}

TEST(world, control_points_change_hands)
{
  auto w = make_world(1.0);
  EXPECT_EQ(w.owner("s_mid"), "");
  EXPECT_TRUE(w.finish("blue", "capture", {"i1", "s_mid"}).success);
  EXPECT_EQ(w.owner("s_mid"), "blue");
  EXPECT_TRUE(w.owned("red").empty());

  // Red takes it back once blue has left
  EXPECT_TRUE(w.finish("blue", "move", {"i1", "s_mid", "s_west"}).success);
  w.add_robot("e_inf2", "infantry", "s_mid", "red");
  EXPECT_TRUE(w.finish("red", "capture", {"e_inf2", "s_mid"}).success);
  EXPECT_EQ(w.owner("s_mid"), "red");
  EXPECT_TRUE(w.owned("blue").empty());
  EXPECT_EQ(w.owned("red"), (std::set<std::string>{"s_mid"}));

  // A blue robot in the zone stops a red capture, and red sees it
  w.add_robot("i3", "infantry", "n_west", "blue");
  w.add_robot("e_inf3", "infantry", "n_west", "red");
  w.add_control_point("n_west");
  auto out = w.finish("red", "capture", {"e_inf3", "n_west"});
  EXPECT_FALSE(out.success);
  EXPECT_EQ(out.discovered, (std::vector<std::string>{"i3"}));
  EXPECT_EQ(w.owner("n_west"), "");
}

TEST(world, a_fog_battle)
{
  // The base sees nothing; the enemy is found while fighting
  auto w = make_world(1.0);
  EXPECT_TRUE(w.explore("blue", "c_west").empty());
  // Infantry walks into c_mid blind: it gets there and sees the enemy
  ASSERT_TRUE(start(w, "advance", {"i1", "c_west", "c_mid"}));
  auto out = w.finish("blue", "advance", {"i1", "c_west", "c_mid"});
  EXPECT_EQ(out.discovered, (std::vector<std::string>{"e_inf1"}));
  // The plan wanted to capture: not yet
  EXPECT_FALSE(w.finish("blue", "capture", {"i1", "c_mid"}).success);
  // Replanned: shoot, then capture
  ASSERT_TRUE(start(w, "attack_infantry", {"i1", "e_inf1", "c_mid"}));
  EXPECT_TRUE(w.finish("blue", "attack_infantry", {"i1", "e_inf1", "c_mid"}).success);
  EXPECT_EQ(w.robots().at("i1").ammo, 4);
  EXPECT_TRUE(w.finish("blue", "capture", {"i1", "c_mid"}).success);
  // The drone finds the tank, the artillery destroys it while the drone spots
  EXPECT_TRUE(w.finish("blue", "fly", {"d1", "c_west", "n_mid"}).success);
  EXPECT_EQ(w.finish("blue", "survey", {"d1", "n_mid"}).discovered,
    (std::vector<std::string>{"e_tank1"}));
  EXPECT_TRUE(w.finish("blue", "move", {"a1", "c_west", "c_mid"}).success);
  ASSERT_TRUE(start(w, "bombard", {"a1", "e_tank1", "c_mid", "n_mid"}));
  EXPECT_TRUE(w.finish("blue", "bombard", {"a1", "e_tank1", "c_mid", "n_mid"}).success);
  EXPECT_TRUE(w.finish("blue", "capture", {"i1", "n_mid"}).success);
  EXPECT_EQ(w.owned("blue").size(), 2u);
  EXPECT_TRUE(w.enemies_at("blue", "n_mid").empty());
}

TEST(world, the_battle_ends_with_a_winner)
{
  // Holding every control point
  auto w = make_world(1.0);
  EXPECT_EQ(w.winner(), "");
  EXPECT_TRUE(w.finish("blue", "attack_infantry", {"i1", "e_inf1", "c_mid"}).success);
  EXPECT_TRUE(w.finish("blue", "capture", {"i1", "c_mid"}).success);
  EXPECT_TRUE(w.finish("blue", "capture", {"i1", "s_mid"}).success);
  EXPECT_EQ(w.winner(), "");  // n_mid still free
  w.add_robot("e_inf9", "infantry", "c_mid", "red");
  EXPECT_TRUE(w.finish("red", "capture", {"e_tank1", "n_mid"}).success);
  EXPECT_EQ(w.winner(), "");  // split between both
  EXPECT_TRUE(w.finish("blue", "attack_tank", {"t1", "e_tank1", "n_mid"}).success);
  EXPECT_TRUE(w.finish("blue", "capture", {"i1", "n_mid"}).success);
  EXPECT_EQ(w.winner(), "blue");

  // The only side with robots left
  auto w2 = make_world(1.0);
  EXPECT_TRUE(w2.finish("blue", "attack_infantry", {"i1", "e_inf1", "c_mid"}).success);
  EXPECT_EQ(w2.winner(), "");
  EXPECT_TRUE(w2.finish("blue", "attack_tank", {"t1", "e_tank1", "n_mid"}).success);
  EXPECT_EQ(w2.winner(), "blue");
}

TEST(world, drones_do_not_hold_a_zone)
{
  auto w = make_world(1.0);
  w.add_robot("e_drone", "drone", "s_mid", "red");
  // An enemy drone over s_mid does not stop the capture
  auto out = w.finish("blue", "capture", {"i1", "s_mid"});
  EXPECT_TRUE(out.success) << out.message;
  EXPECT_EQ(w.owner("s_mid"), "blue");
  // An enemy on the ground does
  out = w.finish("blue", "capture", {"i1", "c_mid"});
  EXPECT_FALSE(out.success);
}

TEST(world, a_side_learns_that_an_enemy_left)
{
  auto w = make_world(1.0);
  EXPECT_EQ(w.explore("blue", "c_mid"), (std::vector<std::string>{"e_inf1"}));
  EXPECT_EQ(w.known_enemies("blue").at("e_inf1"), "c_mid");

  // It leaves: blue still believes it is in c_mid
  EXPECT_TRUE(w.finish("red", "move", {"e_inf1", "c_mid", "c_east"}).success);
  EXPECT_EQ(w.known_enemies("blue").at("e_inf1"), "c_mid");

  // A shot at c_mid misses it, and blue learns it is not there
  EXPECT_FALSE(w.finish("blue", "attack_infantry", {"i1", "e_inf1", "c_mid"}).success);
  EXPECT_EQ(w.known_enemies("blue").at("e_inf1"), "");

  // Seen again elsewhere
  w.explore("blue", "c_east");
  EXPECT_EQ(w.known_enemies("blue").at("e_inf1"), "c_east");

  // It leaves again, and a survey of where it was shows it gone
  EXPECT_TRUE(w.finish("red", "move", {"e_inf1", "c_east", "s_east"}).success);
  w.explore("blue", "c_east");
  EXPECT_EQ(w.known_enemies("blue").at("e_inf1"), "");
}

TEST(world, an_action_starts_where_its_robot_is)
{
  // A plan older than the world: the robot is not where the action starts
  auto w = make_world(1.0);
  std::string why;
  EXPECT_FALSE(w.start("blue", "move", {"i1", "c_mid", "s_mid"}, why));
  EXPECT_NE(why.find("i1 is in c_west, not in c_mid"), std::string::npos) << why;
  EXPECT_FALSE(w.start("blue", "capture", {"i1", "c_mid"}, why));
  EXPECT_FALSE(w.start("blue", "attack_infantry", {"i1", "e_inf1", "c_mid"}, why));
  EXPECT_FALSE(w.start("blue", "survey", {"d1", "c_mid"}, why));
  // Nothing was spent by the rejected starts
  EXPECT_EQ(w.robots().at("i1").ammo, 5);
  EXPECT_DOUBLE_EQ(w.robots().at("d1").battery, 100.0);
  // From where it is: artillery fires from its zone at the next one
  EXPECT_TRUE(w.start("blue", "move", {"i1", "c_west", "c_mid"}, why)) << why;
  EXPECT_TRUE(w.start("blue", "bombard", {"a1", "e_inf1", "c_west", "c_mid"}, why)) << why;
}

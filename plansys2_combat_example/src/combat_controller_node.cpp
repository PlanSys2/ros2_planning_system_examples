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

// A side's commander, in the side's namespace with its own PlanSys2: turns what the arbiter
// says it sees into knowledge, plans, executes, and replans whenever the battle does not go as
// planned (a miss, enemies found, a zone not clear) until the goal is reached

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "kairos/plan.hpp"
#include "kairos/validate.hpp"
#include "plansys2_combat_example/Chronicle.hpp"
#include "plansys2_combat_example/World.hpp"
#include "plansys2_combat_example/msg/side_view.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT
using SideView = plansys2_combat_example::msg::SideView;

namespace
{
std::string fact(const std::string & name, const std::vector<std::string> & args)
{
  std::string ret = "(" + name;
  for (const auto & a : args) {
    ret += " " + a;
  }
  return ret + ")";
}

std::string value(const std::string & function, const std::vector<std::string> & args, double v)
{
  return "(= " + fact(function, args) + " " + std::to_string(v) + ")";
}

// What a plan costs from a problem (its metric, the euros spent; else its makespan), checked by
// Kairos's validator; empty if it does not reach the goal from there
std::optional<double> cost(
  const std::string & domain, const std::string & problem, const plansys2_msgs::msg::Plan & plan)
{
  if (plan.items.empty()) {
    return std::nullopt;
  }
  double start = plan.items.front().time;
  for (const auto & item : plan.items) {
    start = std::min(start, static_cast<double>(item.time));
  }
  kairos::Plan steps;
  steps.temporal = true;
  for (const auto & item : plan.items) {
    std::istringstream words(item.action.substr(1, item.action.rfind(')') - 1));
    kairos::PlanStep step;
    step.time = item.time - start;  // the remaining plan goes on from now
    step.duration = item.duration;
    words >> step.name;
    for (std::string arg; words >> arg; ) {
      step.args.push_back(arg);
    }
    steps.steps.push_back(step);
  }
  std::stable_sort(
    steps.steps.begin(), steps.steps.end(),
    [](const auto & a, const auto & b) {return a.time < b.time;});
  const auto report = kairos::validate_plan(domain, problem, steps);
  if (!report.valid()) {
    return std::nullopt;
  }
  return report.metric ? *report.metric : report.makespan;
}

// Predicates that change while fighting; the rest (the map, mine) are set once
const std::set<std::string> kDynamic = {
  "at", "available", "hostile", "destroyed", "explored", "owned", "enemy_owned", "spotted"};
}  // namespace

class CombatController : public rclcpp::Node
{
public:
  CombatController()
  : rclcpp::Node("combat_controller")
  {
    std::string ns = get_namespace();
    ns.erase(0, ns.find_first_not_of('/'));  // /blue -> blue
    side_ = declare_parameter<std::string>("side", ns);
    goal_mode_ = declare_parameter<std::string>("goal", "capture");  // capture | total
    metric_ = declare_parameter<std::string>("metric", "minimize (spent)");
    max_rounds_ = declare_parameter<int>("max_rounds", 100);
    retry_period_ = declare_parameter<double>("retry_period", 5.0);  // seconds, with no plan
    better_by_ = declare_parameter<double>("better_by", 0.05);  // a new plan, cheaper by this
    assign_objectives_ = declare_parameter<bool>("assign_objectives", true);
    chronicle_ = std::make_unique<plansys2_combat_example::Chronicle>(
      get_logger(), side_, declare_parameter<bool>("colors", true));

    view_sub_ = create_subscription<SideView>(
      "/combat/" + side_ + "/view", rclcpp::QoS(1).transient_local().reliable(),
      [this](SideView::SharedPtr msg) {view_ = msg; new_view_ = true;});
  }

  void init()
  {
    domain_client_ = std::make_shared<plansys2::DomainExpertClient>(*this);
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(*this);
    planner_client_ = std::make_shared<plansys2::PlannerClient>(*this);
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(*this);
  }

  // Runs in a loop: knowledge from the arbiter, and a new round whenever a plan ends
  void step()
  {
    if (finished_ || !view_) {
      return;
    }
    if (round_ == 0 && !executing_ && !waiting_) {
      // PlanSys2 may still be starting: wait until its problem expert answers
      if (!problem_client_->addInstance(plansys2::Instance(view_->zones.front(), "zone"))) {
        if (++waiting_expert_ > 75) {  // 15 s at 5 Hz
          RCLCPP_ERROR(get_logger(), "[%s] no problem expert", side_.c_str());
          finished_ = true;
        }
        return;
      }
      // The first view: what our side knows at the start
      if (!setup()) {
        RCLCPP_ERROR(get_logger(), "[%s] the problem expert rejected our knowledge", side_.c_str());
        finished_ = true;
        return;
      }
      new_view_ = false;
      next_round();
      return;
    }

    // The arbiter says the battle is over
    if (!view_->winner.empty()) {
      if (executing_) {
        executor_client_->cancel_plan_execution();
      }
      chronicle_->announce(
        "[" + side_ + "] " + (view_->winner == side_ ? "VICTORY" : "DEFEAT") + " after " +
        std::to_string(round_) + " rounds");
      battle_over_ = true;
      finished_ = true;
      return;
    }

    if (new_view_) {
      // While executing, only what we see of the world: our robots are the executor's
      new_view_ = false;
      news_ = true;
      reconcile(false);
      // A threat the plan did not count on: maybe there is a better plan now
      if (executing_) {
        for (const auto & t : threats()) {
          if (!planned_threats_.count(t)) {
            chronicle_->tell("[" + side_ + "] news: " + t + ", is there a better plan?");
            reconsider();
            break;
          }
        }
      }
    }

    // No plan last time: try again with news from the arbiter, or after a while
    if (waiting_) {
      if (news_ || now() >= retry_at_) {
        news_ = false;
        reconcile(true);
        next_round();
      }
      return;
    }

    if (executor_client_->execute_and_check_plan()) {
      return;
    }
    auto result = executor_client_->getResult();
    if (result.has_value() &&
      result.value().result == plansys2_msgs::action::ExecutePlan::Result::PREEMPT)
    {
      return;  // the plan that replaced it goes on
    }
    const bool success = result.has_value() &&
      result.value().result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
    end_round(success ? "done" : "failed");
  }

  // The plan is over (or given up): the problem as the view says, and the next round. A cancel
  // is not waited for: the result of a plan that had already ended would never come
  void end_round(const char * how)
  {
    executing_ = false;
    chronicle_->tell(
      "[" + side_ + "] round " + std::to_string(round_) + " " + how + ": planning again");
    reconcile(true);
    next_round();
  }

  void give_up_plan(const char * why)
  {
    chronicle_->tell("[" + side_ + "] " + why);
    executor_client_->cancel_plan_execution();
    end_round("cancelled");
  }

  bool finished() const {return finished_;}
  bool battle_over() const {return battle_over_;}

private:
  // The map and our army, set once
  bool setup()
  {
    const auto & v = *view_;
    bool ok = true;
    for (const auto & z : v.zones) {
      ok = add_instance(z, "zone") && ok;
    }
    for (size_t i = 0; i < v.connection_from.size(); i++) {
      const std::vector<std::string> ab = {v.connection_from[i], v.connection_to[i]};
      ok = add(fact("connected", ab)) && ok;
      ok = set_function(value("distance", ab, v.connection_distance[i])) && ok;
    }
    for (const auto & z : v.control_points) {
      ok = add(fact("control_point", {z})) && ok;
    }
    if (!v.base.empty()) {
      ok = add(fact("base", {v.base})) && ok;
    }
    ok = set_function("(= (spent) 0)") && ok;
    for (const auto & r : v.robots) {
      const auto & k = plansys2_combat_example::World::kinds().at(r.type);
      ok = add_instance(r.name, r.type) && ok;
      ok = add(fact("mine", {r.name})) && ok;
      for (const auto & [f, x] : std::vector<std::pair<std::string, double>>{
        {"value", k.value}, {"op_cost", k.op_cost}, {"shot_cost", k.shot_cost}})
      {
        ok = set_function(value(f, {r.name}, x)) && ok;
      }
      if (k.speed > 0.0) {
        ok = set_function(value("speed", {r.name}, k.speed)) && ok;
      }
    }
    ok = reconcile(true) && ok;
    if (!problem_client_->setMetric(plansys2::Metric(metric_))) {
      RCLCPP_WARN(get_logger(), "Metric [%s] not accepted, planning without it", metric_.c_str());
    }
    return ok;
  }

  // Makes the problem match the view. Between rounds, all of it; while a plan runs, only the
  // enemies, the zones explored and who holds them (the executor keeps our robots)
  bool reconcile(bool between_rounds)
  {
    const auto & v = *view_;
    std::set<std::string> enemies;
    for (const auto & e : v.enemies) {
      enemies.insert(e.name);
    }
    auto in_scope = [&](const std::string & name, const std::vector<std::string> & args) {
        if (between_rounds) {
          return kDynamic.count(name) > 0;
        }
        if (name == "at") {
          return !args.empty() && enemies.count(args[0]) > 0;
        }
        return name == "hostile" || name == "destroyed" || name == "explored" ||
               name == "owned" || name == "enemy_owned";
      };

    // What the view says
    std::set<std::string> wanted;
    bool ok = true;
    if (between_rounds) {
      for (const auto & r : v.robots) {
        if (!r.alive) {
          if (lost_.insert(r.name).second) {
            chronicle_->tell("[" + side_ + "] has lost " + r.name);
            problem_client_->removeInstance(plansys2::Instance(r.name, r.type));
          }
          continue;
        }
        wanted.insert(fact("at", {r.name, r.zone}));
        wanted.insert(fact("available", {r.name}));
        ok = set_function(value("ammo", {r.name}, r.ammo)) && ok;
        if (r.type == "drone") {
          ok = set_function(value("battery", {r.name}, r.battery)) && ok;
        }
      }
    }
    for (const auto & e : v.enemies) {
      if (seen_.insert(e.name).second) {
        RCLCPP_INFO(
          get_logger(), "[%s] enemy %s (%s) in %s", side_.c_str(), e.name.c_str(),
          e.type.c_str(), e.zone.c_str());
        ok = add_instance(e.name, e.type) && ok;
      }
      wanted.insert(fact("hostile", {e.name}));
      if (!e.alive) {
        wanted.insert(fact("destroyed", {e.name}));
      } else if (!e.zone.empty()) {  // where we last saw it, unless we know it left
        wanted.insert(fact("at", {e.name, e.zone}));
      }
    }
    for (const auto & z : v.explored) {
      wanted.insert(fact("explored", {z}));
    }
    for (const auto & z : v.owned) {
      wanted.insert(fact("owned", {z}));
    }
    for (const auto & z : v.enemy_owned) {
      wanted.insert(fact("enemy_owned", {z}));
    }

    // What the problem has: out what is stale, in what is missing
    std::set<std::string> have;
    for (const auto & p : problem_client_->getPredicates()) {
      std::vector<std::string> args;
      for (const auto & a : p.parameters) {
        args.push_back(a.name);
      }
      if (!in_scope(p.name, args)) {
        continue;
      }
      const auto f = fact(p.name, args);
      if (wanted.count(f)) {
        have.insert(f);
      } else {
        problem_client_->removePredicate(p);
      }
    }
    for (const auto & f : wanted) {
      if (!have.count(f)) {
        ok = add(f) && ok;
      }
    }
    return ok;
  }

  // With what we know now: a plan from the state of now, against what is left of the running one.
  // The new plan replaces it if the running one no longer reaches the goal or the new one is
  // cheaper; the actions in both go on
  void reconsider()
  {
    const auto domain = domain_client_->getDomain();
    const auto problem = executor_client_->get_replanning_problem();
    const auto remaining = executor_client_->get_remaining_plan();
    if (!problem || !remaining) {
      // The plan is starting or has just ended: its result says which. The threats stay new,
      // to look again with the next news
      return;
    }
    planned_threats_ = threats();
    const auto current = cost(domain, *problem, *remaining);
    const auto plan = planner_client_->getPlan(domain, *problem);
    const auto better = plan ? cost(domain, *problem, *plan) : std::nullopt;

    auto text = [](const std::optional<double> & c) {
        return c ? std::to_string(static_cast<int>(*c)) + " EUR" : std::string("none valid");
      };
    if (better && (!current || *better < *current * (1.0 - better_by_))) {
      chronicle_->tell(
        "[" + side_ + "] changes to a better plan: " + text(better) + " instead of " +
        text(current));
      log_plan(*plan);
      if (!executor_client_->start_plan_execution(*plan)) {
        give_up_plan("the executor did not take the better plan");
      }
    } else if (!current) {
      give_up_plan("the plan no longer works, and no other");
    } else {
      chronicle_->tell(
        "[" + side_ + "] keeps its plan (" + text(current) + "); " +
        (better ? "a new one, " + text(better) + ", is not enough better" : "no other works"));
    }
  }

  void log_plan(const plansys2_msgs::msg::Plan & plan)
  {
    for (const auto & item : plan.items) {
      RCLCPP_INFO(get_logger(), "  %7.2f: %s [%.2f]", item.time, item.action.c_str(),
        item.duration);
    }
  }

  // Plans and executes towards the goal; without a plan, waits to try again
  void next_round()
  {
    waiting_ = false;
    if (round_ >= max_rounds_) {
      RCLCPP_ERROR(get_logger(), "[%s] gave up after %d rounds", side_.c_str(), max_rounds_);
      finished_ = true;
      return;
    }

    const auto goal = make_goal();
    if (!problem_client_->setGoal(goal)) {
      RCLCPP_ERROR(get_logger(), "Goal not accepted: %s", parser::pddl::toString(goal).c_str());
      finished_ = true;
      return;
    }

    std::optional<plansys2_msgs::msg::Plan> plan;
    if (!problem_client_->isGoalSatisfied(goal)) {
      plan = planner_client_->getPlan(domain_client_->getDomain(), problem_client_->getProblem());
    }
    if (!plan.has_value() || plan->items.empty()) {
      // The goal holds (the arbiter will say so) or there is no way now: wait and look again
      waiting_ = true;
      retry_at_ = now() + rclcpp::Duration::from_seconds(retry_period_);
      return;
    }

    round_++;
    chronicle_->tell(
      "[" + side_ + "] round " + std::to_string(round_) + ": a plan of " +
      std::to_string(plan->items.size()) + " actions" + orders_);
    log_plan(*plan);
    planned_threats_ = threats();
    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "[%s] the executor did not start the plan", side_.c_str());
      waiting_ = true;
      retry_at_ = now() + rclcpp::Duration::from_seconds(retry_period_);
      return;
    }
    executing_ = true;
  }

  // Enemies alive where we know they are, and zones the enemy holds
  std::set<std::string> threats() const
  {
    std::set<std::string> ret;
    for (const auto & e : view_->enemies) {
      if (e.alive && !e.zone.empty()) {
        ret.insert(e.name + "@" + e.zone);
      }
    }
    for (const auto & z : view_->enemy_owned) {
      ret.insert("enemy_owned " + z);
    }
    return ret;
  }

  // Every control point ours; with "total", also every enemy we know destroyed. As a commander
  // would, each point still to take goes to the nearest free infantry, which ends there: the
  // planner then shares the work among the army instead of sending one robot everywhere
  plansys2::Goal make_goal()
  {
    std::string goal = "(and";
    for (const auto & z : view_->control_points) {
      goal += " (owned " + z + ")";
    }
    orders_.clear();
    if (assign_objectives_) {
      for (const auto & [robot, zone] : objectives()) {
        goal += " (at " + robot + " " + zone + ")";
        orders_ += (orders_.empty() ? "; orders: " : ", ") + robot + " to " + zone;
      }
    }
    if (goal_mode_ == "total") {
      for (const auto & e : view_->enemies) {
        goal += " (destroyed " + e.name + ")";
      }
    }
    return plansys2::Goal(goal + ")");
  }

  // Control points not ours yet, each with the nearest free infantry (roads away)
  std::vector<std::pair<std::string, std::string>> objectives() const
  {
    std::map<std::string, std::vector<std::string>> roads;
    for (size_t i = 0; i < view_->connection_from.size(); i++) {
      roads[view_->connection_from[i]].push_back(view_->connection_to[i]);
    }
    auto hops_from = [&roads](const std::string & start) {
        std::map<std::string, int> hops = {{start, 0}};
        std::vector<std::string> queue = {start};
        for (size_t i = 0; i < queue.size(); i++) {
          for (const auto & next : roads[queue[i]]) {
            if (hops.emplace(next, hops[queue[i]] + 1).second) {
              queue.push_back(next);
            }
          }
        }
        return hops;
      };
    std::map<std::string, std::map<std::string, int>> free;  // infantry -> hops to each zone
    for (const auto & r : view_->robots) {
      if (r.alive && r.type == "infantry") {
        free[r.name] = hops_from(r.zone);
      }
    }
    const std::set<std::string> owned(view_->owned.begin(), view_->owned.end());
    std::vector<std::pair<std::string, std::string>> ret;
    for (const auto & z : view_->control_points) {
      if (owned.count(z) || free.empty()) {
        continue;
      }
      auto best = free.end();
      for (auto it = free.begin(); it != free.end(); ++it) {
        auto h = it->second.find(z);
        if (h != it->second.end() &&
          (best == free.end() || h->second < best->second.at(z)))
        {
          best = it;
        }
      }
      if (best != free.end()) {
        ret.emplace_back(best->first, z);
        free.erase(best);
      }
    }
    return ret;
  }

  bool add(const std::string & p)
  {
    return logged(problem_client_->addPredicate(plansys2::Predicate(p)), p);
  }

  bool add_instance(const std::string & name, const std::string & type)
  {
    return logged(problem_client_->addInstance(plansys2::Instance(name, type)),
      name + " - " + type);
  }

  bool set_function(const std::string & f)
  {
    return logged(problem_client_->addFunction(plansys2::Function(f)), f);
  }

  bool logged(bool ok, const std::string & what)
  {
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "[%s] not accepted: %s", side_.c_str(), what.c_str());
    }
    return ok;
  }

  std::string side_;
  std::string goal_mode_;
  std::string metric_;
  int max_rounds_;
  int round_ = 0;
  int waiting_expert_ = 0;
  bool executing_ = false;
  bool waiting_ = false;  // for retry_at_, after a round with no plan
  bool news_ = false;     // the view changed since the last plan
  rclcpp::Time retry_at_;
  double retry_period_;
  double better_by_;
  bool assign_objectives_;
  std::string orders_;  // the objectives of the last goal, to tell
  std::unique_ptr<plansys2_combat_example::Chronicle> chronicle_;
  bool battle_over_ = false;
  bool finished_ = false;

  rclcpp::Subscription<SideView>::SharedPtr view_sub_;
  SideView::SharedPtr view_;
  bool new_view_ = false;
  std::set<std::string> planned_threats_;  // what the running plan was made with
  std::set<std::string> seen_;   // enemies already in the problem
  std::set<std::string> lost_;   // our robots destroyed

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CombatController>();
  node->init();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  rclcpp::Rate rate(5);
  while (rclcpp::ok() && !node->finished()) {
    exec.spin_some();
    node->step();
    rate.sleep();
  }

  rclcpp::shutdown();
  // A test tells by it whether the battle came to an end
  return node->battle_over() ? 0 : 1;
}

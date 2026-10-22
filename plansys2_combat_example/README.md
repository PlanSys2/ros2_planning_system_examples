# Combat example

Two robot armies on a battlefield. Each side plans with what it knows, the battle does not go as
planned, and PlanSys2 replans: a shot misses, infantry walks into a zone and finds enemies there,
an enemy is not where it was seen. Each side that plans has its own PlanSys2 in its namespace
(`/blue`, `/red`), with its own controller; an arbiter, shared and outside them, keeps the true
state of the battle. The side that takes every control point, or the only one with robots left,
wins.

## Running it

PlanSys2 recommends Zenoh. In every terminal:

```shell
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

and in one of them, the router:

```shell
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Each mode has its launcher, which starts the arbiter, the PlanSys2, performers and controller of
every side that plans, and RViz2 with the battle. The battle is told in the terminal of the
launcher, in colors: what really happens, from the arbiter, in yellow; what blue decides in
light cyan; what red decides in red. Captures, robots destroyed and the end of the battle stand
out in bold.

The PlanSys2 TUI shows each side's plan, its execution and its knowledge. Each side has its
PlanSys2 in its namespace, so the TUI of a side runs in it.

### Fog defense

Blue attacks a red defense that holds still on a 3x3 map: an infantry in the center, a tank in
the north and artillery in the east. Blue only knows its base: its drone surveys ahead, and
every enemy found changes the plan. Infantry takes the three control points in the middle
column; the artillery fires while the drone spots its target. Blue wins when it holds them all.

```shell
ros2 launch plansys2_combat_example fog_defense.launch.yaml
ros2 run plansys2_tui_cli plansys2_tui --ros-args -r __ns:=/blue
```

### Known defense

The same defense, with the whole map surveyed from the start: blue plans knowing where every
enemy is, and only misses and losses change the plan.

```shell
ros2 launch plansys2_combat_example known_defense.launch.yaml
ros2 run plansys2_tui_cli plansys2_tui --ros-args -r __ns:=/blue
```

### Duel

Blue and red, each with its PlanSys2 and its controller, fight for the three control points of
the same 3x3 map. Each one only knows its base at the start: they find each other as they move,
shoot at each other and take points from each other. The first to hold every point, or the only
one with robots left, wins.

```shell
ros2 launch plansys2_combat_example duel.launch.yaml
ros2 run plansys2_tui_cli plansys2_tui --ros-args -r __ns:=/blue
ros2 run plansys2_tui_cli plansys2_tui --ros-args -r __ns:=/red
```

### Big duel

A big battle of the same kind: twelve robots a side (two drones, six infantry, two tanks and two
artillery) on a 6x6 map crossed by a river with two bridges, and eight control points in pairs
on both banks. Plans have dozens of actions and take seconds, so time runs slower: a battle lasts
two or three minutes.

```shell
ros2 launch plansys2_combat_example big_duel.launch.yaml
ros2 run plansys2_tui_cli plansys2_tui --ros-args -r __ns:=/blue
ros2 run plansys2_tui_cli plansys2_tui --ros-args -r __ns:=/red
```

### Arguments

Every launcher takes:

- `goal`: `capture` (every control point) or `total` (also destroy every enemy found).
- `seed`: the shots that hit or miss; by default, the world's.
- `rviz`: `false` to run without RViz2.
- `colors`: `false` to tell the battle without colors, e.g. into a file.
- `shutdown_at_end`: `true` to stop everything when the battle ends; by default RViz2 keeps the
  last view.

`combat_launch.py` takes a `world` of `config/worlds` and the sides that plan (`armies`, by
default those of the world). In a world, `time_scale` sets how many real seconds one second of
the plan takes; `rviz` the RViz2 configuration that frames its map, and `water` what is only
drawn (the river of `big_duel`).

## Pieces

- **The domain** (`pddl/combat.pddl`): zones connected on a map, control points, and four kinds
  of robot. Drones fly anywhere, survey zones and spot targets; they fly over the battle, so they
  neither stop ground robots nor hold a zone. Infantry is the only one that captures control
  points. Tanks assault defended zones. Artillery fires at a connected zone while a drone spots
  it, which needs the STN executor (required concurrency). Each action costs euros: operation,
  ammunition and the risk of losing the robot. The metric is the euros spent.
- **The arbiter** (`combat_arbiter_node`): the true state of the battle, loaded from a world in
  `config/worlds`. It knows nothing of PlanSys2. It resolves every action
  (`/combat/start_action`, `/combat/finish_action`, `/combat/cancel_action`): whether the robot
  can do it (where it is, its ammunition and battery), how long it takes, where robots arrive,
  what they see, whether a shot hits, whether a zone is taken. It publishes what each side knows
  on `/combat/<side>/view` (transient local): its robots, the enemies it has seen where it saw
  them last (until it finds they left), the zones it has explored, who holds them and, at the
  end, the winner.
- **The performers** of a side (`combat_performers_node`): one `BTAction` per robot and action,
  in a single process, with `specialized_arguments` set to the robot. Every action is a behavior
  tree (`behavior_trees_xml`) with the `ArbiterAction` node, which takes the time the arbiter
  gives and fails when the action did not do what the plan expected.
- **The controller** of a side (`combat_controller_node`): turns its view into knowledge in its
  problem expert, sets the goal and the metric, plans with Kairos and executes. After every plan,
  finished or failed, it makes the problem match the view and plans again; with no plan, it waits
  for news. It stops when the arbiter says who has won.

  As a commander would, it gives each control point still to take to the nearest free infantry,
  which must end there (`assign_objectives`): the planner shares the work among the army, instead
  of a plan where one robot goes everywhere (in `pddl/problems/large/huge_fog_1.pddl`, 6 infantry
  and a plan of 75 s instead of one infantry and 130 s).

## Replanning with what is found

When a side finds an enemy where it did not know of one, or loses a zone, its controller plans
again from the state of that moment (`ExecutorClient::get_replanning_problem`). Kairos's
validator says what the new plan and what is left of the running one cost from there. If the
running one no longer reaches the goal, or the new one is cheaper (by `better_by`, 5 %), the new
plan replaces it while it runs: the actions in both go on. Otherwise the running plan goes on.
In a big battle a plan can take seconds: if the world changed meanwhile so that the new plan no
longer fits (an action already done, an enemy found on its way), the executor rejects it, and
the controller plans again from what it knows then.

When an action fails, PlanSys2 cancels every other action under way, and a new plan follows.
`ArbiterAction` tells the arbiter when it is halted: a robot on the road turns back to the zone
it left, as fast as it came. If the next plan gives it an action before it is back, the action
takes that much longer. Resources spent at the start of a cancelled action are not returned.

## Watching it in RViz2

The arbiter publishes the battle from its true state, outside every side's namespace:

- Zones as squares on the ground (`zone_positions` in the world), with a flag in a corner: gray,
  control points yellow until captured, then in the color of the side that holds them, bases in
  their side's color. The ground has a softer tone than the flag. Roads join connected zones.
- Every robot alive has its TF frame, child of `map`, and is drawn at its origin: infantry as a
  thin cylinder, tanks as boxes, artillery as a flat cylinder, drones as spheres in the air. Blue
  and red by side; in a world with a `viewpoint` side, the enemies it has not seen are faint.
- A robot travels along the road for as long as its action lasts, and turns back if cancelled.
- Shots are short lines; a destroyed robot disappears in an orange explosion.

The scene goes on `/combat/markers` (transient local, published only when something changes),
shots and explosions on `/combat/effects`, and the robots' frames on `/tf` at 20 Hz.
`rviz/combat.rviz` shows both.

## Tests

`world_test` checks the rules of the battle. `test_fog_defense.py` and `test_duel.py` run a
battle end to end and check that it ends with a winner that its controller hears of.

## Problems

`pddl/problems` has PDDL problems of this domain to try planners on: `scenarios` (s1-s7 on the
3x3 map, with euros or euros and time as metric), `large` (maps up to 6x6 and armies up to 24
robots), `mini` (minimal problems that isolate planner difficulties) and `plans` (hand-written
plans of some scenarios).

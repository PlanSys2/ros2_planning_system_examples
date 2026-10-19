# plansys2_pddlplus_example

A robot looks after a greenhouse. It is the example with the most of PDDL in it: PDDL+ (time,
processes, events), PDDL 2.1 continuous effects, PDDL3 constraints and preferences, and the
ADL features, each one with a part to play. PlanSys2 plans it with Kairos and executes it to the
end, and the controller checks, in the final state, that each part did what it does.

```sh
ros2 launch plansys2_pddlplus_example plansys2_pddlplus_example.launch.yaml
ros2 run plansys2_pddlplus_example greenhouse_controller_node
```

## The task

The rover starts at the dock with little battery. It has to tend the orchid and the fern in the
nursery, fertilize the orchid, fill the cistern and open the window before the nursery gets too
hot. The feeder it needs to fertilize is delivered at the dock after a while, and the nursery has
no power until the rover plugs the cable in the corridor. It should come back to the dock, if it
is worth it.

| Feature | In the greenhouse |
|---|---|
| Continuous effects (`#t`) | `move` spends battery while it lasts, `recharge` charges it |
| Duration inequalities | `recharge` lasts what the planner chooses, between 1 and 10 s |
| Processes | `filling` while the valve is open; `warming` while the window is closed |
| Events | `float_switch` closes the valve of a full cistern; `alarm` at 35 degrees |
| Timed initial literal | `(at 6 (tool_at feeder dock))`: the feeder is delivered |
| Timed initial fluent | `(at 8 (= (flow cistern) 2))`: the water pressure goes up |
| Recursive derived predicate | `powered`: from the generator, room after room, along the cables |
| Object fluent | `(tool rover)`: the tool it carries, changed by `pick_tool` |
| Conditional effect | `tend` fertilizes the plant only `(when (= (tool ?r) feeder) ...)` |
| Constants | `dock` and `feeder`, named by the domain itself |
| `forall`, `exists`, `not`, `=` | goal `(forall (?p - plant) (tended ?p))`; `exists` in `powered` |
| Constraints | `(always (not (heat_alarm nursery)))`, `(always (>= (battery rover) 5))` |
| Preference and metric | `(preference home (robot_at rover dock))`, `(is-violated home)` |

Kairos plans it in about 100 ms. Its plan waits for the feeder while it recharges, plugs the
cable, and opens the valve and the window with the nursery at about 32 degrees; it gives up the
preference rather than add a trip back.

## How PlanSys2 executes it

- The problem expert runs the dynamics: each step, the processes whose precondition holds and
  the continuous effects of the running actions act for the time since the last one, and then
  the events whose precondition holds happen. It applies the timed literals and fluents at their
  time, and keeps the derived predicates up to date. A function that perception measures goes in
  its `measured_functions` parameter: the dynamics leave it to perception.
- With processes, events, continuous effects or constraints with times, the executor starts each
  action at its planned time, holds a durative one until its planned duration, and ends the plan
  at the end Kairos gives it, when the goal must hold. An action that starts late shifts the rest
  of the plan, so the planned intervals hold.
- The executor follows the hard constraints while the plan runs: if one is violated, the plan
  fails and says which one. Preferences count only for the planner.
- The controller loads `watering.pddl` each time it runs, once PlanSys2 is up: the nursery warms
  up from the moment the problem is there.
- `timed_action_node` performs any action: it lasts what the plan gives it
  (`get_planned_duration()`). With `fail_once_at`, its first execution fails at that fraction of
  its duration.
- A plan that fails is made again from where the world is: the executor stops the continuous
  effects of what was running and takes back what had started, and the controller plans from
  there, up to three times. With the first recharge failing,

  ```sh
  ros2 launch plansys2_pddlplus_example plansys2_pddlplus_example.launch.yaml recharge_fails_at:=0.3
  ```

  the second plan goes on with the charge it got, if it is enough. A plan replaced while it runs
  keeps the running actions it still has: their continuous effects go on, for the time of the
  new plan, which their performers are told (`get_planned_duration()` changes).

`colcon test --packages-select plansys2_pddlplus_example` runs the whole example, and again with
the failing recharge, and checks the goal is reached and every part did its job.

# plansys2_pddlplus_example

A robot waters a greenhouse, in PDDL+: what happens depends on time, and PlanSys2 plans it with
Kairos and executes it to the end.

- **Continuous effects (PDDL 2.1):** `move` spends battery while it lasts,
  `(decrease (battery ?r) (* #t 2))`, and `recharge` charges it, for a duration Kairos chooses
  between 1 and 10 s.
- **Processes:** while its valve is open, the cistern fills (`filling`); while its window is
  closed, the nursery warms up (`warming`).
- **Events:** the float switch closes the valve of a full cistern (`float_switch`), and the
  nursery at 35 degrees raises an alarm (`alarm`).

The goal asks for a full cistern, the window open, no alarm and the robot back at the dock. No
action closes the valve: the robot opens it and leaves, and the event closes it when the water
gets there, however late or early the plan runs.

```sh
ros2 launch plansys2_pddlplus_example plansys2_pddlplus_example.launch.yaml
ros2 run plansys2_pddlplus_example greenhouse_controller_node
```

The controller loads `watering.pddl` each time it runs: the nursery warms up from the moment the
problem is there, so it starts from the beginning however long the system has been up.

## How PlanSys2 executes it

- The problem expert runs the dynamics: each step, the processes whose precondition holds and
  the continuous effects of the running actions act for the time since the last one, and then
  the events whose precondition holds happen. A function that perception measures goes in its
  `measured_functions` parameter: the dynamics leave it to perception.
- With processes, events or continuous effects, the executor starts each action at its planned
  time, holds a durative one until its planned duration, and ends the plan at the end Kairos
  gives it, when the goal must hold.
- `timed_action_node` performs any action: it lasts what the plan gives it
  (`get_planned_duration()`).

`colcon test --packages-select plansys2_pddlplus_example` runs the whole example and checks the
goal is reached.

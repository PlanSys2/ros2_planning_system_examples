# PlanSys2 Patrol Navigation Example

## Description

(**Requires Navigation2**)

This example shows:
- How to use an application to initiate knowledge (the PDDL problem) and control its execution flow.
- How to interact with ROS2 Navigation

The patrolling_controller_node node must be run separately. Use the PlanSys2 API to populate instances, predicates, and goals. Implement a finite state machine to establish as a goal which waypoint to patrol. Patrolling a waypoint is going to that point and turning around for a few seconds.

## How to run

In terminal 1:

```
ros2 launch plansys2_patrol_navigation_example patrol_example_launch.py
```

In terminal 2:

```
ros2 run plansys2_patrol_navigation_example patrolling_controller_node
```

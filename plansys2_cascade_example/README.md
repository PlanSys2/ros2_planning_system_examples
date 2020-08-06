# PlanSys2 Cascade Example

## Description

This example shows how an action can cascade trigger other ROS2 nodes. This can be useful when an action requires the execution of other ROS2 nodes to carry out its task.

In `move` action we have specified this dependency:

```

  MoveAction()
  : plansys2::ActionExecutorClient("move")
  {
    getFeedback()->progress = 0.0;

    add_activation("check_obstacles_node");
  }
```
## How to run

In terminal 1:

```
ros2 launch plansys2_cascade_example plansys2_cascade_example_launch.py
```

In terminal 2:

```
ros2 run plansys2_terminal plansys2_terminal        # enters in PlanSys2 Terminal
set instance leia robot
set instance entrance room
set instance kitchen room
set instance bedroom room
set instance dinning room
set instance bathroom room
set instance chargingroom room

set predicate (connected entrance dinning)
set predicate (connected dinning entrance)

set predicate (connected dinning kitchen)
set predicate (connected kitchen dinning)

set predicate (connected dinning bedroom)
set predicate (connected bedroom dinning)

set predicate (connected bathroom bedroom)
set predicate (connected bedroom bathroom)

set predicate (connected chargingroom kitchen)
set predicate (connected kitchen chargingroom)

set predicate (charging_point_at chargingroom)
set predicate (battery_low leia)
set predicate (robot_at leia entrance)

set goal (and(robot_at leia bathroom))                # Sets the goal
get plan                                              # Creates plan and shows it
run                                                   # Creates plan and runs it
```

Please, check that check_obstacles is running while move is running.
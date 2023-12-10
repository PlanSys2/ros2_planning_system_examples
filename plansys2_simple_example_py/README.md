# PlanSys2 Simple Example in Python

## Description

This is a simple example that shows the basic operation of PlanSys2. A simple PDDL domain has been specified, and we will use the PlanSys2 Terminal to insert commands. Actions simulate their execution.

## How to run

In terminal 1:

```
ros2 launch plansys2_simple_example_py plansys2_simple_example_launch.py
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

get problem instances                               # Checks instances

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

get problem predicates                                # Checks predicates

set goal (and(robot_at leia bathroom))                # Sets the goal
get plan                                              # Creates plan and shows it
run                                                   # Creates plan and runs it
```

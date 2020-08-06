# PlanSys2 Multidomain Example

## Description

This example shows how to use multiple PDDL domains, which are mixed at the beginning of the execution. 
- domain1.pddl: This domain contains all related to robot navigation
- domain2.pddl: This domain contains all related to object manipulation

We will use the PlanSys2 Terminal to insert commands. Actions simulate their execution.

## How to run

In terminal 1:

```
ros2 launch plansys2_multidomain_example plansys2_multidomain_example_launch.py
```

In terminal 2:

```
ros2 run plansys2_terminal plansys2_terminal        # enters in PlanSys2 Terminal
get domain                                          # Checks that both domains are correctly mixed

set instance leia robot
set instance entrance room
set instance kitchen room
set instance bedroom room
set instance dinning room
set instance bathroom room
set instance chargingroom room
set instance object1 pickable_object

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

set predicate (object_at_room object1 bedroom)

set goal (and(object_at_room object1 bathroom))       # Sets the goal
get plan                                              # Creates plan and shows it
run                                                   # Creates plan and runs it
```

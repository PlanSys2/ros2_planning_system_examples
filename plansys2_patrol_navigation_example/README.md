# PlanSys2 Patrol Navigation Example

## Description

(**Requires [EasyNav](https://easynavigation.github.io)**; without it, `move_action_node` is not built)

This example shows:
- How to use an application to initiate knowledge (the PDDL problem) and control its execution flow.
- How to navigate with EasyNav, using `easynav_system::GoalManagerClient`

The patrolling_controller_node node must be run separately. Use the PlanSys2 API to populate instances, predicates, and goals. Implement a finite state machine to establish as a goal which waypoint to patrol. Patrolling a waypoint is going to that point and turning around for a few seconds. While it patrols, it shows the navigation feedback of EasyNav every 2 seconds: a subscription of the controller, served by the executor that spins its node in the loop of `main` (the PlanSys2 clients bring their own answers).

## How to run

### With a simulated navigation

In terminal 1:

```
ros2 launch plansys2_patrol_navigation_example patrol_example_fakesim.launch.yaml
```

In terminal 2:

```
ros2 run plansys2_patrol_navigation_example patrolling_controller_node
```

### With a simulated robot

It needs the kobuki playground of [easynav_playgrounds](https://github.com/EasyNavigation/easynav_playgrounds)
and the EasyNav plugins it uses. All at once (Gazebo, EasyNav, PlanSys2 and the controller;
`gui:=false` runs Gazebo headless):

```
ros2 launch plansys2_patrol_navigation_example patrol_example_kobuki.launch.yaml
```

Or step by step. In terminal 1, the kobuki playground:

```
ros2 launch easynav_playground_kobuki easynav_costmap_rpp.launch.yaml
```

In terminal 2 (`waypoints_file` sets the waypoints for another map):

```
ros2 launch plansys2_patrol_navigation_example patrol_example.launch.yaml
```

In terminal 3:

```
ros2 run plansys2_patrol_navigation_example patrolling_controller_node
```

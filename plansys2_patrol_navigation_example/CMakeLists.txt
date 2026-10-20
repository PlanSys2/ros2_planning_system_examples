cmake_minimum_required(VERSION 3.5)
project(plansys2_patrol_navigation_example)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# Navigation with EasyNav is optional: without it, move_action_node and
# patrolling_controller_node (it shows the feedback of EasyNav) are not built
find_package(easynav_interfaces QUIET)
find_package(easynav_system QUIET)
if(easynav_system_FOUND AND easynav_interfaces_FOUND)
  add_executable(move_action_node src/move_action_node.cpp)
  target_link_libraries(move_action_node
    PRIVATE
    ${easynav_interfaces_TARGETS}
    easynav_system::easynav_system
    ${geometry_msgs_TARGETS}
    plansys2_executor::plansys2_executor
    rclcpp::rclcpp
  )

  add_executable(patrolling_controller_node src/patrolling_controller_node.cpp)
  target_link_libraries(patrolling_controller_node
    PRIVATE
    ${easynav_interfaces_TARGETS}
    plansys2_executor::plansys2_executor
    plansys2_domain_expert::plansys2_domain_expert
    ${plansys2_msgs_TARGETS}
    plansys2_pddl_parser::plansys2_pddl_parser
    plansys2_planner::plansys2_planner
    plansys2_problem_expert::plansys2_problem_expert
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
  )
  install(TARGETS move_action_node patrolling_controller_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "EasyNav not found: move_action_node and patrolling_controller_node are not built")
endif()

add_executable(patrol_action_node src/patrol_action_node.cpp)
target_link_libraries(patrol_action_node
  PRIVATE
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

add_executable(patrol_waypoints_controller_node src/patrol_waypoints_controller_node.cpp)
target_link_libraries(patrol_waypoints_controller_node
  PRIVATE
  ${lifecycle_msgs_TARGETS}
  plansys2_executor::plansys2_executor
  plansys2_domain_expert::plansys2_domain_expert
  ${plansys2_msgs_TARGETS}
  plansys2_planner::plansys2_planner
  plansys2_problem_expert::plansys2_problem_expert
  rclcpp::rclcpp
)

install(DIRECTORY launch pddl params
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  patrol_action_node
  patrol_waypoints_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # The example run end to end; it needs the simulated EasyNav
  if(easynav_system_FOUND AND easynav_interfaces_FOUND)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_patrol_waypoints.py TIMEOUT 300)
  endif()
endif()

ament_export_dependencies(
  lifecycle_msgs
  plansys2_domain_expert
  plansys2_executor
  plansys2_msgs
  plansys2_planner
  plansys2_problem_expert
  plansys2_pddl_parser
  rclcpp
  rclcpp_action
)

ament_package()

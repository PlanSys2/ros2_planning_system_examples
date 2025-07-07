cmake_minimum_required(VERSION 3.5)
project(plansys2_patrol_navigation_example)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)


set(dependencies
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    plansys2_domain_expert::plansys2_domain_expert
    plansys2_executor::plansys2_executor
    plansys2_planner::plansys2_planner
    plansys2_problem_expert::plansys2_problem_expert
    plansys2_pddl_parser::plansys2_pddl_parser
    ${plansys2_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
)

add_executable(move_action_node src/move_action_node.cpp)
target_link_libraries(move_action_node PUBLIC ${dependencies})

add_executable(patrol_action_node src/patrol_action_node.cpp)
target_link_libraries(patrol_action_node PUBLIC ${dependencies})

add_executable(patrolling_controller_node src/patrolling_controller_node.cpp)
target_link_libraries(patrolling_controller_node PUBLIC ${dependencies})

install(DIRECTORY launch pddl params DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  patrol_action_node
  patrolling_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()

cmake_minimum_required(VERSION 3.16)
project(plansys2_pddlplus_example)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)

add_executable(timed_action_node src/timed_action_node.cpp)
target_link_libraries(timed_action_node
  PRIVATE
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
)

add_executable(greenhouse_controller_node src/greenhouse_controller_node.cpp)
target_link_libraries(greenhouse_controller_node
  PRIVATE
  ament_index_cpp::ament_index_cpp
  ${lifecycle_msgs_TARGETS}
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_executor::plansys2_executor
  plansys2_planner::plansys2_planner
  plansys2_problem_expert::plansys2_problem_expert
  rclcpp::rclcpp
)

install(DIRECTORY launch pddl
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  timed_action_node
  greenhouse_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Humble's uncrustify (0.72) formats otherwise than the newer one this code follows
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(ament_cmake_uncrustify_FOUND TRUE)
  endif()
  ament_lint_auto_find_test_dependencies()

  # The whole example: planned and executed to its goal
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_greenhouse.py TIMEOUT 120)
  add_launch_test(test/test_greenhouse_replans.py TIMEOUT 150)
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(plansys2_simple_example)

find_package(ament_cmake REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(move_action_node src/move_action_node.cpp)
target_link_libraries(move_action_node
  PRIVATE
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

add_executable(charge_action_node src/charge_action_node.cpp)
target_link_libraries(charge_action_node
  PRIVATE
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
)

add_executable(ask_charge_action_node src/ask_charge_action_node.cpp)
target_link_libraries(ask_charge_action_node
  PRIVATE
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

install(DIRECTORY launch pddl
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  move_action_node
  charge_action_node
  ask_charge_action_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(
  plansys2_executor
  rclcpp
  rclcpp_action
)

ament_package()
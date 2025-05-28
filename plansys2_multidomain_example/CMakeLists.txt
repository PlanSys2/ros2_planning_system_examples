project(plansys2_multidomain_example)

cmake_minimum_required(VERSION 3.5)

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
  rclcpp_action::rclcpp_action
)

add_executable(ask_charge_action_node src/ask_charge_action_node.cpp)
target_link_libraries(ask_charge_action_node
  PRIVATE
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

add_executable(pick_object_action_node src/pick_object_action_node.cpp)
target_link_libraries(pick_object_action_node
  PRIVATE
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

add_executable(place_object_action_node src/place_object_action_node.cpp)
target_link_libraries(place_object_action_node
  PRIVATE
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

install(DIRECTORY launch pddl_1 pddl_2 DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  charge_action_node
  ask_charge_action_node
  pick_object_action_node
  place_object_action_node
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
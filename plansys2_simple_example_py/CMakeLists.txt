cmake_minimum_required(VERSION 3.5)
project(plansys2_simple_example_py)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_executor REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

ament_python_install_package(${PROJECT_NAME})

add_executable(move_action_node src/move_action_node.cpp)
target_link_libraries(move_action_node
  PRIVATE
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
)

add_executable(ask_charge_action_node src/ask_charge_action_node.cpp)
target_link_libraries(ask_charge_action_node
  PRIVATE
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
)

install(DIRECTORY launch pddl
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  move_action_node
  ask_charge_action_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  plansys2_simple_example_py/charge_action_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(
  plansys2_executor
  rclcpp
)

ament_package()
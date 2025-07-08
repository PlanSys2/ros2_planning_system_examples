cmake_minimum_required(VERSION 3.5)
project(plansys2_simple_example)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)



set(dependencies
    rclcpp::rclcpp
    plansys2_executor::plansys2_executor
    ${plansys2_msgs_TARGETS}
)

add_executable(move_action_node src/move_action_node.cpp)
target_link_libraries(move_action_node PUBLIC ${dependencies})

add_executable(charge_action_node src/charge_action_node.cpp)
target_link_libraries(charge_action_node PUBLIC ${dependencies})

add_executable(ask_charge_action_node src/ask_charge_action_node.cpp)
target_link_libraries(ask_charge_action_node PUBLIC ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  charge_action_node
  ask_charge_action_node
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
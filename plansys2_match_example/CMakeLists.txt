cmake_minimum_required(VERSION 3.5)
project(plansys2_match_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_action
  plansys2_msgs
  plansys2_executor
)

add_executable(light_match_action_node src/light_match_action_node.cpp)
ament_target_dependencies(light_match_action_node ${dependencies})

add_executable(mend_fuse_action_node src/mend_fuse_action_node.cpp)
ament_target_dependencies(mend_fuse_action_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  light_match_action_node
  mend_fuse_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()

cmake_minimum_required(VERSION 3.5)
project(plansys2_bt_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_executor
    behaviortree_cpp_v3
    ament_index_cpp
)

include_directories(include)

add_library(${PROJECT_NAME}
  src/behavior_tree_nodes/ApproachObject.cpp
  src/behavior_tree_nodes/CloseGripper.cpp
  src/behavior_tree_nodes/Move.cpp
  src/behavior_tree_nodes/OpenGripper.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(charge_bt_main src/charge_bt_main.cpp)
ament_target_dependencies(charge_bt_main ${dependencies})
target_link_libraries(charge_bt_main ${PROJECT_NAME})

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})
target_link_libraries(move_action_node ${PROJECT_NAME})

add_executable(assemble_action_node src/assemble_action_node.cpp)
ament_target_dependencies(assemble_action_node ${dependencies})
target_link_libraries(assemble_action_node ${PROJECT_NAME})

add_executable(transport_action_node src/transport_action_node.cpp)
ament_target_dependencies(transport_action_node ${dependencies})
target_link_libraries(transport_action_node ${PROJECT_NAME})

add_executable(nav2_sim_node src/nav2_sim_node.cpp)
ament_target_dependencies(nav2_sim_node ${dependencies})
target_link_libraries(nav2_sim_node ${PROJECT_NAME})

install(DIRECTORY launch pddl behavior_trees_xml DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  assemble_action_node
  transport_action_node
  charge_bt_main
  nav2_sim_node
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
cmake_minimum_required(VERSION 3.5)
project(plansys2_bt_example)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(plansys2_recharge_bt_node SHARED src/behavior_tree_nodes/Recharge.cpp)
list(APPEND plugin_libs plansys2_recharge_bt_node)

add_library(plansys2_close_gripper_bt_node SHARED src/behavior_tree_nodes/CloseGripper.cpp)
list(APPEND plugin_libs plansys2_close_gripper_bt_node)

add_library(plansys2_open_gripper_bt_node SHARED src/behavior_tree_nodes/OpenGripper.cpp)
list(APPEND plugin_libs plansys2_open_gripper_bt_node)

add_library(plansys2_move_bt_node SHARED src/behavior_tree_nodes/Move.cpp)
list(APPEND plugin_libs plansys2_move_bt_node)

add_library(plansys2_approach_object_bt_node SHARED src/behavior_tree_nodes/ApproachObject.cpp)
list(APPEND plugin_libs plansys2_approach_object_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${bt_plugin}
    PUBLIC
    behaviortree_cpp::behaviortree_cpp
    ${geometry_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
    plansys2_bt_actions::plansys2_bt_actions
    PRIVATE
    ${tf2_geometry_msgs_TARGETS}
    rclcpp_lifecycle::rclcpp_lifecycle
  )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(assemble_action_node src/assemble_action_node.cpp)
target_include_directories(assemble_action_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(assemble_action_node
  PRIVATE
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  plansys2_executor::plansys2_executor
)

add_executable(assemble_controller_node src/assemble_controller_node.cpp)
target_include_directories(assemble_controller_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(assemble_controller_node
  PRIVATE
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_executor::plansys2_executor
  ${plansys2_msgs_TARGETS}
  plansys2_pddl_parser::plansys2_pddl_parser
  plansys2_planner::plansys2_planner
  plansys2_problem_expert::plansys2_problem_expert
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

add_executable(nav2_sim_node src/nav2_sim_node.cpp)
target_include_directories(nav2_sim_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav2_sim_node
  PRIVATE
  ${nav2_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${tf2_geometry_msgs_TARGETS}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch pddl behavior_trees_xml config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${plugin_libs}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS assemble_action_node assemble_controller_node nav2_sim_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
  behaviortree_cpp
  geometry_msgs
  nav2_msgs
  plansys2_msgs
  plansys2_domain_expert
  plansys2_executor
  plansys2_planner
  plansys2_problem_expert
  plansys2_pddl_parser
  plansys2_bt_actions
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
)

ament_package()

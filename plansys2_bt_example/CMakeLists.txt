cmake_minimum_required(VERSION 3.5)
project(plansys2_bt_example)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Navigation with EasyNav is optional: without it, Move and easynav_sim_node are not built
find_package(easynav_interfaces QUIET)
find_package(easynav_system QUIET)
if(NOT easynav_interfaces_FOUND)
  message(WARNING "easynav_interfaces not found: easynav_sim_node is not built")
endif()
if(NOT easynav_system_FOUND)
  message(WARNING "easynav_system not found: the Move BT node is not built")
endif()

add_library(plansys2_recharge_bt_node SHARED src/behavior_tree_nodes/Recharge.cpp)
list(APPEND plugin_libs plansys2_recharge_bt_node)

add_library(plansys2_close_gripper_bt_node SHARED src/behavior_tree_nodes/CloseGripper.cpp)
list(APPEND plugin_libs plansys2_close_gripper_bt_node)

add_library(plansys2_open_gripper_bt_node SHARED src/behavior_tree_nodes/OpenGripper.cpp)
list(APPEND plugin_libs plansys2_open_gripper_bt_node)

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
    plansys2_bt_actions::plansys2_bt_actions
    PRIVATE
    ${tf2_geometry_msgs_TARGETS}
    rclcpp_lifecycle::rclcpp_lifecycle
  )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

if(easynav_system_FOUND AND easynav_interfaces_FOUND)
  add_library(plansys2_move_bt_node SHARED src/behavior_tree_nodes/Move.cpp)
  target_include_directories(plansys2_move_bt_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(plansys2_move_bt_node
    PUBLIC
    behaviortree_cpp::behaviortree_cpp
    ${easynav_interfaces_TARGETS}
    easynav_system::easynav_system
    ${geometry_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    PRIVATE
    ${tf2_geometry_msgs_TARGETS}
  )
  target_compile_definitions(plansys2_move_bt_node PRIVATE BT_PLUGIN_EXPORT)
  list(APPEND plugin_libs plansys2_move_bt_node)
else()
  # Move.hpp needs EasyNav
  set(install_header_excludes PATTERN "Move.hpp" EXCLUDE)
endif()

if(easynav_interfaces_FOUND)
  add_executable(easynav_sim_node src/easynav_sim_node.cpp)
  target_link_libraries(easynav_sim_node
    PRIVATE
    ${easynav_interfaces_TARGETS}
    rclcpp::rclcpp
  )
  install(TARGETS easynav_sim_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

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
  ${lifecycle_msgs_TARGETS}
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_executor::plansys2_executor
  ${plansys2_msgs_TARGETS}
  plansys2_pddl_parser::plansys2_pddl_parser
  plansys2_planner::plansys2_planner
  plansys2_problem_expert::plansys2_problem_expert
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

add_executable(timed_action_node src/timed_action_node.cpp)
target_link_libraries(timed_action_node
  PRIVATE
  rclcpp::rclcpp
  plansys2_executor::plansys2_executor
)

add_executable(assemble_arm_controller_node src/assemble_arm_controller_node.cpp)
target_link_libraries(assemble_arm_controller_node
  PRIVATE
  ${lifecycle_msgs_TARGETS}
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_executor::plansys2_executor
  ${plansys2_msgs_TARGETS}
  plansys2_pddl_parser::plansys2_pddl_parser
  plansys2_planner::plansys2_planner
  plansys2_problem_expert::plansys2_problem_expert
  rclcpp::rclcpp
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  ${install_header_excludes}
)

install(DIRECTORY launch pddl behavior_trees_xml config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${plugin_libs}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS assemble_action_node assemble_controller_node
  timed_action_node assemble_arm_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # The example run end to end; it needs the simulated EasyNav
  if(easynav_system_FOUND AND easynav_interfaces_FOUND)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_terminal_commands.py TIMEOUT 300)
    add_launch_test(test/test_assemble_controller.py TIMEOUT 300)
    add_launch_test(test/test_assemble_arm_controller.py TIMEOUT 900)
  endif()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
if(easynav_system_FOUND)
  ament_export_dependencies(easynav_system)
endif()
ament_export_dependencies(
  behaviortree_cpp
  geometry_msgs
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

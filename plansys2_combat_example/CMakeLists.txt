cmake_minimum_required(VERSION 3.8)

project(plansys2_combat_example)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(kairos REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/SideView.msg"
  "srv/StartAction.srv"
  "srv/FinishAction.srv"
  "srv/CancelAction.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# The rules of the battle, without ROS: the arbiter and the tests use it
add_library(combat_world SHARED src/World.cpp)
target_include_directories(combat_world PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# The BT node every action uses: the arbiter resolves the action
add_library(plansys2_combat_arbiter_action_bt_node SHARED
  src/behavior_tree_nodes/ArbiterAction.cpp)
target_include_directories(plansys2_combat_arbiter_action_bt_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(plansys2_combat_arbiter_action_bt_node
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  "${cpp_typesupport_target}"
)
target_compile_definitions(plansys2_combat_arbiter_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_executable(combat_arbiter_node src/combat_arbiter_node.cpp src/BattleView.cpp)
target_link_libraries(combat_arbiter_node
  combat_world
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  "${cpp_typesupport_target}"
)

add_executable(combat_controller_node src/combat_controller_node.cpp)
target_link_libraries(combat_controller_node
  combat_world
  kairos::kairos
  rclcpp::rclcpp
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_executor::plansys2_executor
  plansys2_planner::plansys2_planner
  plansys2_problem_expert::plansys2_problem_expert
  plansys2_pddl_parser::plansys2_pddl_parser
  "${cpp_typesupport_target}"
)

# Every performer of a side in one process
add_executable(combat_performers_node src/combat_performers_node.cpp)
target_link_libraries(combat_performers_node
  rclcpp::rclcpp
  plansys2_bt_actions::plansys2_bt_actions
  ${lifecycle_msgs_TARGETS}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch pddl config behavior_trees_xml rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS combat_world plansys2_combat_arbiter_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS combat_arbiter_node combat_controller_node combat_performers_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Humble's uncrustify (0.72) formats otherwise than the newer one this code follows
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(ament_cmake_uncrustify_FOUND TRUE)
  endif()
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(world_test test/world_test.cpp)
  target_link_libraries(world_test combat_world)

  # The battles run end to end
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_fog_defense.py TIMEOUT 300)
  add_launch_test(test/test_duel.py TIMEOUT 300)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

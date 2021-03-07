# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_bt_example')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/bt_example.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    move_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_1',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])

    move_2_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_2',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1670,
            'server_port': 1671,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])

    move_3_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_3',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1672,
            'server_port': 1673,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])

    transport_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='transport_1',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'transport',
            'publisher_port': 1674,
            'server_port': 1675,
            'bt_xml_file': example_dir + '/behavior_trees_xml/transport.xml'
          }
        ])
    transport_2_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='transport_2',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'transport',
            'publisher_port': 1676,
            'server_port': 1677,
            'bt_xml_file': example_dir + '/behavior_trees_xml/transport.xml'
          }
        ])
    transport_3_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='transport_3',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'transport',
            'publisher_port': 1678,
            'server_port': 1679,
            'bt_xml_file': example_dir + '/behavior_trees_xml/transport.xml'
          }
        ])

    assemble_1_cmd = Node(
        package='plansys2_bt_example',
        executable='assemble_action_node',
        name='assemble_1',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate
    assemble_2_cmd = Node(
        package='plansys2_bt_example',
        executable='assemble_action_node',
        name='assemble_2',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate
    assemble_3_cmd = Node(
        package='plansys2_bt_example',
        executable='assemble_action_node',
        name='assemble_3',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate

    recharge_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='recharge_1',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'recharge',
            'bt_xml_file': example_dir + '/behavior_trees_xml/recharge.xml'
          }
        ])

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_1_cmd)
    ld.add_action(move_2_cmd)
    ld.add_action(move_3_cmd)
    ld.add_action(transport_1_cmd)
    ld.add_action(transport_2_cmd)
    ld.add_action(transport_3_cmd)
    ld.add_action(assemble_1_cmd)
    ld.add_action(assemble_2_cmd)
    ld.add_action(assemble_3_cmd)
    ld.add_action(recharge_1_cmd)

    return ld

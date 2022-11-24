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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    move_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml',
          }
        ])

    pick_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'pick',
            'publisher_port': 3670,
            'server_port': 3671,
            'bt_xml_file': example_dir + '/behavior_trees_xml/pick.xml',
            'rate': 2.0
          }
        ])

    prepick_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='prepick',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'prepick',
            'publisher_port': 3672,
            'server_port': 3673,
            'bt_xml_file': example_dir + '/behavior_trees_xml/prepick.xml',
            'rate': 2.0
          }
        ])

    release_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='release',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'release',
            'publisher_port': 3674,
            'server_port': 3675,
            'bt_xml_file': example_dir + '/behavior_trees_xml/release.xml',
            'rate': 2.0
          }
        ])

    prerelease_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='prerelease',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'prerelease',
            'publisher_port': 3676,
            'server_port': 3677,
            'bt_xml_file': example_dir + '/behavior_trees_xml/prerelease.xml',
            'rate': 2.0
          }
        ])

    assemble_cmd = Node(
        package='plansys2_bt_example',
        executable='assemble_action_node',
        name='assemble',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_cmd)
    ld.add_action(pick_cmd)
    ld.add_action(prepick_cmd)
    ld.add_action(release_cmd)
    ld.add_action(prerelease_cmd)
    ld.add_action(assemble_cmd)

    return ld

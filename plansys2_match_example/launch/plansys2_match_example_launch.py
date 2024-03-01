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
    example_dir = get_package_share_directory('plansys2_match_example')
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
          'model_file': example_dir + '/pddl/match_domain.pddl',
          'namespace': namespace,
          'bt_builder_plugin': 'STNBTBuilder'
          }.items())

    # Specify the actions
    light_match_cmd = Node(
        package='plansys2_match_example',
        executable='light_match_action_node',
        name='light_match_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    light_match_cmd_2 = Node(
        package='plansys2_match_example',
        executable='light_match_action_node',
        name='light_match_action_node_2',
        namespace=namespace,
        output='screen',
        parameters=[])

    mend_fuse_cmd = Node(
        package='plansys2_match_example',
        executable='mend_fuse_action_node',
        name='mend_fuse_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    mend_fuse_cmd_2 = Node(
        package='plansys2_match_example',
        executable='mend_fuse_action_node',
        name='mend_fuse_action_node_2',
        namespace=namespace,
        output='screen',
        parameters=[])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(light_match_cmd)
    ld.add_action(light_match_cmd_2)
    ld.add_action(mend_fuse_cmd)
    ld.add_action(mend_fuse_cmd_2)

    return ld

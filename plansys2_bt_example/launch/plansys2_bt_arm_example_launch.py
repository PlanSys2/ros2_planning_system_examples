# Copyright 2026 Intelligent Robotics Lab
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

# Assembly with arms: the same plan executed with SimpleBTBuilder or STNBTBuilder
# (bt_builder:=simple|stn) to compare how each one parallelizes it.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Action: seconds, as in the domain (move is the simulated navigation)
TIMED_ACTIONS = {
    'approach': 3.0,
    'prepare_pick': 3.0,
    'pick': 2.0,
    'prepare_release': 3.0,
    'release': 2.0,
    'assemble': 5.0,
}


def generate_launch_description():
    example_dir = get_package_share_directory('plansys2_bt_example')
    namespace = LaunchConfiguration('namespace')
    bt_builder = LaunchConfiguration('bt_builder')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace')
    declare_bt_builder_cmd = DeclareLaunchArgument(
        'bt_builder', default_value='simple', choices=['simple', 'stn'],
        description='Behavior tree builder that executes the plan')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': example_dir + '/pddl/bt_arm_example.pddl',
            'namespace': namespace,
            'params_file': PathJoinSubstitution(
                [example_dir, 'config', ['plansys2_arm_', bt_builder, '.yaml']]),
        }.items())

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_bt_builder_cmd)
    ld.add_action(plansys2_cmd)
    # Navigation is simulated: no Nav2 needed
    ld.add_action(Node(
        package='plansys2_bt_example',
        executable='nav2_sim_node',
        namespace=namespace,
        output='screen'))

    # One performer per robot for each action
    for i in range(1, 4):
        ld.add_action(Node(
            package='plansys2_bt_actions',
            executable='bt_action_node',
            name=f'move_{i}',
            namespace=namespace,
            output='screen',
            parameters=[
                example_dir + '/config/params.yaml',
                {
                    'action_name': 'move',
                    'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml',
                    # Many nodes here: discovering navigate_to_pose can take over 1 s
                    'wait_for_service_timeout': 5000,
                }
            ]))
        for action, duration in TIMED_ACTIONS.items():
            ld.add_action(Node(
                package='plansys2_bt_example',
                executable='timed_action_node',
                name=f'{action}_{i}',
                namespace=namespace,
                output='screen',
                parameters=[{'action_name': action, 'duration': duration, 'rate': 10.0}]))

    return ld

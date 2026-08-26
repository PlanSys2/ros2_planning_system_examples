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
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_auction_example')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': example_dir + '/pddl/patrol.pddl'}.items()
        )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'tb3_simulation_launch.py')),
        launch_arguments={
            'autostart': 'true',
            'params_file': os.path.join(example_dir, 'params', 'nav2_params.yaml')
        }.items())

    
    action_param_path = os.path.join(
        example_dir,
        'params',
        'actions_params.yaml'
        )
    with open(action_param_path, 'r') as file:
        configFile = yaml.safe_load(file)
        configParams = configFile['move_action_node']['ros__parameters']
        configParams1 = configFile['move_action_node1']['ros__parameters']
        # configParams2 = configFile['move_action_node2']['ros__parameters']

    print(configParams)
    move_cmd = Node(
        package='plansys2_auction_example',
        executable='move_action_node',
        name='move_action_node',
        output='screen',
        parameters=[configParams])
    move_cmd1 = Node(
        package='plansys2_auction_example',
        executable='move_action_node',
        name='move_action_node1',
        output='screen',
        parameters=[configParams1])
    # move_cmd2 = Node(
    #     package='plansys2_auction_example',
    #     executable='move_action_node',
    #     name='move_action_node2',
    #     output='screen',
    #     parameters=[configParams2])
    # move_cmd2 = Node(
    #     package='plansys2_auction_example',
    #     executable='move_action_node',
    #     name='move_action_node',
    #     output='screen',
    #     parameters=[])
    patrol_cmd = Node(
        package='plansys2_auction_example',
        executable='patrol_action_node',
        name='patrol_action_node',
        output='screen',
        parameters=[])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    # ld.add_action(nav2_cmd)

    ld.add_action(move_cmd)
    ld.add_action(move_cmd1)
    # ld.add_action(move_cmd2)
    # ld.add_action(move_cmd2)
    ld.add_action(patrol_cmd)

    return ld

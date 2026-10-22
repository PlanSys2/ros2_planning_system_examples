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

"""
Launch the battle: the arbiter, and for every army that plans, its PlanSys2 in its namespace.

Each planning army gets its PlanSys2, one performer per robot and action, and its controller,
all in /<side>. The arbiter and RViz stay outside: /combat/... is shared by every side.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# Actions each kind of robot does, and how many arguments they have (pddl/combat.pddl)
ACTIONS = {
    'drone': {'fly': 3, 'survey': 2, 'spot': 2, 'recharge': 2},
    'infantry': {'move': 3, 'advance': 3, 'assault_infantry': 3, 'attack_infantry': 3,
                 'capture': 2},
    'tank': {'move': 3, 'advance': 3, 'assault_tank': 3, 'attack_tank': 3},
    'artillery': {'move': 3, 'advance': 3, 'bombard': 4},
}


def battle(context):
    """Create the arbiter, and PlanSys2, performers and controller for every army that plans."""
    example_dir = get_package_share_directory('plansys2_combat_example')
    world_file = os.path.join(
        example_dir, 'config', 'worlds', LaunchConfiguration('world').perform(context) + '.yaml')
    with open(world_file) as f:
        world = yaml.safe_load(f)['combat_arbiter']['ros__parameters']

    # Another seed: other shots hit or miss
    colors = LaunchConfiguration('colors').perform(context) == 'true'
    arbiter_params = [world_file, {'colors': colors}]
    seed = LaunchConfiguration('seed').perform(context)
    if seed:
        arbiter_params.append({'seed': int(seed)})
    nodes = [Node(
        package='plansys2_combat_example',
        executable='combat_arbiter_node',
        name='combat_arbiter',
        output='screen',
        parameters=arbiter_params)]

    # The sides that plan: the argument, or else the world's
    armies = LaunchConfiguration('armies').perform(context).replace(',', ' ').split()
    if not armies:
        armies = world.get('armies', ['blue'])
    for side in armies:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('plansys2_bringup'),
                'launch', 'plansys2_bringup_launch_monolithic.py')),
            launch_arguments={
                'model_file': os.path.join(example_dir, 'pddl', 'combat.pddl'),
                'params_file': os.path.join(example_dir, 'config', 'plansys2_params.yaml'),
                'bt_builder_plugin': 'STNBTBuilder',
                'namespace': side,
            }.items()))

        # One performer per robot and action, only for that robot's actions, all in one process
        performers = {}
        for entry in world[side]['robots']:
            robot, kind, _ = entry.split()
            for action, nargs in ACTIONS[kind].items():
                performers[f'/{side}/{action}_{robot}'] = {'ros__parameters': {
                    'action_name': action,
                    # The robot is the first argument of every action
                    'specialized_arguments': [robot] + [''] * (nargs - 1),
                    'bt_xml_file': os.path.join(
                        example_dir, 'behavior_trees_xml', action + '.xml'),
                    'plugins': ['plansys2_combat_arbiter_action_bt_node'],
                }}
        params = tempfile.NamedTemporaryFile(
            mode='w', prefix=f'combat_{side}_', suffix='.yaml', delete=False)
        yaml.safe_dump(performers, params)
        params.close()
        nodes.append(Node(
            package='plansys2_combat_example',
            executable='combat_performers_node',
            namespace=side,
            output='screen',
            arguments=[name.rsplit('/', 1)[1] for name in performers],
            parameters=[params.name]))

        controller = Node(
            package='plansys2_combat_example',
            executable='combat_controller_node',
            name='combat_controller',
            namespace=side,
            output='screen',
            parameters=[{
                'goal': LaunchConfiguration('goal'),
                'metric': LaunchConfiguration('metric'),
                'colors': colors,
            }])
        nodes.append(controller)
        # A controller ends with the battle: everything else may end with it
        if LaunchConfiguration('shutdown_at_end').perform(context) == 'true':
            nodes.append(RegisterEventHandler(OnProcessExit(
                target_action=controller,
                on_exit=[Shutdown(reason='the battle is over')])))
    # The battle in RViz: the arbiter publishes the map, the robots' frames and the shots; the
    # world says which configuration frames its map
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(example_dir, 'rviz', world.get('rviz', 'combat.rviz'))],
        condition=IfCondition(LaunchConfiguration('rviz'))))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='fog_defense',
            description='World in config/worlds: fog_defense, known_defense or duel'),
        DeclareLaunchArgument(
            'goal', default_value='capture',
            description='capture: every control point; total: also destroy every enemy found'),
        DeclareLaunchArgument(
            'metric', default_value='minimize (spent)',
            description='Metric of the problem, e.g. minimize (+ (spent) (* 100 (total-time)))'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Show the battle in RViz2'),
        DeclareLaunchArgument(
            'colors', default_value='true',
            description='The battle told in colors: the arbiter yellow, blue cyan, red red'),
        DeclareLaunchArgument(
            'shutdown_at_end', default_value='false',
            description='Stop everything when the battle ends (else RViz keeps the last view)'),
        DeclareLaunchArgument(
            'seed', default_value='', description="Seed of the shots; by default, the world's"),
        DeclareLaunchArgument(
            'armies', default_value='',
            description='Sides that plan with their own PlanSys2, e.g. "blue red"; '
            'by default, those of the world'),
        OpaqueFunction(function=battle),
    ])

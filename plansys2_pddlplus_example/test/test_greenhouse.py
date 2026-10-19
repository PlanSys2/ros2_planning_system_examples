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

"""The greenhouse is watered: planned by Kairos and executed to its goal."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest


@pytest.mark.launch_test
def generate_test_description():
    example = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('plansys2_pddlplus_example'), 'launch',
                'plansys2_pddlplus_example.launch.yaml')))
    controller = Node(
        package='plansys2_pddlplus_example', executable='greenhouse_controller_node',
        output='screen')
    return (
        launch.LaunchDescription([
            # A domain of its own: what another test left shutting down does not answer here
            SetEnvironmentVariable('ROS_DOMAIN_ID', str(110 + os.getpid() % 100)),
            example, controller, launch_testing.actions.ReadyToTest()]),
        {'controller': controller})


class TestGreenhouse(unittest.TestCase):

    def test_the_controller_ends(self, proc_info, controller):
        # About 15 s of plan, and the start of PlanSys2
        proc_info.assertWaitForShutdown(process=controller, timeout=90)


@launch_testing.post_shutdown_test()
class TestGreenhouseResult(unittest.TestCase):

    def test_the_goal_was_reached(self, proc_info, controller):
        launch_testing.asserts.assertExitCodes(proc_info, process=controller)

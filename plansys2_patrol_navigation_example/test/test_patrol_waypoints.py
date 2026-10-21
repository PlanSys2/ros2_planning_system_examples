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

"""Patrolling two waypoints with the simulated EasyNav: the controller ends when both are done."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest


@pytest.mark.launch_test
def generate_test_description():
    example = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('plansys2_patrol_navigation_example'), 'launch',
                'patrol_example_fakesim.launch.yaml')),
        launch_arguments=[])
    # Once PlanSys2 is up, as the README runs it: a start at once is the robustness PR's (55)
    controller = ExecuteProcess(
        cmd=['bash', '-c',
             'until ros2 lifecycle get --no-daemon /executor 2>/dev/null | grep -q "^active"; '
             'do sleep 0.5; '
             'done; exec ros2 run '
             'plansys2_patrol_navigation_example patrol_waypoints_controller_node'],
        output='screen')
    return (
        launch.LaunchDescription([
            # A domain of its own: what another test left shutting down does not answer here
            SetEnvironmentVariable('ROS_DOMAIN_ID', str(110 + os.getpid() % 100)),
            example, controller, launch_testing.actions.ReadyToTest()]),
        {'controller': controller})


class TestController(unittest.TestCase):

    def test_the_controller_ends(self, proc_info, controller):
        proc_info.assertWaitForShutdown(process=controller, timeout=200)


@launch_testing.post_shutdown_test()
class TestControllerResult(unittest.TestCase):

    def test_the_goal_was_reached(self, proc_info, controller):
        launch_testing.asserts.assertExitCodes(proc_info, process=controller)

    def test_every_plan_ended_well(self, proc_output):
        # The execution trace of the executor: each plan says how it ended
        ends = [line for process in proc_output.processes() for output in proc_output[process]
                for line in output.text.decode().splitlines() if 'PLAN_END:' in line]
        self.assertTrue(ends)
        for end in ends:
            self.assertIn('PLAN_END: succeeded', end)

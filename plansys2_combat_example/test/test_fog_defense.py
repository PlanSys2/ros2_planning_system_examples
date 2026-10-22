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

"""One PlanSys2 against an enemy that holds still: blue takes every control point."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    battle = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('plansys2_combat_example'), 'launch',
                'fog_defense.launch.yaml')),
        launch_arguments=[('rviz', 'false'), ('shutdown_at_end', 'true')])
    return launch.LaunchDescription([
        # A domain of its own: what another test left shutting down does not answer here
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(110 + os.getpid() % 100)),
        battle, launch_testing.actions.ReadyToTest()])


class TestBattle(unittest.TestCase):

    def test_the_battle_ends_with_a_winner(self, proc_output):
        proc_output.assertWaitFor('THE BATTLE IS OVER', timeout=240, stream='stderr')
        # The winner's controller hears it from the arbiter
        proc_output.assertWaitFor('] VICTORY after', timeout=30, stream='stderr')


@launch_testing.post_shutdown_test()
class TestBattleEnd(unittest.TestCase):

    def test_every_controller_knows_the_result(self, proc_output):
        lines = [line for process in proc_output.processes() for output in proc_output[process]
                 for line in output.text.decode().splitlines()]
        over = [line for line in lines if 'THE BATTLE IS OVER' in line]
        self.assertEqual(len(over), 1, over)
        winner = over[0].split()[-2]
        self.assertIn('[%s] VICTORY' % winner, '\n'.join(lines))

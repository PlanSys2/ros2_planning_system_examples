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

"""The simple example: the README commands in the terminal, and the plan they make runs."""

import os
import tempfile
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

# What the terminal says when a command is wrong: the commands must all be right
WRONG = ['Usage:', 'not valid', 'Could not', 'could not', 'Command not found',
         'Finished with error']


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory('plansys2_simple_example')
    commands = tempfile.NamedTemporaryFile('w', suffix='.txt', delete=False)
    commands.write(f'source {share}/launch/commands\nget plan\nrun\nquit\n')
    commands.close()
    example = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(share, 'launch', 'plansys2_simple_example.launch.yaml')))
    # The terminal once the executor is active, as one would type
    terminal = ExecuteProcess(
        cmd=['bash', '-c',
             'until ros2 lifecycle get /executor 2>/dev/null | grep -q "^active"; do sleep 0.5; '
             f'done; ros2 run plansys2_terminal plansys2_terminal < {commands.name}'],
        output='screen')
    return (
        launch.LaunchDescription([
            # A domain of its own: what another test left shutting down does not answer here
            SetEnvironmentVariable('ROS_DOMAIN_ID', str(110 + os.getpid() % 100)),
            example, terminal, launch_testing.actions.ReadyToTest()]),
        {'terminal': terminal})


class TestTerminal(unittest.TestCase):

    def test_the_terminal_ends(self, proc_info, terminal):
        proc_info.assertWaitForShutdown(process=terminal, timeout=150)


@launch_testing.post_shutdown_test()
class TestTerminalResult(unittest.TestCase):

    def test_the_plan_succeeded(self, proc_info, proc_output, terminal):
        launch_testing.asserts.assertExitCodes(proc_info, process=terminal)
        said = ''.join(output.text.decode() for output in proc_output[terminal])
        self.assertIn('Successfully finished', said)
        for wrong in WRONG:
            self.assertNotIn(wrong, said)

    def test_every_plan_ended_well(self, proc_output):
        # The execution trace of the executor: each plan says how it ended
        ends = [line for process in proc_output.processes() for output in proc_output[process]
                for line in output.text.decode().splitlines() if 'PLAN_END:' in line]
        self.assertTrue(ends)
        for end in ends:
            self.assertIn('PLAN_END: succeeded', end)

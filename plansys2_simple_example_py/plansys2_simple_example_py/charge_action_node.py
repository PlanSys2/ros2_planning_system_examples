#!/usr/bin/python3

# Copyright 2023 Intelligent Robotics Lab
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

from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter


class ChargeAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('charge', 0.5)
        self.progress_ = 0.0

    def do_work(self):
        # As long as the plan says it lasts
        rate = self.get_parameter('rate').value
        self.progress_ += 1.0 / max(1.0, self.get_planned_duration() * rate)
        if self.progress_ < 1.0 - 1e-3:
            self.send_feedback(self.progress_, 'Charge running')
        else:
            self.finish(True, 1.0, 'Charge completed')
            self.progress_ = 0.0

        self.get_logger().info('charging ... {}'.format(self.progress_))


def main(args=None):
    rclpy.init(args=args)

    node = ChargeAction()
    node.set_parameters([Parameter(name='action_name', value='charge')])

    node.trigger_configure()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

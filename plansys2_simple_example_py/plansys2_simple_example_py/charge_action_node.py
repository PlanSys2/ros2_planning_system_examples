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

import rclpy
from rclpy.parameter import Parameter, ParameterType

from plansys2_support_py.ActionExecutorClient import ActionExecutorClient


class ChargeAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('charge', 0.5)
        self.progress_ = 0.0

    def do_work(self):
        if self.progress_ < 1.0:
            self.progress_ += 0.05
            self.send_feedback(self.progress_, 'Charge running')
        else:
            self.finish(True, 1.0, 'Charge completed');
            self.progress_ = 0.0
        
        self.get_logger().info('charging ... {}'.format(self.progress_))

def main(args=None):
    rclpy.init(args=args)

    node = ChargeAction()
    node.set_parameters([Parameter(name='action_name', value='charge')])

    node.trigger_configure()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


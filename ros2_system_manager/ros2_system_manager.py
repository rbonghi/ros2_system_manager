# GNU General Public License 3.0
#
# This file is part of the ros2_system_manager
# package (https://github.com/rbonghi/ros2_system_manager or http://rnext.it).
# Copyright (c) 2021 Raffaello Bonghi.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

from .exceptions import SystemManagerException
from .system_manager import system_manager
from .common import get_var


class system_manager_wrapper(Node):

    def __init__(self):
        super().__init__('system_manager')
        self.system_manager = system_manager()
        # Initialize services
        self.srv = self.create_service(Empty, 'shutdown', self.shutdown)
        self.srv = self.create_service(Empty, 'reboot', self.reboot)
        # Node started
        version = get_var()
        self.get_logger().info(f'System_manager {version} started')

    def shutdown(self, request, response):
        self.get_logger().info(f'System shutdown')
        # Run shutdown command
        self.system_manager.shutdown()
        return response

    def reboot(self, request, response):
        self.get_logger().info(f'System reboot')
        # Run reboot command
        self.system_manager.reboot()
        return response


def main(args=None):
    rclpy.init(args=args)
    # Start Nanosaur
    try:
        wrapper = system_manager_wrapper()
        try:
            rclpy.spin(wrapper)
        except (KeyboardInterrupt, SystemExit):
            pass
        # Destroy the node explicitly
        wrapper.destroy_node()
    except SystemManagerException as e:
        print(e)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# EOF

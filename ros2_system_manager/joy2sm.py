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
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

from .common import get_var


def make_mask(vec, size):
    return [True if n in vec else False for n in range(size)]


class Joy2SM(Node):
    
    def __init__(self):
        super().__init__('Joy2SM')
        self.declare_parameter("buttons", [4, 5])
        self.buttons = self.get_parameter("buttons").value
        self.declare_parameter("axes", [2, 5])
        self.axes = self.get_parameter("axes").value
        self.declare_parameter("timeout", 3)
        self._timeout_number = int(self.get_parameter("timeout").value)
        self.declare_parameter("name_service", 'shutdown')
        self._name_service = self.get_parameter("name_service").value
        self.cli = self.create_client(Empty, self._name_service)
        # Initialize timer
        self._first = None
        self.send_message = True
        self._timeout = Duration(seconds=self._timeout_number)
        # Register subcriber
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            1)
        self.subscription  # prevent unused variable warning
        # Get version
        version = get_var()
        self.get_logger().info(f'joy2sm {version} started')
        self.get_logger().info(f"Buttons: {self.buttons} - Axes: {self.axes}")
        self.get_logger().info(f"Timeout: {self._timeout_number} Service name: {self._name_service}")

    def joy_callback(self, msg):
        # Decode joystick message
        axes_buttons = [True if axe < 0 else False for axe in msg.axes]
        mask_axes = make_mask(self.axes, len(axes_buttons))
        buttons = [True if but == 1 else False for but in msg.buttons]
        mask_buttons = make_mask(self.buttons, len(buttons))
        # Check if all buttons are all pressed
        if (buttons == mask_buttons) and (axes_buttons == mask_axes):
            if self._first is None:
                self._first = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
                self.get_logger().info(f"All buttons pressed, wait {self._timeout_number} seconds")
            # Compare time
            now = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
            # Compare time
            if now - self._first > self._timeout and self.send_message:
                self.send_message = False
                self.get_logger().info(f"Call service {self._name_service}")
                # Send service message
                self.req = Empty.Request()
                self.future = self.cli.call_async(self.req)
        else:
            # Reset last time
            self._first = None
            self.send_message = True


def main(args=None):
    rclpy.init(args=args)
    # Start Nanosaur
    joy2sm = Joy2SM()
    try:
        rclpy.spin(joy2sm)
    except (KeyboardInterrupt, SystemExit):
        pass
    # Destroy the node explicitly
    joy2sm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# EOF

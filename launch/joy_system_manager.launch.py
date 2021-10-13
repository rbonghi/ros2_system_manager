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

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')

    # System manager
    system_manager = Node(
        package='ros2_system_manager',
        executable='system_manager',
        name='system_manager',
        output='screen'
    )

    launcher = [
        system_manager
        ]
    
    # teleoperation joystick nanosaur
    # only if joystick is connected
    if os.path.exists("/dev/input/js0"):
        print("Enable Joystick")
        launcher += [
            # Initialize joystick argument
            DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0', description='Joystick path'),
            # Run joy_node
            Node(package='joy',
                 executable='joy_node', name='joy_node',
                 parameters=[
                     {'dev': joy_dev,
                      'deadzone': 0.3,
                      'autorepeat_rate': 20.0,
                      }
                     ]
                 ),
            # Run Joystick to system_manager node
            Node(package='ros2_system_manager',
                 executable='joy2sm',
                 name='joy2sm',
                 output='screen'
                )
            ]

    return LaunchDescription(launcher)
# EOF

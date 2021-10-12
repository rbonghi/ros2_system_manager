#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# GNU AFFERO GENERAL PUBLIC LICENSE
#
# This file is part of the jetson_stats package (https://github.com/rbonghi/ros2_system_manager or http://rnext.it).
# Copyright (c) 2021 Raffaello Bonghi.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

from ros2_system_manager import SystemManagerException, system_manager


if __name__ == '__main__':
    print('ros2_system_manager - Send shutdown')

    try:
        sm_robot = system_manager()
        # Send shutdown
        sm_robot.shutdown()
    except SystemManagerException as e:
        print(e)
# EOF

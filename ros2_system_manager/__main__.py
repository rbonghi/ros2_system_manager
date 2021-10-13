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

import argparse
import logging
import re

from .common import get_var
from .exceptions import SystemManagerException
from .service import SystemManagerServer
# Create logger
logger = logging.getLogger(__name__)
# Version match
VERSION_RE = re.compile(r""".*__version__ = ["'](.*?)['"]""", re.S)


def main():
    parser = argparse.ArgumentParser(
        description='robot docker manager',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # Configuration
    parser.add_argument('--force',
                        dest='force',
                        help=argparse.SUPPRESS,
                        action='store_true',
                        default=False)
    parser.add_argument('--debug',
                        dest='debug',
                        help=argparse.SUPPRESS,
                        action='store_true',
                        default=False)
    parser.add_argument('-v', '--version',
                        action='version',
                        version='%(prog)s {version}'.format(version=get_var(VERSION_RE)))
    # Parse arguments
    args = parser.parse_args()
    try:
        # Initialize system manager server
        server = SystemManagerServer(force=args.force, debug=args.debug)
        logger.info('robot_manager server loaded')
        print('ROS2 system manager running')
        server.loop_for_ever()
    except SystemManagerException as e:
        print(e)


if __name__ == '__main__':
    main()
# EOF

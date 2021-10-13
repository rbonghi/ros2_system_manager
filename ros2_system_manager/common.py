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

import logging
from base64 import b64encode
from random import choice
import os
import re
from string import ascii_letters
# Load Author
AUTH_RE = re.compile(r""".*__author__ = ["'](.*?)['"]""", re.S)
# Version match
VERSION_RE = re.compile(r""".*__version__ = ["'](.*?)['"]""", re.S)
# Create logger
logger = logging.getLogger(__name__)


def get_var(MATCH_RE=VERSION_RE):
    # Load version package
    with open(os.path.join(os.path.abspath(os.path.dirname(__file__)), '__init__.py')) as fp:
        match = MATCH_RE.match(fp.read())
        value = match.group(1) if match else ''.join(choice(ascii_letters) for i in range(16))
    return value


def get_key():
    return str(b64encode(get_var(AUTH_RE).encode('utf-8')))
# EOF

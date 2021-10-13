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
import re
from multiprocessing import AuthenticationError, Event

from .exceptions import SystemManagerException
from .service import SystemManager
from .common import get_var
# Create logger
logger = logging.getLogger(__name__)
# Gain timeout lost connection
TIMEOUT_GAIN = 3

class system_manager:

    def __init__(self, interval=0.5):
        # Local Event thread
        self._trigger = Event()
        # Error message from thread
        self._error = None
        # Start server
        self._running = False
        # Load interval
        self._interval = float(interval)
        # Initialize observer
        self._observers = set()
        # Stats read from service
        self._stats = {}
        # Read stats
        SystemManager.register('get_queue')
        SystemManager.register('sync_data')
        SystemManager.register('sync_event')
        # Initialize broadcaster manager
        self._broadcaster = SystemManager()
        # Initialize connection
        self._start()

    def shutdown(self):
        # Send new nvpmodel
        self._controller.put({'system': 'shutdown'})

    def reboot(self):
        # Send new nvpmodel
        self._controller.put({'system': 'reboot'})

    def _get_server_config(self):
        while True:
            # Send configuration connection
            self._controller.put({'config': {}})
            # Return configuration
            data = self._controller.get(self._interval * TIMEOUT_GAIN)
            if 'config' in data:
                return data['config']

    def _start(self):
        # Connected to broadcaster
        try:
            self._broadcaster.connect()
        except FileNotFoundError as e:
            # Message error: 'No such file or directory' or 'Connection refused'
            if e.errno == 2 or e.errno == 111:
                raise SystemManagerException('The robot_manager.service is not active. \
                    Please run:\nsudo systemctl restart robot_manager.service')
            elif e.errno == 13:  # Message error: 'Permission denied'
                raise SystemManagerException('I cannot access ros2_system_manager.service.\
                    \nPlease logout or reboot this board.')
            else:
                raise FileNotFoundError(e)
        except ConnectionRefusedError as e:
            if e.errno == 111:  # Connection refused
                # When server is off but socket files exists in /run
                raise SystemManagerException('The ros2_system_manager.service is not active. \
                    Please run:\nsudo systemctl restart ros2_system_manager.service')
            else:
                raise ConnectionRefusedError(e)
        except PermissionError as e:
            if e.errno == 13:  # Permission denied
                raise SystemManagerException('I cannot access ros2_system_manager.service.\
                    \nPlease logout or reboot this board.')
            else:
                raise PermissionError(e)
        except ValueError:
            # https://stackoverflow.com/questions/54277946/queue-between-python2-and-python3
            raise SystemManagerException('Mismatch of Python versions between library and service')
        except AuthenticationError:
            raise SystemManagerException('Authentication with ros2_system_manager server failed')
        # Initialize synchronized data and condition
        self._controller = self._broadcaster.get_queue()
        self._sync_data = self._broadcaster.sync_data()
        self._sync_event = self._broadcaster.sync_event()
        # Initialize connection
        server_config = self._get_server_config()
        client_version = get_var()
        server_version = server_config['version']
        # Check server/client package
        if server_version != client_version:
            raise SystemManagerException(f"Mismatch server version:\
                                        \n Server: {server_version} \
                                        \n Client: {client_version} \
                                        \nPlease update package!")
# EOF

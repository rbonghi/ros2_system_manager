# -*- coding: UTF-8 -*-
# Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import logging
import os
import sys
from multiprocessing import Event, AuthenticationError
from threading import Thread
from .service import RobotManager
from .exceptions import RobotException
# Create logger
logger = logging.getLogger(__name__)
# Gain timeout lost connection
TIMEOUT_GAIN = 3

class robot_manager:    
    
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
        RobotManager.register('get_queue')
        RobotManager.register("sync_data")
        RobotManager.register('sync_event')
        # Initialize broadcaster manager
        self._broadcaster = RobotManager()
        # Initialize connection
        self._start()

    def shutdown(self):
        # Send new nvpmodel
        self._controller.put({'system': 'shutdown'})

    def reboot(self):
        # Send new nvpmodel
        self._controller.put({'system': 'reboot'})

    def _start(self):
        # Connected to broadcaster
        try:
            self._broadcaster.connect()
        except FileNotFoundError as e:
            if e.errno == 2 or e.errno == 111:  # Message error: 'No such file or directory' or 'Connection refused'
                raise RobotException("The robot_manager.service is not active. Please run:\nsudo systemctl restart robot_manager.service")
            elif e.errno == 13:  # Message error: 'Permission denied'
                raise RobotException("I can't access robot_manager.service.\nPlease logout or reboot this board.")
            else:
                raise FileNotFoundError(e)
        except ConnectionRefusedError as e:
            if e.errno == 111:  # Connection refused
                # When server is off but socket files exists in /run
                raise RobotException("The robot_manager.service is not active. Please run:\nsudo systemctl restart robot_manager.service")
            else:
                raise ConnectionRefusedError(e)
        except PermissionError as e:
            if e.errno == 13:  # Permission denied
                raise RobotException("I can't access robot_manager.service.\nPlease logout or reboot this board.")
            else:
                raise PermissionError(e)
        except ValueError:
            # https://stackoverflow.com/questions/54277946/queue-between-python2-and-python3
            raise RobotException("Mismatch of Python versions between library and service")
        except AuthenticationError:
            raise RobotException("Authentication with robot_manager server failed")
        # Initialize synchronized data and condition
        self._controller = self._broadcaster.get_queue()
        self._sync_data = self._broadcaster.sync_data()
        self._sync_event = self._broadcaster.sync_event()
# EOF
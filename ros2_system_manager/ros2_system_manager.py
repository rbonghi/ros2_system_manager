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

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from .system_manager import system_manager
from .exceptions import SystemManagerException


class system_manager_wrapper(Node):

    def __init__(self):
        super().__init__('system_manager')
        self.system_manager = system_manager()
        # Initialize services
        self.srv = self.create_service(Empty, 'shutdown', self.shutdown)
        self.srv = self.create_service(Empty, 'reboot', self.reboot)
        # Node started
        self.get_logger().info("Hello system_manager!")

    def shutdown(self, request, response):
        self.get_logger().info(f"System shutdown")
        # Run shutdown command
        self.system_manager.shutdown()
        return response

    def reboot(self, request, response):
        self.get_logger().info(f"System reboot")
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

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

import argparse
import os
import re
import logging
from random import choice
from string import ascii_letters
# Create logger
logger = logging.getLogger(__name__)
# Version match
VERSION_RE = re.compile(r""".*__version__ = ["'](.*?)['"]""", re.S)


def get_var(MATCH_RE):
    """
    Show the version of this package
    :return: Version number
    :rtype: string
    """
    # Load version package
    with open(os.path.join(os.path.abspath(os.path.dirname(__file__)), '../', "__init__.py")) as fp:
        match = MATCH_RE.match(fp.read())
        value = match.group(1) if match else ''.join(choice(ascii_letters) for i in range(16))
    return value

def main():
    parser = argparse.ArgumentParser(
        description='robot docker manager',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # Configuration
    parser.add_argument('-v', '--version', action='version', version='%(prog)s {version}'.format(version=get_var(VERSION_RE)))
    # Parse arguments
    args = parser.parse_args()
    # print
    print("robot docker manager")

if __name__ == "__main__":
    main()
# EOF
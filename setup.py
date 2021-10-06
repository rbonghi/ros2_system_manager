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

# Always prefer setuptools over distutils
from setuptools import setup, find_packages
from os import path
import os
import sys
import re
import logging

logging.basicConfig(stream=sys.stderr, level=logging.INFO)
log = logging.getLogger()


here = os.path.abspath(os.path.dirname(__file__))
project_homepage = "https://github.com/rbonghi/robot_docker_manager"
documentation_homepage = "https://github.com/rbonghi/robot_docker_manager"

with open(path.join(here, 'requirements.txt'), encoding='utf-8') as f:
    requirements = f.read().splitlines()

# Get the long description from the README file
with open(os.path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

# Load version package
with open(os.path.join(here, "robot_docker_manager", "__init__.py")) as fp:
    VERSION = (
        re.compile(
            r""".*__version__ = ["'](.*?)['"]""", re.S).match(fp.read()).group(1)
    )
# Store version package
version = VERSION

# Configuration setup module
setup(
    name="robot_docker_manager",
    version=version,
    author="Raffaello Bonghi",
    author_email="raffaello@rnext.it",
    description="robot docker manager",
    license='MIT',
    long_description=long_description,
    long_description_content_type="text/markdown",
    url=project_homepage,
    download_url=(project_homepage + "/archive/master.zip"),
    project_urls={
        "How To": documentation_homepage,
        "Examples": (project_homepage + "/tree/master/examples"),
        "Bug Reports": (project_homepage + "/issues"),
        "Source": (project_homepage + "/tree/master")
    },
    install_requires=requirements,
    packages=find_packages(
        exclude=['examples', 'scripts', 'tests']),  # Required
    # Load jetson_variables
    package_data={},
    # Define research keywords
    keywords=(),
    classifiers=["Development Status :: 4 - Beta",
                 # Audiencence and topics
                 "Intended Audience :: Developers",
                 # License
                 "License :: OSI Approved :: MIT License",
                 # Programming and Operative system
                 "Programming Language :: Python :: 3",
                 "Programming Language :: Python :: 3.4",
                 "Programming Language :: Python :: 3.5",
                 "Programming Language :: Python :: 3.6",
                 "Programming Language :: Python :: 3.7",
                 "Operating System :: POSIX :: Linux",
                 ],
    # Requisites
    # https://packaging.python.org/guides/distributing-packages-using-setuptools/#python-requires
    python_requires='>=3, <4',
    platforms=["linux", "linux2", "darwin"],
    # Zip safe configuration
    # https://setuptools.readthedocs.io/en/latest/setuptools.html#setting-the-zip-safe-flag
    zip_safe=False,
    # Add jetson_variables in /opt/jetson_stats
    # http://docs.python.org/3.4/distutils/setupscript.html#installing-additional-files
    data_files=[],
    # Install extra scripts
    cmdclass={},
    # The following provide a command called `robot_manager`
    entry_points={'console_scripts': [
        'robot_manager=robot_docker_manager.__main__:main',]},
)
# EOF

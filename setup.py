# -*- coding: UTF-8 -*-
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

# Always prefer setuptools over distutils
import grp
from glob import glob
from os import path
from shutil import copyfile
import logging
import os
import re
import sys

from setuptools import find_packages, setup
from setuptools.command.develop import develop
from setuptools.command.install import install


logging.basicConfig(stream=sys.stderr, level=logging.INFO)
log = logging.getLogger()


# https://gist.github.com/anantkamath/623ce7f5432680749e087cf8cfba9b69
def runningInDocker():
    with open('/proc/self/cgroup', 'r') as procfile:
        for line in procfile:
            fields = line.strip().split('/')
            if 'docker' in fields:
                return True
    return False


def check_systemctl():
    return os.path.exists('/bin/systemctl')


def is_superuser():
    return os.getuid() == 0


def list_scripts():
    # Load scripts to install
    scripts = []
    return scripts


def list_services():
    return ['services/{file}'.format(file=f) for f in os.listdir('services')
            if os.path.isfile(os.path.join('services', f))]


package_name = 'ros2_system_manager'
here = os.path.abspath(os.path.dirname(__file__))
project_homepage = 'https://github.com/rbonghi/ros2_system_manager'
documentation_homepage = 'https://github.com/rbonghi/ros2_system_manager'

with open(path.join(here, 'requirements.txt'), encoding='utf-8') as f:
    requirements = f.read().splitlines()

# Get the long description from the README file
with open(os.path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

# Load version package
with open(os.path.join(here, package_name, '__init__.py')) as fp:
    VERSION = (
        re.compile(
            r""".*__version__ = ["'](.*?)['"]""", re.S).match(fp.read()).group(1)
    )
# Store version package
version = VERSION


def install_services(copy):
    print(f'System prefix {sys.prefix}')
    # Make jetson stats folder
    root = sys.prefix + f'/lib/{package_name}/'
    if not os.path.exists(root):
        os.makedirs(root)
    # Copy all files
    for f_service in list_services():
        name_service = os.path.basename(f_service)
        print(f'Service name: {name_service}')
        folder, _ = os.path.split(__file__)
        path = root + name_service
        # Check if service is active
        if os.system('systemctl is-active --quiet {name_service}') == 0:
            # Stop service
            os.system(f'systemctl stop {name_service}')
            # Disable ros2_system_manager at startup
            os.system(f'systemctl disable {name_service}')
        # remove if exist file
        if os.path.exists(path):
            os.remove(path)
        # Copy or link file
        if copy:
            type_service = 'Copying'
            copyfile(folder + '/' + f_service, path)
        else:
            type_service = 'Linking'
            os.symlink(folder + '/' + f_service, path)
        # Prompt message
        print(f'{type_service} {name_service} -> {path}')
    # Reload all services
    os.system('systemctl daemon-reload')
    # Enable and start all services
    for f_service in list_services():
        name_service = os.path.basename(f_service)
        path = root + name_service
        # Link service
        copyfile(path, f'/etc/systemd/system/{name_service}')
        print(f'make {name_service} as a service')
        # Enable ros2_system_manager at startup
        os.system(f'systemctl enable {name_service}')
        # Start service
        os.system(f'systemctl start {name_service}')


def pre_installer(installer, obj, copy):
    group = 'ros2sm'
    # Get user
    user = os.getenv('SUDO_USER', os.getenv('USER'))
    if user is None:
        user = os.getenv('USER')
    # Install services
    if not runningInDocker() and is_superuser() and check_systemctl():
        print('Install services')
        # Install services
        install_services(copy)
        # Create group
        try:
            grp.getgrnam(group)
        except KeyError:
            print(f'Group {group} does not exist.')
            os.system(f'groupadd {group}')
        # Add system_manager to group
        os.system(f'usermod -a -G {group} {user}')
    # Run the default installation script
    installer.run(obj)


class PostInstallCommand(install):

    def run(self):
        # Run the uninstaller before to copy all scripts
        pre_installer(install, self, True)


class PostDevelopCommand(develop):

    def run(self):
        # Run the uninstaller before to copy all scripts
        # Install services (linking)
        pre_installer(develop, self, False)


# Configuration setup module
setup(
    name=package_name,
    version=version,
    author='Raffaello Bonghi',
    author_email='raffaello@rnext.it',
    maintainer='Raffaello Bonghi',
    maintainer_email='raffaello@rnext.it',
    description='ros system manager',
    license='GPLv3',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url=project_homepage,
    download_url=(project_homepage + '/archive/master.zip'),
    project_urls={
        'How To': documentation_homepage,
        'Examples': (project_homepage + '/tree/master/examples'),
        'Bug Reports': (project_homepage + '/issues'),
        'Source': (project_homepage + '/tree/master')
    },
    install_requires=requirements,
    packages=find_packages(
        exclude=['examples', 'scripts', 'tests']),  # Required
    # Load jetson_variables
    package_data={},
    # Define research keywords
    keywords=('ros2 foxy system manager'),
    tests_require=['pytest'],
    classifiers=['Development Status :: 4 - Beta',
                 # Audiencence and topics
                 'Intended Audience :: Developers',
                 # License
                 'License :: OSI Approved :: MIT License',
                 # Programming and Operative system
                 'Programming Language :: Python :: 3',
                 'Programming Language :: Python :: 3.4',
                 'Programming Language :: Python :: 3.5',
                 'Programming Language :: Python :: 3.6',
                 'Programming Language :: Python :: 3.7',
                 'Operating System :: POSIX :: Linux',
                 ],
    # Requisites
    # https://packaging.python.org/guides/distributing-packages-using-setuptools/#python-requires
    python_requires='>=3, <4',
    platforms=['linux', 'linux2', 'darwin'],
    # Zip safe configuration
    # https://setuptools.readthedocs.io/en/latest/setuptools.html#setting-the-zip-safe-flag
    zip_safe=True,
    # Add jetson_variables in /opt/jetson_stats
    # http://docs.python.org/3.4/distutils/setupscript.html#installing-additional-files
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['requirements.txt']),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name, ['LICENSE']),
        ('share/' + package_name, list_services()),
        (path.join('share', package_name), glob('launch/*.py'))
        ],
    # Install extra scripts
    scripts=list_scripts(),
    cmdclass={'develop': PostDevelopCommand,
              'install': PostInstallCommand},
    # The following provide a command called `ros2_system_manager`
    entry_points={'console_scripts': [
        'system_manager_server=ros2_system_manager.__main__:main',
        'system_manager=ros2_system_manager.ros2_system_manager:main',
        'joy2sm=ros2_system_manager.joy2sm:main',
        ]},
)
# EOF

#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     name='tesse_ros_bridge',
     version='0.0.1',
     description='TESSE/ROS bridge',
     packages=['tesse_ros_bridge'],
     package_dir={'': 'src'}
)

setup(**setup_args)

#!/usr/bin/env python
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['march_launch', 'march_launch.checks'],
    package_dir={'': 'src'},
    scripts=['scripts/march_rqt_software_check'],
)

setup(**d)

# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['march_state_machine', 'march_state_machine.states', 'march_state_machine.state_machines'],
    package_dir={'': 'src'},
    scripts=['scripts/march_state_machine'],
)

setup(**setup_args)

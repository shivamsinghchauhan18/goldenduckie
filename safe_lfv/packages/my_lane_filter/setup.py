## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['my_lane_filter', 'my_lane_filter_generic', 'my_lane_filter_generic_tests',
              'my_grid_helper', 'my_grid_helper_tests',
              ],
    package_dir={'': 'include'},
)

setup(**setup_args)

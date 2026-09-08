from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup(**generate_distutils_setup(packages=['competition_route_requester'], package_dir={'': 'src'}))

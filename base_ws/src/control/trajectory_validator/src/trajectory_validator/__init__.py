"""rqt support package for trajectory_validator.

ROS generates ``trajectory_validator.srv`` into the devel Python path while
the rqt plugin stays in this source package.  Make both locations visible as
one package at runtime.
"""
from pkgutil import extend_path

__path__ = extend_path(__path__, __name__)

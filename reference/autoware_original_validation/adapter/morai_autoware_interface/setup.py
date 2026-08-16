from glob import glob
import os

from setuptools import find_packages
from setuptools import setup


package_name = 'morai_autoware_interface'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ajou AutoRacing Study',
    maintainer_email='ajou@example.com',
    description='MORAI SIM interface for Autoware Universe 0.51',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'morai_autoware_interface = '
            'morai_autoware_interface.node:main',
            'morai_autoware_verifier = '
            'morai_autoware_interface.verifier:main',
        ],
    },
)

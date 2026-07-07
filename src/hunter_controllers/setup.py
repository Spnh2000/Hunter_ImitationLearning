from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hunter_controllers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'waypoints'), glob('waypoints/*.csv')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thesp',
    maintainer_email='sagharpanahi2@gmail.com',
    description='Path-tracking controllers for AgileX Hunter V2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hunter_controller_node = hunter_controllers.hunter_controller_node:main',
            'waypoint_loader        = hunter_controllers.waypoint_loader:main',
            'waypoint_recorder      = hunter_controllers.waypoint_recorder:main',
        ],
    },
)

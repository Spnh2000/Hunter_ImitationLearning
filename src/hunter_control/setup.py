import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hunter_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simaes',
    maintainer_email='simaes@rptu.de',
    description='PID path tracker and benchmarking metrics for the AgileX Hunter V2 simulation (macOS compatible)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_tracker = hunter_control.pid_tracker:main',
            'performance_metrics = hunter_control.performance_metrics:main',
        ],
    },
)

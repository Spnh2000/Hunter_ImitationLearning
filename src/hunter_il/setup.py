from setuptools import find_packages, setup

package_name = 'hunter_il'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/train_default.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Saghar',
    maintainer_email='sagharpanahi2@gmail.com',
    description='Phase 8 imitation-learning harness (no ROS).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'il_train = hunter_il.train:main',
            'il_eval_offline = hunter_il.eval_offline:main',
            'il_diagnostics = hunter_il.diagnostics:main',
            'il_train_all = hunter_il.train_all:main',
        ],
    },
)

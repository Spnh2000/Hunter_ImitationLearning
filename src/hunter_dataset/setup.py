from setuptools import find_packages, setup

package_name = 'hunter_dataset'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/record_expert.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Saghar',
    maintainer_email='sagharpanahi2@gmail.com',
    description='Phase 6 imitation dataset schema, converter and inspector.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convert_dataset = hunter_dataset.convert_dataset:main',
            'inspect_dataset = hunter_dataset.inspect_dataset:main',
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vehicle_control_unit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autovhc',
    maintainer_email='autovhc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rc_bridge_node = vehicle_control_unit.rc_bridge_node:main',
            'vesc_bridge_node = vehicle_control_unit.vesc_bridge_node:main',

        ],
    },
)

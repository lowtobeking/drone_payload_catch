from setuptools import setup
import os
from glob import glob

package_name = 'payload_catch'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='caolihao',
    maintainer_email='caolihao@todo.local',
    description='Aerial payload drop-and-catch: A releases, B intercepts in mid-air',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        # ROS 2 节点在 M1 接入（planner_node / payload_node / b_controller_node /
        # capture_monitor / a_hold_node）。核心算法层不依赖 ROS，无需 entry point。
        'console_scripts': [],
    },
)

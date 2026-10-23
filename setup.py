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
        (os.path.join('share', package_name, 'deploy'), [p for p in glob(os.path.join('deploy','*')) if os.path.isfile(p)]),
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
        'console_scripts': [
            'mocap_source = payload_catch.mocap_source_node:main',
            'mocap_odometry = payload_catch.mocap_odometry_node:main',
            'mocap_preview = payload_catch.mocap_preview_node:main',
            'a_solo_validation = payload_catch.solo_validation_node:main',
            'fleet_link = payload_catch.fleet_link:main',
            'a_node       = payload_catch.a_node:main',
            'b_node       = payload_catch.b_node:main',
            'payload_node = payload_catch.payload_node:main',
            'relnav_node  = payload_catch.relnav_node:main',
            'safety_supervisor = payload_catch.safety_supervisor:main',
            'release_guard = payload_catch.release_guard:main',
            'payload_servo = payload_catch.payload_servo_node:main',
            'pwm33_restore = payload_catch.servo_pwm:main',
        ],
    },
)

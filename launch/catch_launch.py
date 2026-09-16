#!/usr/bin/env python3
"""M1 SITL 启动：A 悬停释放 + B 会合捕获（2 机）。

用法（先按 run_m1_sitl.sh 起 gz + 2×PX4 + agent，再本 launch；或直接用 run_m1_sitl.sh）：
  ros2 launch payload_catch catch_launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # A（drone0）：世界系悬停/释放点
    a_hover = LaunchConfiguration('a_hover')
    # B（drone1）：待命点 + 本机 PX4 原点在世界 NED 的位置
    b_standby = LaunchConfiguration('b_standby')
    b_offset = LaunchConfiguration('b_offset')
    release_delay = LaunchConfiguration('release_delay')
    model_path = LaunchConfiguration('model_path')

    a = Node(package='payload_catch', executable='a_node', name='a_node', output='screen',
             parameters=[{'drone_id': 0, 'hover_world': a_hover}])
    b = Node(package='payload_catch', executable='b_node', name='b_node', output='screen',
             parameters=[{'drone_id': 1, 'standby_world': b_standby, 'world_offset': b_offset,
                          'a_release_world': a_hover, 'start_delay': 18.0,
                          'capture_radius': 0.35, 'capture_rel_speed': 1.5,
                          'replan_dt': 0.10, 'b_max_speed': 9.0, 'b_max_accel': 10.0,
                          'catch_alt_min': 1.5, 'catch_alt_max': 2.8, 'kp_pos': 2.0}])
    payload = Node(package='payload_catch', executable='payload_node', name='payload_node',
                   output='screen',
                   parameters=[{'release_pos': a_hover, 'release_vel': [0.0, 0.0, 0.0],
                                'release_delay': release_delay, 'use_gazebo': True,
                                'model_path': model_path}])
    return LaunchDescription([
        DeclareLaunchArgument('a_hover', default_value='[0.0, 0.0, -3.0]'),
        DeclareLaunchArgument('b_standby', default_value='[0.5, 0.0, -3.5]'),
        DeclareLaunchArgument('b_offset', default_value='[0.6, 0.0, 0.0]'),
        DeclareLaunchArgument('release_delay', default_value='25.0'),
        DeclareLaunchArgument('model_path',
                              default_value='/home/caolihao/drone_payload_catch/models/payload/model.sdf'),
        a, b, payload,
    ])

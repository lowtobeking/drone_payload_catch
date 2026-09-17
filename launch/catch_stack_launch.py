#!/usr/bin/env python3
"""M6 垂直堆叠投放 SITL 启动：A 正上方释放 + B 对正/温和下潜软捕获。

布局（世界系 NED）：
  A 悬停在 [a_x, a_y, -a_alt]（默认 4.5m）；B 待命 [a_x, a_y, -b_alt]（默认 3.5m，正下方）。
  B 起飞 → 飞到 A 正下方 → 水平速度归零、投影重合 → 广播释放 → 温和下潜 → 捕获。

相对定位：A 把自身世界位姿发到 `/drone_a/state`，B 订阅后加噪声/延迟（mesh 替身）。

用法（先用 run_m6_sitl.sh 起 gz + 2×PX4 + agent，或直接本 launch）：
  ros2 launch payload_catch catch_stack_launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    a_hover = LaunchConfiguration('a_hover')        # A 悬停/释放点（世界系 NED）
    b_standby = LaunchConfiguration('b_standby')    # B 待命点（世界系 NED，应在 A 正下方）
    b_offset = LaunchConfiguration('b_offset')      # B 的 PX4 原点在世界 NED
    release_offset = LaunchConfiguration('release_offset')  # 载荷相对 A 的释放偏移(NED)
    model_path = LaunchConfiguration('model_path')

    a = Node(package='payload_catch', executable='a_node', name='a_node', output='screen',
             parameters=[{'drone_id': 0, 'hover_world': a_hover, 'publish_state': True}])
    b = Node(package='payload_catch', executable='b_node', name='b_node', output='screen',
             parameters=[{'drone_id': 1, 'mode': 'stack',
                          'standby_world': b_standby, 'world_offset': b_offset,
                          'a_release_world': a_hover,
                          'a_state_topic': '/drone_a/state',
                          'rel_pos_sigma': 0.03, 'rel_latency': 0.05,
                          'align_xy_tol': 0.12, 'align_vel_tol': 0.12, 'align_alt_tol': 0.20,
                          'align_hold_s': 1.0, 'release_lead': 0.20,
                          'a_dive': 3.0, 'a_brake': 6.0,
                          'funnel_mouth_radius': 0.20, 'funnel_eff_radius': 0.14,
                          'funnel_mount_height': 0.21, 'payload_release_offset': 0.15,
                          'px4_z_bias': 0.24, 'catch_z_tol': 0.12,
                          'funnel_depth': 0.30,
                          'funnel_restitution': 0.60, 'v_retain': 4.04,
                          'b_max_speed': 5.0, 'b_max_accel': 6.0,
                          'stack_kp_xy': 1.5, 'stack_kp_z': 1.5}])
    payload = Node(package='payload_catch', executable='payload_node', name='payload_node',
                   output='screen',
                   parameters=[{'release_pos': a_hover, 'release_vel': [0.0, 0.0, 0.0],
                                'release_delay': 100000.0,   # 只等 B 的释放广播
                                'use_gazebo': True, 'use_a_state': True,
                                'a_state_topic': '/drone_a/state',
                                'release_offset': release_offset,
                                'model_path': model_path}])
    return LaunchDescription([
        DeclareLaunchArgument('a_hover', default_value='[0.0, 0.0, -4.5]'),
        DeclareLaunchArgument('b_standby', default_value='[0.0, 0.0, -3.5]'),
        DeclareLaunchArgument('b_offset', default_value='[5.0, 0.0, 0.0]'),
        DeclareLaunchArgument('release_offset', default_value='[0.0, 0.0, 0.15]'),
        DeclareLaunchArgument('model_path',
                              default_value='/home/caolihao/drone_payload_catch/models/payload/model.sdf'),
        a, b, payload,
    ])

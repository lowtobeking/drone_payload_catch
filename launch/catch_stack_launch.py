#!/usr/bin/env python3
"""M6 垂直堆叠投放 SITL 启动：A 正上方释放 + B 对正/温和下潜软捕获 + 两机分开落地。

布局（世界系 NED）：
  A 悬停 [a_x,a_y,-a_alt]（默认 4.5m）；B 待命在 A 正下方（默认 3.5m）。
  B 起飞→垂直爬升→等 A 到位→定高横移到 A 正下方→对正→释放→温和下潜→刚性漏斗捕获→分开落地。

相对定位：# A 广播 /drone_a/state，B 订阅后加 延迟/抖动/丢包/慢变偏置/噪声（mesh 替身）。
  B 在 DIVE 阶段跟踪【载荷】本身（带测量噪声/延迟）形成闭环。

用法：
  ros2 launch payload_catch catch_stack_launch.py
  ros2 launch payload_catch catch_stack_launch.py rel_pos_sigma:=0.10 release_xy_sigma:=0.10
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _f(arg):
    return ParameterValue(LaunchConfiguration(arg), value_type=float)


def _i(arg):
    return ParameterValue(LaunchConfiguration(arg), value_type=int)


def _b(arg):
    return ParameterValue(LaunchConfiguration(arg), value_type=bool)


def generate_launch_description():
    a_hover = LaunchConfiguration('a_hover')
    b_standby = LaunchConfiguration('b_standby')
    b_offset = LaunchConfiguration('b_offset')
    release_offset = LaunchConfiguration('release_offset')
    model_path = LaunchConfiguration('model_path')

    # ── 可扫描参数（默认值 = 标称）─────────────────────────────────────────
    args = {
        'rel_pos_sigma': '0.03', 'rel_latency': '0.05', 'rel_jitter': '0.02',
        'rel_dropout': '0.0', 'rel_bias': '0.0', 'rel_seed': '0', 'est_lpf_alpha': '0.30',
        'payload_meas_sigma': '0.0', 'payload_meas_latency': '0.0', 'payload_dropout': '0.0',
        'release_xy_sigma': '0.0', 'release_seed': '0',
        'align_xy_tol': '0.12', 'align_vel_tol': '0.12', 'align_alt_tol': '0.20',
        'align_hold_s': '1.0', 'release_lead': '0.20',
        'a_dive': '3.0', 'a_brake': '6.0',
        'funnel_mouth_radius': '0.20', 'funnel_eff_radius': '0.14',
        'funnel_mount_height': '0.21', 'funnel_depth': '0.30',
        'funnel_restitution': '0.60', 'v_retain': '4.04',
        'b_max_speed': '5.0', 'b_max_accel': '6.0',
        'stack_kp_xy': '1.5', 'stack_kp_z': '1.5',
        'px4_z_bias': '0.24', 'catch_z_tol': '0.12',
        'min_ab_gap': '0.80', 'approach_alt_tol': '0.15', 'payload_release_offset': '0.15',
        'land_after_catch_s': '6.0', 'land_xy_tol': '0.25',
        'track_payload': 'true',
    }
    decls = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]

    a = Node(package='payload_catch', executable='a_node', name='a_node', output='screen',
             parameters=[{'drone_id': 0, 'hover_world': a_hover, 'publish_state': True,
                          'auto_land': True, 'land_after_catch_s': _f('land_after_catch_s'),
                          'land_xy': [-4.0, 0.0], 'land_xy_tol': _f('land_xy_tol')}])
    b = Node(package='payload_catch', executable='b_node', name='b_node', output='screen',
             parameters=[{
                 'drone_id': 1, 'mode': 'stack',
                 'standby_world': b_standby, 'world_offset': b_offset,
                 'a_release_world': a_hover, 'a_state_topic': '/drone_a/state',
                 'track_payload': _b('track_payload'),
                 'rel_pos_sigma': _f('rel_pos_sigma'), 'rel_latency': _f('rel_latency'),
                 'rel_jitter': _f('rel_jitter'), 'rel_dropout': _f('rel_dropout'),
                 'rel_bias': _f('rel_bias'), 'rel_seed': _i('rel_seed'),
                 'est_lpf_alpha': _f('est_lpf_alpha'),
                 'payload_meas_sigma': _f('payload_meas_sigma'),
                 'payload_meas_latency': _f('payload_meas_latency'),
                 'payload_dropout': _f('payload_dropout'),
                 'align_xy_tol': _f('align_xy_tol'), 'align_vel_tol': _f('align_vel_tol'),
                 'align_alt_tol': _f('align_alt_tol'), 'align_hold_s': _f('align_hold_s'),
                 'release_lead': _f('release_lead'),
                 'a_dive': _f('a_dive'), 'a_brake': _f('a_brake'),
                 'funnel_mouth_radius': _f('funnel_mouth_radius'),
                 'funnel_eff_radius': _f('funnel_eff_radius'),
                 'funnel_mount_height': _f('funnel_mount_height'),
                 'funnel_depth': _f('funnel_depth'),
                 'funnel_restitution': _f('funnel_restitution'), 'v_retain': _f('v_retain'),
                 'b_max_speed': _f('b_max_speed'), 'b_max_accel': _f('b_max_accel'),
                 'stack_kp_xy': _f('stack_kp_xy'), 'stack_kp_z': _f('stack_kp_z'),
                 'px4_z_bias': _f('px4_z_bias'), 'catch_z_tol': _f('catch_z_tol'),
                 'min_ab_gap': _f('min_ab_gap'), 'approach_alt_tol': _f('approach_alt_tol'),
                 'payload_release_offset': _f('payload_release_offset'),
                 'auto_land': True, 'land_after_catch_s': _f('land_after_catch_s'),
                 'land_xy': [5.0, 0.0], 'land_xy_tol': _f('land_xy_tol'),
             }])
    payload = Node(package='payload_catch', executable='payload_node', name='payload_node',
                   output='screen',
                   parameters=[{'release_pos': a_hover, 'release_vel': [0.0, 0.0, 0.0],
                                'release_delay': 100000.0,
                                'use_gazebo': True, 'use_a_state': True,
                                'a_state_topic': '/drone_a/state',
                                'release_offset': release_offset,
                                'release_xy_sigma': _f('release_xy_sigma'),
                                'release_seed': _i('release_seed'),
                                'model_path': model_path}])
    return LaunchDescription([
        DeclareLaunchArgument('a_hover', default_value='[0.0, 0.0, -4.5]'),
        DeclareLaunchArgument('b_standby', default_value='[0.0, 0.0, -3.5]'),
        DeclareLaunchArgument('b_offset', default_value='[5.0, 0.0, 0.0]'),
        DeclareLaunchArgument('release_offset', default_value='[0.0, 0.0, 0.15]'),
        DeclareLaunchArgument('model_path',
                              default_value='/home/caolihao/drone_payload_catch/models/payload/model.sdf'),
        *decls, a, b, payload,
    ])

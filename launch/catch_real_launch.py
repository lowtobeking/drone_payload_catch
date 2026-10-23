#!/usr/bin/env python3
"""catch_real_launch.py —— **真机** M6 任务启动（无 Gazebo、无 PX4 SITL）。

与 SITL（catch_stack_launch.py）的区别：
  · 不启动 gz / payload_node（载荷状态来自**真实感知**源，需另跑一个发布 /payload/state 的节点）；
  · A 不再广播真值：a_node publish_state:=false；改由 **relnav_node** 从 RTK/视觉发布 /drone_a/state；
  · b_node 打开 **contact_detect**（真机接触事件触发捕获/锁扣），rel_* 噪声置 0。

真机前置（见 report/real_hardware_bringup.md）：
  · 两机 RTK（或 UWB/动捕）已发布到 a_rtk_topic / b_rtk_topic；
  · B 的 PX4 本地 NED 与 relnav 的世界原点已标定（b_offset）；
  · 释放机构、末端（托盘/漏斗）、接触检测（可选 /payload/contact）就绪。

用法（先 source 真机环境）：
  ros2 launch payload_catch catch_real_launch.py a_hover:="[0,0,-4.5]" b_standby:="[0,0,-3.5]" \
    b_offset:="[5.0,0,0]" source:=rtk a_rtk_topic:=/rtk/a b_rtk_topic:=/rtk/b \
    a_lever:="[实测天线到机体原点]" b_lever:="[实测天线到机体原点]"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import hashlib
import json
import os
import re
import socket
import subprocess
from payload_catch.fleet_runtime import load_fleet
import yaml
from payload_catch.payload_servo import ServoConfig, servo_signature


def _f(a):
    return ParameterValue(LaunchConfiguration(a), value_type=float)


def _b(a):
    return ParameterValue(LaunchConfiguration(a), value_type=bool)


def generate_launch_description(default_role='both'):
    a_hover = LaunchConfiguration('a_hover')
    b_standby = LaunchConfiguration('b_standby')
    b_offset = LaunchConfiguration('b_offset')

    args = {
        'deployment_role': default_role,
        'fleet_config': '',
        'release_id': '',
        'servo_external': 'false',
        'servo_config': str(Path(get_package_share_directory('payload_catch')) / 'config/payload_servo.yaml'),
        'preflight_report': '',
        'preflight_require_gps': 'true',
        'a_hover': '[0.0,0.0,-4.5]',
        'b_standby': '[0.0,0.0,-3.5]',
        'b_offset': '[0.0,0.0,0.0]',
        # 相对定位驱动
        'source': 'rtk',               # rtk | px4（均要求真实观测和标定）
        'a_rtk_topic': '/rtk/a', 'b_rtk_topic': '/rtk/b',
        'rtk_type': 'navsatfix', 'use_geodetic': 'true',
        'origin_lat': '0.0', 'origin_lon': '0.0', 'origin_alt': '0.0',
        'a_lever': '[0.0,0.0,0.0]', 'b_lever': '[0.0,0.0,0.0]',
        'a_lpos_topic': '/fmu/out/vehicle_local_position',
        'b_lpos_topic': '/px4_1/fmu/out/vehicle_local_position',
        'b_att_topic': '/px4_1/fmu/out/vehicle_attitude',
        # 末端/捕获（托盘默认）
        'funnel_mouth_radius': '0.15', 'funnel_eff_radius': '0.12',
        'funnel_mount_height': '0.21', 'funnel_depth': '0.05',
        'funnel_restitution': '0.15', 'v_retain': '6.60',
        'px4_z_bias': '0.24', 'catch_z_tol': '0.10',
        'a_dive': '3.0', 'auto_min_dive': 'true', 'a_brake': '6.0',
        # 相对不确定度模型（真机用 relative + 公共抵消 ρ）
        'sigma_model': 'relative', 'sigma_rho': '0.8', 'sigma_a': '0.15', 'sigma_sensor': '0.03',
        'lever_a': '0.2', 'lever_b': '0.21', 'sigma_att_a': '0.05', 'sigma_att_b': '0.05',
        'gate_use_relative': 'true',
        # 真机接触检测
        'contact_detect': 'true', 'contact_accel_thresh': '15.0',
        'contact_topic': '/payload/contact', 'contact_timeout_s': '0.30',
        # 安全
        'safety_geofence_xy': '50.0', 'safety_geofence_alt': '30.0',
        'safety_alt_min': '-1.0', 'safety_auto_kill': 'false',
        # 协同
        'coord_mode': 'handshake',
        'land_after_catch_s': '6.0',
    }
    decls = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]

    return LaunchDescription([*decls, OpaqueFunction(function=_nodes)])


def _nodes(context):
    a_hover=LaunchConfiguration('a_hover')
    b_standby=LaunchConfiguration('b_standby')
    b_offset=LaunchConfiguration('b_offset')
    role=LaunchConfiguration('deployment_role').perform(context)
    if role not in ('a','b','both'):raise ValueError('deployment_role must be a, b or both')
    fingerprint=''
    if role!='both':
        fleet_path=LaunchConfiguration('fleet_config').perform(context)
        fleet,servo_path,digest=load_fleet(fleet_path)
        release_id=LaunchConfiguration('release_id').perform(context)
        if not re.fullmatch(r'[0-9a-f]{64}',release_id):raise ValueError('verified release_id required')
        if os.environ.get('ROS_DOMAIN_ID')!=str(fleet['domain_id']) or os.environ.get('ROS_LOCALHOST_ONLY')!='0':
            raise ValueError('load generated fleet network.env first')
        if os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':raise ValueError('fleet requires Fast DDS')
        if os.environ.get('FLEET_CONFIG_DIGEST')!=digest or os.environ.get('FLEET_ROLE')!=role:
            raise ValueError('regenerate network.env for this configuration and role')
        host=fleet['hosts'][role]
        if socket.gethostname()!=host['hostname']:raise ValueError('wrong host for requested role')
        addresses=json.loads(subprocess.check_output(['ip','-j','-4','address'],text=True))
        if not any(a.get('local')==host['ip'] for link in addresses for a in link.get('addr_info',[])):
            raise ValueError('configured flight network IP missing')
        allowed=set(context.launch_configurations)-{'deployment_role','fleet_config','release_id','preflight_report','servo_config','servo_external'}
        for key,value in fleet['launch'].items():
            if key not in allowed:raise ValueError('unsupported fleet launch setting: '+key)
            context.launch_configurations[key]=(json.dumps(value) if isinstance(value,list) else str(value).lower() if type(value) is bool else str(value))
        context.launch_configurations['servo_config']=str(servo_path)
        fingerprint=hashlib.sha256((release_id+digest).encode()).hexdigest()
    config_path=LaunchConfiguration('servo_config').perform(context)
    values=yaml.safe_load(Path(config_path).read_text())['payload_servo']['ros__parameters']
    cfg=ServoConfig(**{k:v for k,v in values.items() if k in ServoConfig.__dataclass_fields__})
    cfg.validate()
    if values.get('backend','jetson')!='jetson':
        raise ValueError('real launch requires jetson backend; mock uses isolated test topics')
    if cfg.enabled and not cfg.calibrated:
        raise ValueError('servo enabled without measured latch calibration')
    if cfg.enabled and LaunchConfiguration('coord_mode').perform(context)!='handshake':
        raise ValueError('PWM flight release requires the dual-aircraft handshake')
    # Plans always describe actual detachment; readiness still blocks disabled hardware.
    delay=cfg.release_delay_s
    relnav = Node(package='payload_catch', executable='relnav_node', name='relnav_node',
                  output='screen',
                  parameters=[{
                      'source': LaunchConfiguration('source'),
                      'out_topic': '/drone_a/state',
                      'a_rtk_topic': LaunchConfiguration('a_rtk_topic'),
                      'b_rtk_topic': LaunchConfiguration('b_rtk_topic'),
                      'rtk_type': LaunchConfiguration('rtk_type'),
                      'use_geodetic': _b('use_geodetic'),
                      'origin_lat': _f('origin_lat'), 'origin_lon': _f('origin_lon'),
                      'origin_alt': _f('origin_alt'),
                      'a_lever': LaunchConfiguration('a_lever'),
                      'b_lever': LaunchConfiguration('b_lever'),
                      'a_lpos_topic': LaunchConfiguration('a_lpos_topic'),
                      'b_lpos_topic': LaunchConfiguration('b_lpos_topic'),
                      'b_att_topic': LaunchConfiguration('b_att_topic'),
                      'b_world_offset': b_offset,
                  }])

    a = Node(package='payload_catch', executable='a_node', name='a_node', output='screen',
             parameters=[{'release_confirmation_mode':cfg.confirmation_mode,
                           'expected_servo_signature':servo_signature(cfg),
                           'expected_release_id':LaunchConfiguration('release_id'),
                           'require_fleet_link':role!='both','drone_id': 0, 'hover_world': a_hover,
                           'preflight_report': LaunchConfiguration('preflight_report'),
                           'preflight_require_gps': _b('preflight_require_gps'),
                           'preflight_require_rc': True,
                          'release_lead': delay + .2,
                          'publish_state': False,     # 真机：A 不广播真值，改由 relnav
                          'gate_use_relative': _b('gate_use_relative'),
                          'coord_mode': LaunchConfiguration('coord_mode'),
                          'safety_kill_topic': '/safety/kill_a',
                          'safety_geofence_xy': _f('safety_geofence_xy'),
                          'safety_geofence_alt': _f('safety_geofence_alt'),
                          'safety_alt_min': _f('safety_alt_min'),
                          'auto_land': True}])

    b = Node(package='payload_catch', executable='b_node', name='b_node', output='screen',
             parameters=[{
                  'release_confirmation_mode':cfg.confirmation_mode,
                  'expected_servo_signature':servo_signature(cfg) if role!='both' else '',
                  'expected_release_id':LaunchConfiguration('release_id') if role!='both' else '',
                  'require_fleet_link':role!='both', 'drone_id': 1, 'mode': 'stack',
                  'preflight_report': LaunchConfiguration('preflight_report'),
                  'preflight_require_gps': _b('preflight_require_gps'),
                  'preflight_require_rc': True,
                  'peer_loss_hold_s': .5, 'peer_loss_land_s': 3.,
                  'require_hardware_status': True, 'release_lead': delay + .2,
                  'require_payload_before_release': True, 'require_physical_capture': True,
                 'standby_world': b_standby, 'world_offset': b_offset,
                 'a_state_topic': '/drone_a/state',
                 'rel_pos_sigma': 0.0, 'rel_latency': 0.0, 'rel_jitter': 0.0,
                 'rel_dropout': 0.0, 'rel_bias': 0.0,
                 'payload_meas_sigma': 0.0, 'payload_meas_latency': 0.0,
                 'funnel_mouth_radius': _f('funnel_mouth_radius'),
                 'funnel_eff_radius': _f('funnel_eff_radius'),
                 'funnel_mount_height': _f('funnel_mount_height'),
                 'funnel_depth': _f('funnel_depth'),
                 'funnel_restitution': _f('funnel_restitution'),
                 'v_retain': _f('v_retain'),
                 'px4_z_bias': _f('px4_z_bias'), 'catch_z_tol': _f('catch_z_tol'),
                 'a_dive': _f('a_dive'), 'auto_min_dive': _b('auto_min_dive'),
                 'a_brake': _f('a_brake'),
                 'sigma_model': LaunchConfiguration('sigma_model'),
                 'sigma_a': _f('sigma_a'), 'sigma_rho': _f('sigma_rho'),
                 'sigma_sensor': _f('sigma_sensor'),
                 'lever_a': _f('lever_a'), 'lever_b': _f('lever_b'),
                 'sigma_att_a': _f('sigma_att_a'), 'sigma_att_b': _f('sigma_att_b'),
                 'contact_detect': _b('contact_detect'),
                 'contact_accel_thresh': _f('contact_accel_thresh'),
                 'contact_topic': LaunchConfiguration('contact_topic'),
                 'contact_timeout_s': _f('contact_timeout_s'),
                 'coord_mode': LaunchConfiguration('coord_mode'),
                 'safety_kill_topic': '/safety/kill_b',
                 'safety_geofence_xy': _f('safety_geofence_xy'),
                 'safety_geofence_alt': _f('safety_geofence_alt'),
                 'safety_alt_min': _f('safety_alt_min'),
                 'auto_land': True, 'land_after_catch_s': _f('land_after_catch_s'),
             }])

    safety=Node(package='payload_catch',executable='safety_supervisor',output='screen',
                parameters=[{'require_rc':True}])
    release=Node(package='payload_catch',executable='release_guard',output='screen',parameters=[{
        'release_delay_s': delay,
        'authority':ParameterValue(PythonExpression(["0 if '",LaunchConfiguration('coord_mode'),"' == 'handshake' else 1"]),value_type=int)}])
    servo=Node(package='payload_catch',executable='payload_servo',name='payload_servo',
        output='screen',parameters=[config_path,{'owner':0,'release_id':LaunchConfiguration('release_id')}])
    if role=='both':return [safety, release, servo, relnav, a, b]
    monitor=Node(package='payload_catch',executable='fleet_link',name='fleet_link_'+role,
        output='screen',parameters=[{'role':role,'fingerprint':fingerprint}])
    external=LaunchConfiguration('servo_external').perform(context)=='true'
    return ([monitor,safety,release,a]+([] if external else [servo])) if role=='a' else [monitor,relnav,b]

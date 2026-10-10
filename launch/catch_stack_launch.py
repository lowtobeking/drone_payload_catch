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
        'use_b_sigma': 'true', 'release_sigma_max': '0.15',  # A 余量闸用 B 在线 σ
        'gate_use_relative': 'false',  # 启发式闸用相对 σ_rel（修绝对 σ 过度保守）
        'sigma_model': 'legacy',       # legacy | relative（相对不确定度模型）
        'sigma_a': '0.0', 'sigma_rho': '0.0', 'sigma_sensor': '0.0',
        'lever_a': '0.0', 'lever_b': '0.0',
        'sigma_att_a': '0.0', 'sigma_att_b': '0.0',
        'release_gate_mode': 'heuristic',   # heuristic | certificate（C1 证书闸）
        'cert_eps': '0.05', 'cert_sigma_track': '0.02',
        # 传感器/估计器约束（读 PX4 EKF 已有字段）
        'sensor_constraints_enable': 'true',
        'sensor_use_ekf_sigma': 'true',
        'sensor_watchdog_enable': 'true',
        'sensor_use_est_limits': 'true',
        'sensor_eph_max': '0.50', 'sensor_epv_max': '0.50',
        'sensor_reset_hold_s': '1.0',
        'sensor_watchdog_heading': 'false',
        # 姿态/角速率约束
        'attitude_constraint_enable': 'true',
        'tilt_soft_deg': '25.0', 'tilt_hard_deg': '40.0',
        'rate_soft_dps': '150.0', 'rate_hard_dps': '300.0',
        'accel_h_max': '5.0',
        'align_xy_tol': '0.12', 'align_vel_tol': '0.20', 'align_alt_tol': '0.20',
        'align_hold_s': '0.6', 'release_lead': '0.20',
        'commit_hold_s': '0.20',      # A 就绪门限需持续多久才提交释放
        'a_dive': '3.0', 'auto_min_dive': 'true', 'a_brake': '6.0',
        'miss_timeout_s': '4.0',    # DIVE 超时未捕获 → MISS，安全悬停→降落
        'funnel_mouth_radius': '0.20', 'funnel_eff_radius': '0.14',
        'funnel_mount_height': '0.21', 'funnel_depth': '0.30',
        'funnel_restitution': '0.60', 'v_retain': '4.04',
        'contact_detect': 'false', 'contact_accel_thresh': '15.0',
        'contact_topic': '/payload/contact', 'contact_timeout_s': '0.30',
        'b_max_speed': '5.0', 'b_max_accel': '6.0',
        'stack_kp_xy': '1.2', 'stack_kp_z': '1.5',
        'a_ff_gain': '1.0',           # DIVE 加速度前馈增益（0=关）
        'keepout_dist': '0.60',       # 3D 反应式 keep-out 触发距离 (m)
        'keepout_gain': '1.0',        # keep-out 排斥增益 (1/s)
        'keepout_mode': 'heuristic',  # heuristic | cbf（C5）
        'keepout_alpha': '1.0',       # CBF 指数增益
        'keepout_delay_s': '0.0',     # T3 延迟鲁棒收紧用的通信延迟 (s)
        'collide_warn': '0.90',       # 硬碰撞地板：预警带 (m, 3D)
        'collide_emerg': '0.50',      # 硬碰撞地板：紧急 (m, 3D)
        'collide_hold_frames': '15',  # 连续紧急多少拍→HOLD
        'peer_loss_hold_s': '0.0',    # 失联看门狗：丢弃冻结阈值 (0=关)
        'peer_loss_land_s': '0.0',    # 失联看门狗：降落阈值（须 > hold）
        'adaptive_dive': 'false',     # 在线自适应下潜
        'adaptive_alt_floor': '0.35', # 自适应下潜：刹停后最小离地 (m)
        'sp_rate_limit': '0.0',       # 速度设定点变化率上限 (m/s²)，0=不限
        'jerk_max': '0.0',            # 加速度变化率上限 (m/s³)，0=不限
        'px4_z_bias': '0.24', 'catch_z_tol': '0.10',
        'min_ab_gap': '0.80', 'approach_alt_tol': '0.15', 'payload_release_offset': '0.15',
        'safety_k': '2.0', 'rel_sigma_floor': '0.0',
        'land_after_catch_s': '6.0', 'land_xy_tol': '0.25',
        'track_payload': 'true',
        # ── 编队同速投放（M6-moving）：formation_vel 非零即启用 ──
        'formation_vel': '[0.0, 0.0, 0.0]',
        'formation_topic': '/formation/start',
        'formation_min_speed_ratio': '0.8',
        'dive_anchor_vz': '0.5',
        'capture_min_vz': '1.0',
        'attach_to_a': 'false',
        'attach_topic': '/payload/attach',
        'detach_topic': '/payload/detach',
        'lock_to_b': 'false',
        'lock_request_topic': '/payload/lock_request',
        'lock_model_path': '',
        'coord_mode': 'direct',        # direct | handshake（协同释放握手）
        'use_intent': 'false',         # B 用 A 广播的预测落点做对正
        'wind_est': '[0.0, 0.0, 0.0]', # A 的风估计 (NED)，用于预测落点漂移
        'use_px4_wind': 'false',       # A 用 PX4 EKF 风估计（/fmu/out/wind）
        'zem_gain': '0.0',             # B 终端导引(ZEM) 增益
        'safety_auto_kill': 'false',   # 异常持续时自动飞行终止
        'safety_tilt_max_deg': '60.0', # 姿态角上限
        'safety_kill_hold_s': '0.8',   # 异常持续多久才 kill
        # 分级安全响应（HOLD 悬停 / PULLBACK 越界回拉 / LAND 降落 / KILL 飞行终止）
        'safety_pullback_enable': 'true',  # 越界时主动回拉（而非只 kill）
        'safety_pullback_k': '1.0',        # 回拉比例增益
        'safety_pullback_speed': '2.0',    # 回拉速度上限 (m/s)
        'safety_pullback_clear': '0.15',   # 回拉恢复滞环 (m)
        'safety_alt_min': '-1.0',          # 高度下界 (m)
        'safety_geofence_xy': '50.0',      # 水平围栏半径 (m)
        'safety_geofence_alt': '30.0',     # 高度上限 (m)
        'safety_hold_escalate': 'none',    # 持续 HOLD/临界后升级：none|land
        'safety_hold_timeout': '8.0',      # HOLD 多久后升级 (s)
        'align_reset_tol': '0.25',     # 编队：短晩失配容忍
        'formation_timeout_s': '12.0', # 编队释放超时→中止投放
    }
    decls = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]

    a = Node(package='payload_catch', executable='a_node', name='a_node', output='screen',
             parameters=[{'drone_id': 0, 'hover_world': a_hover, 'publish_state': True,
                          'formation_vel': LaunchConfiguration('formation_vel'),
                          'formation_topic': LaunchConfiguration('formation_topic'),
                          'coord_mode': LaunchConfiguration('coord_mode'),
                          'wind_est': LaunchConfiguration('wind_est'),
                          'use_px4_wind': _b('use_px4_wind'),
                          'safety_kill_topic': '/safety/kill_a',
                          'safety_auto_kill': _b('safety_auto_kill'),
                          'safety_tilt_max_deg': _f('safety_tilt_max_deg'),
                          'safety_kill_hold_s': _f('safety_kill_hold_s'),
                          'safety_pullback_enable': _b('safety_pullback_enable'),
                          'safety_pullback_k': _f('safety_pullback_k'),
                          'safety_pullback_speed': _f('safety_pullback_speed'),
                          'safety_pullback_clear': _f('safety_pullback_clear'),
                          'safety_alt_min': _f('safety_alt_min'),
                          'safety_geofence_xy': _f('safety_geofence_xy'),
                          'safety_geofence_alt': _f('safety_geofence_alt'),
                          'safety_hold_escalate': LaunchConfiguration('safety_hold_escalate'),
                          'safety_hold_timeout': _f('safety_hold_timeout'),
                          'sp_rate_limit': _f('sp_rate_limit'),
                          'jerk_max': _f('jerk_max'),
                          'use_b_sigma': _b('use_b_sigma'),
                          'release_sigma_max': _f('release_sigma_max'),
                          'gate_use_relative': _b('gate_use_relative'),
                          'release_gate_mode': LaunchConfiguration('release_gate_mode'),
                          'cert_eps': _f('cert_eps'),
                          'cert_sigma_track': _f('cert_sigma_track'),
                          'sensor_constraints_enable': _b('sensor_constraints_enable'),
                          'sensor_use_ekf_sigma': _b('sensor_use_ekf_sigma'),
                          'sensor_watchdog_enable': _b('sensor_watchdog_enable'),
                          'sensor_use_est_limits': _b('sensor_use_est_limits'),
                          'sensor_eph_max': _f('sensor_eph_max'),
                          'sensor_epv_max': _f('sensor_epv_max'),
                          'sensor_reset_hold_s': _f('sensor_reset_hold_s'),
                          'sensor_watchdog_heading': _b('sensor_watchdog_heading'),
                          'attitude_constraint_enable': _b('attitude_constraint_enable'),
                          'tilt_soft_deg': _f('tilt_soft_deg'), 'tilt_hard_deg': _f('tilt_hard_deg'),
                          'rate_soft_dps': _f('rate_soft_dps'), 'rate_hard_dps': _f('rate_hard_dps'),
                          'accel_h_max': _f('accel_h_max'),
                          'commit_hold_s': _f('commit_hold_s'),
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
                 'formation_vel': LaunchConfiguration('formation_vel'),
                 'formation_topic': LaunchConfiguration('formation_topic'),
                 'formation_min_speed_ratio': _f('formation_min_speed_ratio'),
                 'coord_mode': LaunchConfiguration('coord_mode'),
                 'use_intent': _b('use_intent'),
                 'zem_gain': _f('zem_gain'),
                 'align_reset_tol': _f('align_reset_tol'),
                 'formation_timeout_s': _f('formation_timeout_s'),
                 'safety_kill_topic': '/safety/kill_b',
                 'safety_auto_kill': _b('safety_auto_kill'),
                 'safety_tilt_max_deg': _f('safety_tilt_max_deg'),
                 'safety_kill_hold_s': _f('safety_kill_hold_s'),
                 'safety_pullback_enable': _b('safety_pullback_enable'),
                 'safety_pullback_k': _f('safety_pullback_k'),
                 'safety_pullback_speed': _f('safety_pullback_speed'),
                 'safety_pullback_clear': _f('safety_pullback_clear'),
                 'safety_alt_min': _f('safety_alt_min'),
                 'safety_geofence_xy': _f('safety_geofence_xy'),
                 'safety_geofence_alt': _f('safety_geofence_alt'),
                 'safety_hold_escalate': LaunchConfiguration('safety_hold_escalate'),
                 'safety_hold_timeout': _f('safety_hold_timeout'),
                 'a_dive': _f('a_dive'), 'auto_min_dive': _b('auto_min_dive'),
                 'a_brake': _f('a_brake'), 'miss_timeout_s': _f('miss_timeout_s'),
                 'funnel_mouth_radius': _f('funnel_mouth_radius'),
                 'funnel_eff_radius': _f('funnel_eff_radius'),
                 'funnel_mount_height': _f('funnel_mount_height'),
                 'funnel_depth': _f('funnel_depth'),
                 'funnel_restitution': _f('funnel_restitution'), 'v_retain': _f('v_retain'),
                 'contact_detect': _b('contact_detect'),
                 'contact_accel_thresh': _f('contact_accel_thresh'),
                 'contact_topic': LaunchConfiguration('contact_topic'),
                 'contact_timeout_s': _f('contact_timeout_s'),
                 'b_max_speed': _f('b_max_speed'), 'b_max_accel': _f('b_max_accel'),
                 'stack_kp_xy': _f('stack_kp_xy'), 'stack_kp_z': _f('stack_kp_z'),
                 'a_ff_gain': _f('a_ff_gain'), 'sp_rate_limit': _f('sp_rate_limit'),
                 'jerk_max': _f('jerk_max'),
                 'keepout_dist': _f('keepout_dist'), 'keepout_gain': _f('keepout_gain'),
                 'keepout_mode': LaunchConfiguration('keepout_mode'),
                 'keepout_alpha': _f('keepout_alpha'),
                 'keepout_delay_s': _f('keepout_delay_s'),
                 'collide_warn': _f('collide_warn'),
                 'collide_emerg': _f('collide_emerg'),
                 'collide_hold_frames': _i('collide_hold_frames'),
                 'peer_loss_hold_s': _f('peer_loss_hold_s'),
                 'peer_loss_land_s': _f('peer_loss_land_s'),
                 'sensor_constraints_enable': _b('sensor_constraints_enable'),
                 'sensor_use_ekf_sigma': _b('sensor_use_ekf_sigma'),
                 'sensor_watchdog_enable': _b('sensor_watchdog_enable'),
                 'sensor_use_est_limits': _b('sensor_use_est_limits'),
                 'sensor_eph_max': _f('sensor_eph_max'),
                 'sensor_epv_max': _f('sensor_epv_max'),
                 'sensor_reset_hold_s': _f('sensor_reset_hold_s'),
                 'sensor_watchdog_heading': _b('sensor_watchdog_heading'),
                 'attitude_constraint_enable': _b('attitude_constraint_enable'),
                 'tilt_soft_deg': _f('tilt_soft_deg'), 'tilt_hard_deg': _f('tilt_hard_deg'),
                 'rate_soft_dps': _f('rate_soft_dps'), 'rate_hard_dps': _f('rate_hard_dps'),
                 'accel_h_max': _f('accel_h_max'),
                 'adaptive_dive': _b('adaptive_dive'),
                 'adaptive_alt_floor': _f('adaptive_alt_floor'),
                 'px4_z_bias': _f('px4_z_bias'), 'catch_z_tol': _f('catch_z_tol'),
                 'dive_anchor_vz': _f('dive_anchor_vz'),
                 'capture_min_vz': _f('capture_min_vz'),
                 'min_ab_gap': _f('min_ab_gap'), 'approach_alt_tol': _f('approach_alt_tol'),
                 'safety_k': _f('safety_k'), 'rel_sigma_floor': _f('rel_sigma_floor'),
                 'sigma_model': LaunchConfiguration('sigma_model'),
                 'sigma_a': _f('sigma_a'), 'sigma_rho': _f('sigma_rho'),
                 'sigma_sensor': _f('sigma_sensor'),
                 'lever_a': _f('lever_a'), 'lever_b': _f('lever_b'),
                 'sigma_att_a': _f('sigma_att_a'), 'sigma_att_b': _f('sigma_att_b'),
                 'payload_release_offset': _f('payload_release_offset'),
                 'lock_to_b': _b('lock_to_b'),
                 'lock_request_topic': LaunchConfiguration('lock_request_topic'),
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
                                'attach_to_a': _b('attach_to_a'),
                                'attach_topic': LaunchConfiguration('attach_topic'),
                                'detach_topic': LaunchConfiguration('detach_topic'),
                                'formation_topic': LaunchConfiguration('formation_topic'),
                                'release_xy_sigma': _f('release_xy_sigma'),
                                'release_seed': _i('release_seed'),
                                'lock_to_b': _b('lock_to_b'),
                                'lock_request_topic': LaunchConfiguration('lock_request_topic'),
                                'lock_model_path': LaunchConfiguration('lock_model_path'),
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

#!/usr/bin/env python3
"""PX4 无人机接口基类（ROS 2 / PX4-1.16）—— A/B 节点共用。

封装：
  · 话题命名（drone0: /fmu/...；drone_i: /px4_i/fmu/...）
  · QoS（订阅 PX4 out=TRANSIENT_LOCAL；发布到 PX4 in=VOLATILE）
  · 状态订阅（vehicle_status_v1 / vehicle_local_position / vehicle_attitude）
  · offboard 心跳 + 速度/位置 setpoint + VehicleCommand（ARM / DO_SET_MODE / LAND）
  · ARM+OFFBOARD 重试、起飞到目标高度、悬停

⚠️ PX4-1.16 话题版本化：vehicle_status → `vehicle_status_v1`；
   vehicle_local_position / vehicle_attitude / trajectory_setpoint / offboard_control_mode 无后缀。
   （见 report/env_bringup.md 与 SITL仿真调试记忆）
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleAttitude, VehicleCommand, VehicleLocalPosition,
                          VehicleStatus)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from .safety_logic import (attitude_govern, clip_to_estimator_limits,
                           fence_velocity, safety_decision, scale01,
                           sensor_health_reasons)

PREFIX = {0: 'out/vehicle_status_v1', 1: 'out/vehicle_status_v1'}   # 1.16


def qos_in() -> QoSProfile:
    """订阅 PX4 'out'（PX4 DataWriter 用 TRANSIENT_LOCAL）。"""
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=5,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def qos_out() -> QoSProfile:
    """发布到 PX4 'in'（PX4 DataReader 用 VOLATILE）。"""
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=5,
                      durability=DurabilityPolicy.VOLATILE)


def topic_for(drone_id: int, suffix: str) -> str:
    return f'/fmu/{suffix}' if drone_id == 0 else f'/px4_{drone_id}/fmu/{suffix}'


def yaw_from_quat(q) -> float:
    w, x, y, z = q[0], q[1], q[2], q[3]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def roll_pitch_from_quat(q) -> tuple:
    w, x, y, z = q[0], q[1], q[2], q[3]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


class Px4Drone(Node):
    def __init__(self, node_name: str, default_id: int = 0,
                 control_hz: float = 50.0, px4_version: str = '1.16',
                 auto_arm: bool = True):
        super().__init__(node_name)
        self.declare_parameter('drone_id', default_id)
        self.declare_parameter('control_hz', control_hz)
        self.declare_parameter('px4_version', px4_version)
        self.declare_parameter('auto_arm', auto_arm)
        self.declare_parameter('world_offset', [0.0, 0.0, 0.0])   # 本机 PX4 原点在世界 NED 中的位置
        self.declare_parameter('sp_rate_limit', 0.0)   # 速度设定点变化率上限 (m/s²)，0=不限
        # ── 安全监督（可 kill / 飞行终止）──
        self.declare_parameter('safety_lock', True)          # 启用安全监督
        self.declare_parameter('safety_kill_topic', '/safety/kill')   # 外部 kill（Bool）
        self.declare_parameter('safety_auto_kill', False)    # 异常持续时自动飞行终止
        self.declare_parameter('safety_tilt_max_deg', 60.0)  # 姿态角上限（超即异常）
        self.declare_parameter('safety_geofence_xy', 50.0)   # 水平边界 (m)
        self.declare_parameter('safety_geofence_alt', 30.0)  # 高度上限 (m)
        self.declare_parameter('safety_state_timeout', 2.0)  # 位置/状态超时 (s)
        self.declare_parameter('safety_kill_hold_s', 0.8)    # 异常持续多久才 kill
        # 分级安全响应：OK → (HOLD 悬停 | PULLBACK 越界回拉) → LAND → KILL
        self.declare_parameter('safety_pullback_enable', True)  # 越界时主动回拉（而非只 kill）
        self.declare_parameter('safety_pullback_k', 1.0)        # 回拉比例增益
        self.declare_parameter('safety_pullback_speed', 2.0)    # 回拉速度上限 (m/s)
        self.declare_parameter('safety_pullback_clear', 0.15)   # 回拉恢复滞环 (m)
        self.declare_parameter('safety_alt_min', -1.0)          # 高度下界 (m，离地)
        self.declare_parameter('safety_hold_escalate', 'none')  # 持续 HOLD/临界后升级：none|land
        self.declare_parameter('safety_hold_timeout', 8.0)      # HOLD 多久后升级 (s)
        # ── 传感器/估计器约束（读 PX4 EKF 已有字段，不加新硬件）──
        self.declare_parameter('sensor_constraints_enable', True)  # 总开关
        self.declare_parameter('sensor_use_ekf_sigma', True)       # 用 eph/epv 作 σ
        self.declare_parameter('sensor_watchdog_enable', True)     # 有效性/健康/跳变看门狗
        self.declare_parameter('sensor_use_est_limits', True)      # 用估计器限值 vxy/vz/hagl
        self.declare_parameter('sensor_eph_max', 0.50)             # 水平位置 σ 上限 (m)
        self.declare_parameter('sensor_epv_max', 0.50)             # 垂直位置 σ 上限 (m)
        self.declare_parameter('sensor_reset_hold_s', 1.0)         # 估计器跳变后保持多久 (s)
        self.declare_parameter('sensor_watchdog_heading', False)   # 航向可用性也当硬约束（本仿真常 false，默认关）
        # ── 姿态/角速率约束（IMU→安全滤波，全部来自已订阅的 attitude/local_position）──
        self.declare_parameter('attitude_constraint_enable', True)
        self.declare_parameter('tilt_soft_deg', 25.0)     # 倾角软限：超过则衰减水平指令
        self.declare_parameter('tilt_hard_deg', 40.0)     # 倾角硬限：达到则水平指令=0
        self.declare_parameter('rate_soft_dps', 150.0)    # 角速率软限 (deg/s)
        self.declare_parameter('rate_hard_dps', 300.0)    # 角速率硬限 (deg/s)
        self.declare_parameter('accel_h_max', 5.0)        # 水平指令加速度上限 (m/s²)
        self.drone_id = int(self.get_parameter('drone_id').value)
        self.hz = float(self.get_parameter('control_hz').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.world_offset = np.asarray(self.get_parameter('world_offset').value,
                                       float).reshape(3)
        self.sp_rate_limit = float(self.get_parameter('sp_rate_limit').value)
        self._last_vsp = None          # 上一次速度设定点（用于速率限幅）
        _v = str(self.get_parameter('px4_version').value)
        self._vs_topic = ('out/vehicle_status_v1' if _v.startswith('1.16')
                          else 'out/vehicle_status_v4' if _v.startswith('main')
                          else 'out/vehicle_status')
        self._lp_topic = ('out/vehicle_local_position' if _v.startswith(('1.16', '1.14'))
                          else 'out/vehicle_local_position_v1')

        qi, qo = qos_in(), qos_out()
        self._pos = np.zeros(3)          # PX4 本地系 NED
        self._vel = np.zeros(3)
        self._yaw = 0.0
        self._armed = False
        self._nav_state = 0
        self._pos_ok = False
        self._att_ok = False
        self._offboard_confirmed = False
        self._last_pos_t = 0.0
        self._landing = False          # 已发出着陆指令：停止 offboard，交回 PX4
        self._land_cmd_count = 0
        # 安全监督状态
        self._roll = 0.0
        self._pitch = 0.0
        self._killed = False           # 已发出飞行终止
        self._bad_since = None
        self.safety_lock = bool(self.get_parameter('safety_lock').value)
        self.safety_auto_kill = bool(self.get_parameter('safety_auto_kill').value)
        self.safety_tilt_max = math.radians(float(self.get_parameter('safety_tilt_max_deg').value))
        self.safety_geofence_xy = float(self.get_parameter('safety_geofence_xy').value)
        self.safety_geofence_alt = float(self.get_parameter('safety_geofence_alt').value)
        self.safety_state_timeout = float(self.get_parameter('safety_state_timeout').value)
        self.safety_kill_hold_s = float(self.get_parameter('safety_kill_hold_s').value)
        self.safety_pullback_enable = bool(self.get_parameter('safety_pullback_enable').value)
        self.safety_pullback_k = float(self.get_parameter('safety_pullback_k').value)
        self.safety_pullback_speed = float(self.get_parameter('safety_pullback_speed').value)
        self.safety_pullback_clear = float(self.get_parameter('safety_pullback_clear').value)
        self.safety_alt_min = float(self.get_parameter('safety_alt_min').value)
        self.safety_hold_escalate = str(self.get_parameter('safety_hold_escalate').value).lower()
        self.safety_hold_timeout = float(self.get_parameter('safety_hold_timeout').value)
        # 传感器/估计器约束状态
        self.sensor_constraints_enable = bool(self.get_parameter('sensor_constraints_enable').value)
        self.sensor_use_ekf_sigma = bool(self.get_parameter('sensor_use_ekf_sigma').value)
        self.sensor_watchdog_enable = bool(self.get_parameter('sensor_watchdog_enable').value)
        self.sensor_use_est_limits = bool(self.get_parameter('sensor_use_est_limits').value)
        self.sensor_eph_max = float(self.get_parameter('sensor_eph_max').value)
        self.sensor_epv_max = float(self.get_parameter('sensor_epv_max').value)
        self.sensor_reset_hold_s = float(self.get_parameter('sensor_reset_hold_s').value)
        self.sensor_watchdog_heading = bool(self.get_parameter('sensor_watchdog_heading').value)
        self.attitude_constraint_enable = bool(self.get_parameter('attitude_constraint_enable').value)
        self.tilt_soft = math.radians(float(self.get_parameter('tilt_soft_deg').value))
        self.tilt_hard = math.radians(float(self.get_parameter('tilt_hard_deg').value))
        self.rate_soft = math.radians(float(self.get_parameter('rate_soft_dps').value))
        self.rate_hard = math.radians(float(self.get_parameter('rate_hard_dps').value))
        self.accel_h_max = float(self.get_parameter('accel_h_max').value)
        self._last_att = None          # (roll, pitch, yaw, t) 供角速率差分
        self._ang_rate = 0.0           # 角速率（最大分量，rad/s）
        self._att_gov = 1.0            # 上一次姿态约束缩放因子（1=未限制）
        self._lpos_valid = False
        self._eph = 0.0
        self._epv = 0.0
        self._evh = 0.0
        self._evv = 0.0
        self._dead_reckoning = False
        self._heading_good = True
        self._reset_counters = None
        self._reset_seen_t = None
        self._est_vxy_max = float('inf')
        self._est_vz_max = float('inf')
        self._est_hagl_min = float('inf')
        # 分级安全状态机：OK | HOLD | PULLBACK | LAND | KILL
        self._safety_state = 'OK'
        self._safety_reason = ''
        self._hold_since = None
        from std_msgs.msg import Bool
        self.create_subscription(Bool, str(self.get_parameter('safety_kill_topic').value),
                                 self._on_kill, 10)

        self.create_subscription(VehicleStatus, topic_for(self.drone_id, self._vs_topic),
                                 self._on_status, qi)
        self.create_subscription(VehicleLocalPosition, topic_for(self.drone_id, self._lp_topic),
                                 self._on_lpos, qi)
        self.create_subscription(VehicleAttitude, topic_for(self.drone_id, 'out/vehicle_attitude'),
                                 self._on_att, qi)
        self.pub_mode = self.create_publisher(
            OffboardControlMode, topic_for(self.drone_id, 'in/offboard_control_mode'), qo)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, topic_for(self.drone_id, 'in/trajectory_setpoint'), qo)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, topic_for(self.drone_id, 'in/vehicle_command'), qo)

        self._t0 = self.get_clock().now()
        self._last_pos_t = self._t0.nanoseconds * 1e-9   # 避免首帧前误报 pos_stale
        self._tick_count = 0
        self.timer = self.create_timer(1.0 / self.hz, self._tick)
        self.get_logger().info(
            f'[{node_name}] drone_id={self.drone_id} px4={_v} '
            f'status_topic={self._vs_topic} pos_topic={self._lp_topic}')

    # ------------------------------------------------------------------ 状态
    def _on_status(self, msg):
        self._armed = (msg.arming_state == 2)
        self._nav_state = msg.nav_state

    def _on_lpos(self, msg):
        if math.isfinite(msg.x) and math.isfinite(msg.y) and math.isfinite(msg.z):
            self._pos = np.array([msg.x, msg.y, msg.z])
            self._vel = np.array([msg.vx, msg.vy, msg.vz])
            self._pos_ok = bool(msg.xy_valid and msg.z_valid)
            self._last_pos_t = self.get_clock().now().nanoseconds * 1e-9
        # ── 传感器/估计器派生量（约束用，全部来自已有 EKF 输出）──
        self._lpos_valid = bool(msg.xy_valid and msg.z_valid
                                and msg.v_xy_valid and msg.v_z_valid)
        self._eph = float(msg.eph) if math.isfinite(msg.eph) else 0.0
        self._epv = float(msg.epv) if math.isfinite(msg.epv) else 0.0
        self._evh = float(msg.evh) if math.isfinite(msg.evh) else 0.0
        self._evv = float(msg.evv) if math.isfinite(msg.evv) else 0.0
        self._dead_reckoning = bool(msg.dead_reckoning)
        self._heading_good = bool(msg.heading_good_for_control)
        # 估计器限值（PX4 用 INFINITY 表示“不限制”）
        self._est_vxy_max = float(msg.vxy_max) if math.isfinite(msg.vxy_max) else float('inf')
        self._est_vz_max = float(msg.vz_max) if math.isfinite(msg.vz_max) else float('inf')
        self._est_hagl_min = (float(msg.hagl_min) if math.isfinite(msg.hagl_min)
                              else float('inf'))
        # 估计器跳变检测（reset counter 变化）
        counters = (int(msg.xy_reset_counter), int(msg.z_reset_counter),
                    int(msg.vxy_reset_counter), int(msg.vz_reset_counter),
                    int(msg.heading_reset_counter))
        if self._reset_counters is not None and counters != self._reset_counters:
            self._reset_seen_t = self.get_clock().now().nanoseconds * 1e-9
        self._reset_counters = counters

    def _on_att(self, msg):
        yaw = yaw_from_quat(msg.q)
        roll, pitch = roll_pitch_from_quat(msg.q)
        # 角速率：对角姿态时间序列做有限差分（vehicle_angular_velocity 在本 RMW 未发布）
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_att is not None:
            dt = max(now - self._last_att[3], 1e-3)
            d = np.array([roll - self._last_att[0], pitch - self._last_att[1],
                          yaw - self._last_att[2]])
            d = (d + np.pi) % (2.0 * np.pi) - np.pi      # 角度回绕
            self._ang_rate = float(np.max(np.abs(d)) / dt)
        self._last_att = (roll, pitch, yaw, now)
        self._yaw = yaw
        self._roll, self._pitch = roll, pitch
        self._att_ok = True

    # ------------------------------------------------------------ 安全监督
    def _on_kill(self, msg):
        """外部 kill（Bool）：真则立即飞行终止。"""
        if bool(msg.data):
            self.kill('external /safety/kill')

    def kill(self, reason: str = '') -> None:
        """飞行终止（切动力）：发 MAV_CMD_DO_FLIGHTTERMINATION 并停发 offboard。"""
        if self._killed:
            return
        self._killed = True
        self._safety_state = 'KILL'
        self._landing = True           # 停止 offboard setpoint
        cmd = getattr(VehicleCommand, 'VEHICLE_CMD_DO_FLIGHTTERMINATION', 185)
        self.send_command(cmd, p1=1.0)
        self.get_logger().error(
            f'[{self.drone_id}] *** SAFETY KILL（飞行终止）*** reason={reason}')

    def request_hold(self, reason: str = '') -> None:
        """请求 HOLD（原地悬停，停止任务追击，可恢复）。"""
        if self._killed or self._landing or self._safety_state == 'PULLBACK':
            return
        if self._safety_state != 'HOLD':
            self.get_logger().warn(f'[{self.drone_id}] SAFETY HOLD: {reason}')
        self._safety_state = 'HOLD'
        self._safety_reason = reason

    def request_land(self, reason: str = '') -> None:
        """请求分级安全响应中的 LAND（交接给 PX4 AUTO_LAND）。"""
        if self._killed or self._landing:
            return
        self._safety_state = 'LAND'
        self._safety_reason = reason
        self.get_logger().error(f'[{self.drone_id}] SAFETY LAND: {reason}')
        self.land()

    def _fence_velocity(self, v_sp) -> np.ndarray:
        """软围栏（世界系 NED）——纯逻辑见 `safety_logic.fence_velocity`。"""
        alt_floor = self.safety_alt_min
        if (self.sensor_constraints_enable and self.sensor_use_est_limits
                and math.isfinite(self._est_hagl_min)):
            alt_floor = max(alt_floor, self._est_hagl_min)   # 估计器最小离地约束
        return fence_velocity(v_sp, self.pos_world,
                              geofence_xy=self.safety_geofence_xy,
                              pullback_k=self.safety_pullback_k,
                              pullback_speed=self.safety_pullback_speed,
                              alt_floor=alt_floor,
                              geofence_alt=self.safety_geofence_alt)

    @staticmethod
    def _scale01(x: float, soft: float, hard: float) -> float:
        """软/硬限之间的线性缩放——纯逻辑见 `safety_logic.scale01`。"""
        return scale01(x, soft, hard)

    def _attitude_govern(self, v: np.ndarray):
        """姿态/角速率约束（安全滤波）——纯逻辑见 `safety_logic.attitude_govern`。"""
        return attitude_govern(v, self._last_vsp,
                               roll=self._roll, pitch=self._pitch,
                               ang_rate=self._ang_rate,
                               tilt_soft=self.tilt_soft, tilt_hard=self.tilt_hard,
                               rate_soft=self.rate_soft, rate_hard=self.rate_hard,
                               accel_h_max=self.accel_h_max, hz=self.hz,
                               enable=self.attitude_constraint_enable,
                               att_ok=self._att_ok)

    def _sensor_health_reasons(self, now: float) -> list:
        """基于 PX4 已有 EKF 字段的健康/一致性约束——纯逻辑见
        `safety_logic.sensor_health_reasons`。"""
        return sensor_health_reasons(
            watchdog_enable=self.sensor_watchdog_enable,
            reset_counters=self._reset_counters,
            lpos_valid=self._lpos_valid, dead_reckoning=self._dead_reckoning,
            heading_good=self._heading_good,
            heading_check=self.sensor_watchdog_heading,
            eph=self._eph, epv=self._epv,
            eph_max=self.sensor_eph_max, epv_max=self.sensor_epv_max,
            reset_seen_t=self._reset_seen_t, now=now,
            reset_hold_s=self.sensor_reset_hold_s)

    def _safety_update(self, now: float) -> None:
        """分级安全响应：OK → (HOLD | PULLBACK) → LAND → KILL。

        决策纯逻辑见 `safety_logic.safety_decision`；本方法只负责读取状态、
        应用决策并打日志/发指令。HOLD/PULLBACK 只覆盖速度指令（见
        publish_velocity），任务逻辑仍运行以继续广播状态；LAND/KILL 则直接
        终止 offboard。
        """
        alt_floor = self.safety_alt_min
        if (self.sensor_constraints_enable and self.sensor_use_est_limits
                and math.isfinite(self._est_hagl_min)):
            alt_floor = max(alt_floor, self._est_hagl_min)
        stale = (now - self._last_pos_t) > self.safety_state_timeout
        stale_reason = (f'pos_stale={now - self._last_pos_t:.1f}s' if stale else '')
        health = (self._sensor_health_reasons(now)
                  if self.sensor_constraints_enable else [])
        d = safety_decision(
            now, self._safety_state, self._safety_reason,
            self._bad_since, self._hold_since,
            armed=self._armed, killed=self._killed, landing=self._landing,
            safety_lock=self.safety_lock, att_ok=self._att_ok,
            roll=self._roll, pitch=self._pitch,
            safety_tilt_max=self.safety_tilt_max,
            safety_kill_hold_s=self.safety_kill_hold_s,
            safety_auto_kill=self.safety_auto_kill,
            safety_hold_escalate=self.safety_hold_escalate,
            safety_hold_timeout=self.safety_hold_timeout,
            pos_world=self.pos_world,
            safety_geofence_xy=self.safety_geofence_xy,
            safety_pullback_clear=self.safety_pullback_clear,
            alt_floor=alt_floor, safety_geofence_alt=self.safety_geofence_alt,
            safety_pullback_enable=self.safety_pullback_enable,
            stale_reason=stale_reason, health_reasons=health,
            sensor_constraints_enable=self.sensor_constraints_enable)
        self._safety_state = d.state
        self._safety_reason = d.reason
        self._bad_since = d.bad_since
        self._hold_since = d.hold_since
        if d.action == 'kill':
            self.kill(d.reason)
            return
        if d.action == 'land':
            self.request_land(d.reason)
            return
        if d.event == 'recover':
            self.get_logger().warn(f'[{self.drone_id}] SAFETY 恢复 → OK')
        elif d.event == 'pullback':
            self.get_logger().warn(f'[{self.drone_id}] SAFETY PULLBACK: {d.reason}')
        elif d.event == 'hold':
            self.get_logger().warn(f'[{self.drone_id}] SAFETY HOLD: {d.reason}')
        elif d.event == 'hold_critical':
            self.get_logger().error(
                f'[{self.drone_id}] SAFETY 临界异常(未自动kill) → HOLD: {d.reason}')

    # ------------------------------------------------------------------ 发布
    def broadcast_offboard_mode(self):
        m = OffboardControlMode()
        m.position = False
        m.velocity = True
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_mode.publish(m)

    def publish_velocity(self, vel_ned, yaw=0.0, acc_ff=None):
        """速度设定点（世界 NED）。acc_ff 非 None 时作为 PX4 加速度前馈
        （velocity 模式下 PositionControl 会把 acceleration 直接叠加，见 PX4 源码）。

        分级安全响应：HOLD 覆盖为原地悬停；PULLBACK 在任务速度上做软围栏限制。
        sp_rate_limit > 0 时只对**正常态**速度指令做变化率限幅（安全指令不被拖延）。
        """
        if self._safety_state == 'HOLD':
            vel_ned = [0.0, 0.0, 0.0]
        elif self._safety_state == 'PULLBACK':
            vel_ned = self._fence_velocity(vel_ned)
        # 速率限幅只在正常态生效：安全指令（HOLD/PULLBACK）不得被平滑拖慢
        if self.sp_rate_limit > 0.0 and self._safety_state == 'OK':
            v = np.asarray(vel_ned, float)
            max_dv = self.sp_rate_limit / max(self.hz, 1e-6)
            if self._last_vsp is not None:
                dv = v - self._last_vsp
                n = float(np.linalg.norm(dv))
                if n > max_dv and n > 1e-9:
                    v = self._last_vsp + dv * (max_dv / n)
            vel_ned = v
        # 估计器限值约束：水平/垂直速度不超过 PX4 EKF 给出的限值
        if self.sensor_constraints_enable and self.sensor_use_est_limits:
            vel_ned = clip_to_estimator_limits(vel_ned, self._est_vxy_max,
                                               self._est_vz_max)
        # 姿态/角速率约束（安全滤波）：只在正常态生效，不改安全指令
        if self._safety_state == 'OK':
            vel_ned, self._att_gov = self._attitude_govern(np.asarray(vel_ned, float))
        else:
            self._att_gov = 1.0
        self._last_vsp = np.asarray(vel_ned, float).copy()
        m = TrajectorySetpoint()
        m.position = [float('nan')] * 3
        m.velocity = [float(v) for v in vel_ned]
        if acc_ff is None:
            m.acceleration = [float('nan')] * 3
        else:
            m.acceleration = [float(a) for a in acc_ff]
        m.yaw = float(yaw)
        m.yawspeed = float('nan')
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_sp.publish(m)

    def publish_position(self, pos_ned, vel_ff=(0.0, 0.0, 0.0), yaw=0.0):
        m = TrajectorySetpoint()
        m.position = [float(p) for p in pos_ned]
        m.velocity = [float(v) for v in vel_ff]
        m.acceleration = [float('nan')] * 3
        m.yaw = float(yaw)
        m.yawspeed = float('nan')
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_sp.publish(m)

    def send_command(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = command
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = self.drone_id + 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_cmd.publish(m)

    def arm_and_offboard(self):
        """重试 ARM + OFFBOARD，直到确认。auto_arm=False 时只等待外部解锁。"""
        if self.auto_arm:
            if self._nav_state != 14:
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if not self._armed:
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        if self._nav_state == 14 and self._armed:
            if not self._offboard_confirmed:
                self._offboard_confirmed = True
                self.get_logger().info(f'[{self.drone_id}] OFFBOARD + ARMED confirmed')

    # ------------------------------------------------------------------ 循环
    def land(self):
        """发出 AUTO_LAND（VEHICLE_CMD_NAV_LAND）并停发 offboard setpoint。"""
        if self._landing:
            return
        self._landing = True
        self._land_cmd_count = 0
        self.get_logger().warn(
            f'[{self.drone_id}] LAND：发出着陆指令 @世界={self.pos_world.round(2)}，停止 offboard')

    def _tick(self):
        if self._killed:
            return
        if self._landing:
            # 重复发几次确保 commander 收到；之后只保持节点存活
            if self._land_cmd_count < 5:
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._land_cmd_count += 1
            return
        self.broadcast_offboard_mode()
        self._tick_count += 1
        # 每 1s 重试一次 ARM/OFFBOARD
        if not self._offboard_confirmed and (self._tick_count % int(self.hz) == 1):
            self.arm_and_offboard()
        now = self.get_clock().now().nanoseconds * 1e-9
        # 先做安全评估：HOLD/PULLBACK 会在 publish_velocity 中覆盖任务指令；
        # LAND/KILL 直接终止 offboard。
        self._safety_update(now)
        if self._killed or self._landing:
            return
        self.control()

    def control(self):
        """子类实现。"""
        raise NotImplementedError

    # ------------------------------------------------------------------ 工具
    @property
    def pos(self):
        return self._pos.copy()

    @property
    def pos_world(self):
        """世界系 NED 位置（= 本地位置 + 本机原点偏移）。"""
        return self._pos + self.world_offset

    @property
    def vel(self):
        return self._vel.copy()

    @property
    def yaw(self):
        return self._yaw

    @property
    def alt(self):
        return float(-self._pos[2])

    @property
    def pos_sigma_h(self) -> float:
        """水平位置估计 σ（来自 EKF 的 eph；未启用时返回 0）。"""
        return self._eph if self.sensor_use_ekf_sigma else 0.0

    @property
    def pos_sigma_v(self) -> float:
        """垂直位置估计 σ（来自 EKF 的 epv；未启用时返回 0）。"""
        return self._epv if self.sensor_use_ekf_sigma else 0.0

    def hover_velocity(self, target_pos_world, target_alt_world, kp_xy=1.0, max_speed=1.0,
                       kp_z=1.0, max_climb=0.8):
        """世界系目标点 → 速度 setpoint（XY 限范数，Z 限幅）。轴与世界系对齐，可直接下发。"""
        pw = self.pos_world
        err = np.array([target_pos_world[0] - pw[0],
                        target_pos_world[1] - pw[1],
                        target_alt_world - (-pw[2])])
        v = np.array([kp_xy * err[0], kp_xy * err[1], -kp_z * err[2]])
        n = float(np.linalg.norm(v[:2]))
        if n > max_speed:
            v[:2] *= max_speed / n
        v[2] = float(np.clip(v[2], -max_climb, max_climb))
        return v

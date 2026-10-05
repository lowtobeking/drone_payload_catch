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

    def _on_att(self, msg):
        self._yaw = yaw_from_quat(msg.q)
        self._roll, self._pitch = roll_pitch_from_quat(msg.q)
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

    def _pullback_velocity(self) -> np.ndarray:
        """越界回拉速度（世界系 NED）：把位置软拉回几何围栏内。"""
        pw = self.pos_world
        gx = self.safety_geofence_xy
        k = self.safety_pullback_k
        vmax = self.safety_pullback_speed
        tx = min(max(float(pw[0]), -gx), gx)
        ty = min(max(float(pw[1]), -gx), gx)
        vx = k * (tx - float(pw[0]))
        vy = k * (ty - float(pw[1]))
        n = math.hypot(vx, vy)
        if n > vmax and n > 1e-9:
            vx, vy = vx * vmax / n, vy * vmax / n
        alt = -float(pw[2])
        want_alt = min(max(alt, self.safety_alt_min), self.safety_geofence_alt)
        vz = float(np.clip(-k * (want_alt - alt), -vmax, vmax))
        return np.array([vx, vy, vz])

    def _safety_update(self, now: float) -> None:
        """分级安全响应：OK → (HOLD | PULLBACK) → LAND → KILL。

        检测：姿态超限(临界) / 位置越界(回拉) / 位置状态超时(悬停)。
        临界异常持续 kill_hold_s：safety_auto_kill=True 则飞行终止，否则按
        safety_hold_escalate 升级（none=保持 HOLD，land=降落）。

        HOLD/PULLBACK 只覆盖速度指令（见 publish_velocity），任务逻辑仍运行以
        继续广播状态；LAND/KILL 则直接终止 offboard。
        """
        if not self.safety_lock or self._killed or self._landing:
            return
        if not self._armed:
            if self._safety_state != 'OK':
                self._safety_state = 'OK'
                self._safety_reason = ''
            self._bad_since = None
            self._hold_since = None
            return

        reasons = []
        critical = False
        if self._att_ok and (abs(self._roll) > self.safety_tilt_max
                             or abs(self._pitch) > self.safety_tilt_max):
            reasons.append(f'tilt(r={math.degrees(self._roll):.0f},'
                           f'p={math.degrees(self._pitch):.0f})')
            critical = True
        pw = self.pos_world
        # 滞环：已处于 PULLBACK 时，需回到围栏内 safety_pullback_clear 才恢复
        _clr = self.safety_pullback_clear if self._safety_state == 'PULLBACK' else 0.0
        out_xy = (abs(float(pw[0])) > self.safety_geofence_xy - _clr
                  or abs(float(pw[1])) > self.safety_geofence_xy - _clr)
        alt = -float(pw[2])
        out_hi = alt > self.safety_geofence_alt - _clr
        out_lo = alt < self.safety_alt_min + _clr
        if out_xy or out_hi or out_lo:
            reasons.append(f'geofence(xy={pw[:2].round(1)},alt={alt:.1f})')
        stale = (now - self._last_pos_t) > self.safety_state_timeout
        if stale:
            reasons.append(f'pos_stale={now - self._last_pos_t:.1f}s')

        if not reasons:
            self._bad_since = None
            self._hold_since = None
            if self._safety_state != 'OK':
                self.get_logger().warn(f'[{self.drone_id}] SAFETY 恢复 → OK')
            self._safety_state = 'OK'
            self._safety_reason = ''
            return

        msg = '; '.join(reasons)

        if critical:
            # 临界异常（姿态）：持续 kill_hold_s 后按策略终止/升级
            if self._bad_since is None:
                self._bad_since = now
            if (now - self._bad_since) >= self.safety_kill_hold_s:
                self._bad_since = now        # 避免刷屏
                if self.safety_auto_kill:
                    self.kill(msg)
                    return
                if self.safety_hold_escalate == 'land':
                    self.request_land(msg)
                    return
                self._safety_state = 'HOLD'
                self._safety_reason = msg
                self.get_logger().error(
                    f'[{self.drone_id}] SAFETY 临界异常(未自动kill) → HOLD: {msg}')
            else:
                self._safety_state = 'HOLD'
                self._safety_reason = msg
            return

        # 可恢复异常：越界 → 回拉；仅状态超时 → 悬停
        self._bad_since = None
        if (out_xy or out_hi or out_lo) and self.safety_pullback_enable and not stale:
            if self._safety_state != 'PULLBACK':
                self.get_logger().warn(f'[{self.drone_id}] SAFETY PULLBACK: {msg}')
            self._safety_state = 'PULLBACK'
            self._safety_reason = msg
            self._hold_since = None
            return
        if self._hold_since is None:
            self._hold_since = now
            self.get_logger().warn(f'[{self.drone_id}] SAFETY HOLD: {msg}')
        if (self.safety_hold_escalate == 'land'
                and (now - self._hold_since) >= self.safety_hold_timeout):
            self.request_land(msg)
            return
        self._safety_state = 'HOLD'
        self._safety_reason = msg

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

        分级安全响应：HOLD 覆盖为原地悬停；PULLBACK 覆盖为越界回拉速度。
        sp_rate_limit > 0 时对速度指令做变化率限幅（平滑、降姿态激励）。
        """
        if self._safety_state == 'HOLD':
            vel_ned = [0.0, 0.0, 0.0]
        elif self._safety_state == 'PULLBACK':
            vel_ned = self._pullback_velocity()
        if self.sp_rate_limit > 0.0:
            v = np.asarray(vel_ned, float)
            max_dv = self.sp_rate_limit / max(self.hz, 1e-6)
            if self._last_vsp is not None:
                dv = v - self._last_vsp
                n = float(np.linalg.norm(dv))
                if n > max_dv and n > 1e-9:
                    v = self._last_vsp + dv * (max_dv / n)
            self._last_vsp = v.copy()
            vel_ned = v
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

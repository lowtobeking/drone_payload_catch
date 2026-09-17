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
        self.drone_id = int(self.get_parameter('drone_id').value)
        self.hz = float(self.get_parameter('control_hz').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.world_offset = np.asarray(self.get_parameter('world_offset').value,
                                       float).reshape(3)
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
        self._att_ok = True

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

    def publish_velocity(self, vel_ned, yaw=0.0):
        m = TrajectorySetpoint()
        m.position = [float('nan')] * 3
        m.velocity = [float(v) for v in vel_ned]
        m.acceleration = [float('nan')] * 3
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
        self.get_logger().warn(f'[{self.drone_id}] LAND：发出着陆指令，停止 offboard')

    def _tick(self):
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

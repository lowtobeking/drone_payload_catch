#!/usr/bin/env python3
"""relnav_node.py —— 相对定位驱动节点（真机 A 机状态源）。

把 A 的位置/速度发布到 **/drone_a/state**（`[t, pA_world_NED(3), vA_world_NED(3)]`），
与现有 `a_node` 的接口完全一致 → b_node 无需改动（把 rel_* 噪声参数置 0 即可）。

三种 source：
  · rtk  —— 真机：订阅 A、B 两机 RTK（大地坐标或本地 NED）+ B 姿态，做**杆臂补偿**后相对化；
  · px4  —— SITL/联调：直接读两机 PX4 local_position（drone0=A / drone1=B）+ world_offset；
  · sim  —— 透传外部真值（如从 gz 桥接）。

RTK 消息类型：
  · navsatfix : sensor_msgs/NavSatFix（lat/lon/alt + status）；
  · array     : std_msgs/Float64MultiArray，`use_geodetic=true` 时为 [lat,lon,alt]，
                否则为已在该公共 NED 系下的 [n,e,d]。

用法（真机）：
  ros2 run payload_catch relnav_node --ros-args \
    -p source:=rtk -p a_rtk_topic:=/rtk/a -p b_rtk_topic:=/rtk/b \
    -p a_lever:="[0.0,0.0,0.15]" -p b_lever:="[0.0,0.0,-0.21]"
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray

from .relnav import (geodetic_to_ned, rpy_to_R, compensate_lever_arm,
                     a_state_for_b, VelEstimator)


def quat_to_rpy(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def qos_in():
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=5,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class RelNavNode(Node):
    def __init__(self):
        super().__init__('relnav_node')
        self.declare_parameter('source', 'rtk')            # rtk | px4 | sim
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('out_topic', '/drone_a/state')
        # RTK
        self.declare_parameter('a_rtk_topic', '/rtk/a')
        self.declare_parameter('b_rtk_topic', '/rtk/b')
        self.declare_parameter('rtk_type', 'navsatfix')    # navsatfix | array
        self.declare_parameter('use_geodetic', True)       # array 时是否 [lat,lon,alt]
        self.declare_parameter('origin_lat', 0.0)          # 公共 NED 原点（默认取首个 B fix）
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('origin_alt', 0.0)
        self.declare_parameter('a_lever', [0.0, 0.0, 0.0])  # A 天线→参考点（机体系）
        self.declare_parameter('b_lever', [0.0, 0.0, 0.0])  # B 天线→末端（机体系）
        # px4 模式（联调）
        self.declare_parameter('a_lpos_topic', '/fmu/out/vehicle_local_position')
        self.declare_parameter('b_lpos_topic', '/px4_1/fmu/out/vehicle_local_position')
        self.declare_parameter('b_att_topic', '/px4_1/fmu/out/vehicle_attitude')
        self.declare_parameter('a_world_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('b_world_offset', [0.0, 0.0, 0.0])
        # 注入（可选，用于 sim 对照）
        self.declare_parameter('noise_sigma', 0.0)
        self.declare_parameter('latency_s', 0.0)
        self.declare_parameter('seed', 0)

        self.source = str(self.get_parameter('source').value).lower()
        self.rtk_type = str(self.get_parameter('rtk_type').value).lower()
        self.use_geo = bool(self.get_parameter('use_geodetic').value)
        self.a_lever = np.asarray(self.get_parameter('a_lever').value, float).reshape(3)
        self.b_lever = np.asarray(self.get_parameter('b_lever').value, float).reshape(3)
        self.a_off = np.asarray(self.get_parameter('a_world_offset').value, float).reshape(3)
        self.b_off = np.asarray(self.get_parameter('b_world_offset').value, float).reshape(3)
        self.noise_sigma = float(self.get_parameter('noise_sigma').value)
        self.latency = float(self.get_parameter('latency_s').value)
        self.rng = np.random.default_rng(int(self.get_parameter('seed').value))

        self._origin = None      # (lat,lon,alt)
        o_lat = float(self.get_parameter('origin_lat').value)
        o_lon = float(self.get_parameter('origin_lon').value)
        o_alt = float(self.get_parameter('origin_alt').value)
        if abs(o_lat) + abs(o_lon) + abs(o_alt) > 1e-9:
            self._origin = (o_lat, o_lon, o_alt)
        self._a_fix = None       # (t, lat, lon, alt) 或 (t, n, e, d)
        self._b_fix = None
        self._a_prev_nav = None  # (t, ned)
        self._b_att = np.zeros(3)     # rpy
        self._b_px4_world = np.zeros(3)
        self._a_px4_world = np.zeros(3)
        self._vel = VelEstimator()
        self._last_pub = None
        self._lat_buf = []       # 延迟缓冲 [(t, payload)]

        from sensor_msgs.msg import NavSatFix
        # 订阅
        if self.source == 'rtk':
            if self.rtk_type == 'navsatfix':
                self.create_subscription(NavSatFix, str(self.get_parameter('a_rtk_topic').value),
                                         self._on_a_gps, 10)
                self.create_subscription(NavSatFix, str(self.get_parameter('b_rtk_topic').value),
                                         self._on_b_gps, 10)
            else:
                self.create_subscription(Float64MultiArray,
                                         str(self.get_parameter('a_rtk_topic').value),
                                         self._on_a_arr, 10)
                self.create_subscription(Float64MultiArray,
                                         str(self.get_parameter('b_rtk_topic').value),
                                         self._on_b_arr, 10)
        elif self.source == 'px4':
            from px4_msgs.msg import VehicleLocalPosition
            self.create_subscription(VehicleLocalPosition,
                                     str(self.get_parameter('a_lpos_topic').value),
                                     lambda m: self._on_px4(m, 'a'), qos_in())
            self.create_subscription(VehicleLocalPosition,
                                     str(self.get_parameter('b_lpos_topic').value),
                                     lambda m: self._on_px4(m, 'b'), qos_in())
            try:
                from px4_msgs.msg import VehicleAttitude
                self.create_subscription(VehicleAttitude,
                                         str(self.get_parameter('b_att_topic').value),
                                         self._on_b_att, qos_in())
            except Exception:  # noqa: BLE001
                pass

        self.pub = self.create_publisher(Float64MultiArray,
                                         str(self.get_parameter('out_topic').value), 10)
        self.timer = self.create_timer(1.0 / float(self.get_parameter('publish_hz').value),
                                       self._tick)
        self.get_logger().info(f'[relnav] source={self.source} rtk_type={self.rtk_type} '
                               f'out={self.get_parameter("out_topic").value}')

    # --------------------------------------------------------------- 回调
    def _stamp(self, msg=None):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_a_gps(self, msg):
        if msg.status.status < 0:
            return
        self._a_fix = (self._stamp(), float(msg.latitude), float(msg.longitude), float(msg.altitude))
        if self._origin is None:
            self._origin = (float(msg.latitude), float(msg.longitude), float(msg.altitude))

    def _on_b_gps(self, msg):
        if msg.status.status < 0:
            return
        self._b_fix = (self._stamp(), float(msg.latitude), float(msg.longitude), float(msg.altitude))
        if self._origin is None:
            self._origin = (float(msg.latitude), float(msg.longitude), float(msg.altitude))

    def _on_a_arr(self, msg):
        d = list(msg.data)
        if len(d) < 3:
            return
        if self.use_geo:
            self._a_fix = (self._stamp(), d[0], d[1], d[2])
            if self._origin is None:
                self._origin = (d[0], d[1], d[2])
        else:
            self._a_fix = (self._stamp(), d[0], d[1], d[2])   # 已是 NED

    def _on_b_arr(self, msg):
        d = list(msg.data)
        if len(d) < 3:
            return
        if self.use_geo:
            self._b_fix = (self._stamp(), d[0], d[1], d[2])
            if self._origin is None:
                self._origin = (d[0], d[1], d[2])
        else:
            self._b_fix = (self._stamp(), d[0], d[1], d[2])

    def _on_b_att(self, msg):
        self._b_att = np.array(quat_to_rpy(msg.q), float)

    def _on_px4(self, msg, who):
        p = np.array([msg.x, msg.y, msg.z], float)
        if who == 'a':
            self._a_px4_world = self.a_off + p
        else:
            self._b_px4_world = self.b_off + p

    # ----------------------------------------------------------- 核心计算
    def _fix_to_ned(self, fix):
        t, x, y, z = fix
        if self.use_geo and self.source == 'rtk':
            lat0, lon0, alt0 = self._origin
            return t, geodetic_to_ned(x, y, z, lat0, lon0, alt0)
        return t, np.array([x, y, z], float)

    def _tick(self):
        if self.source == 'px4':
            p_a_ref = self._a_px4_world + \
                rpy_to_R(0.0, 0.0, 0.0) @ np.zeros(3)      # px4 模式杆臂已在 offset 内
            p_b_ref = self._b_px4_world
            p_a_world = a_state_for_b(p_a_ref, p_b_ref, self._b_px4_world)
            v = self._vel.update(self._stamp(), p_a_world)
            self._pub(p_a_world, v)
            return
        if self._a_fix is None or self._b_fix is None:
            return
        t_a, a_ned = self._fix_to_ned(self._a_fix)
        t_b, b_ned = self._fix_to_ned(self._b_fix)
        # 杆臂补偿（B 用自身姿态；A 姿态若未提供，按水平处理）
        a_ref = compensate_lever_arm(a_ned, self.a_lever, 0.0, 0.0, 0.0)
        b_ref = compensate_lever_arm(b_ned, self.b_lever, *self._b_att)
        p_a_world = a_state_for_b(a_ref, b_ref, self._b_px4_world)
        if self.noise_sigma > 0:
            p_a_world = p_a_world + self.rng.normal(0.0, self.noise_sigma, 3)
        v = self._vel.update(self._stamp(), p_a_world)
        self._pub(p_a_world, v)

    def _pub(self, p, v):
        now = self._stamp()
        payload = (now, p, v)
        if self.latency > 0:
            self._lat_buf.append(payload)
            cut = now - self.latency
            while len(self._lat_buf) > 1 and self._lat_buf[0][0] < cut:
                payload = self._lat_buf.pop(0)
        t, p, v = payload
        m = Float64MultiArray()
        m.data = [float(t), float(p[0]), float(p[1]), float(p[2]),
                  float(v[0]), float(v[1]), float(v[2])]
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = RelNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

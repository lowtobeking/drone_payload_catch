#!/usr/bin/env python3
"""A（投送方）节点：起飞 → 悬停在释放点。

M6 垂直堆叠模式下，A 额外把自身【世界系 NED】位姿广播到 `/drone_a/state`，
供 B 做**相对定位**（mesh/UWB 的仿真替身：B 订阅后加噪声/延迟）。

话题格式 Float64MultiArray:
    [t, N, E, D, vN, vE, vD]   （世界系 NED）
"""
from __future__ import annotations

import numpy as np
import rclpy
from std_msgs.msg import Bool, Float64, Float64MultiArray

from .px4_iface import Px4Drone
from .stack_drop import _horiz_drift


class ANode(Px4Drone):
    def __init__(self):
        super().__init__('a_node', default_id=0)
        self.declare_parameter('hover_world', [0.0, 0.0, -2.5])   # 世界系 NED
        self.declare_parameter('publish_state', True)
        self.declare_parameter('auto_land', False)          # 捕获后自动降落
        self.declare_parameter('land_after_catch_s', 6.0)   # 捕获后再悬停多久开始着陆流程
        self.declare_parameter('land_xy', [-4.0, 0.0])      # 世界系 NED 着陆点 x,y（与 B 分开）
        self.declare_parameter('land_xy_tol', 0.25)         # 到达着陆点的水平容差
        # ── M6-moving：编队同速巡航（收到 /formation/start 后以 formation_vel 直线飞行）──
        self.declare_parameter('formation_vel', [0.0, 0.0, 0.0])   # 世界系 NED 水平速度
        self.declare_parameter('formation_topic', '/formation/start')
        self.declare_parameter('formation_abort_topic', '/formation/abort')
        # ── 协同释放握手：'direct'(B 直接决定释放) | 'handshake'(B 报就绪→A 作释放权威) ──
        self.declare_parameter('coord_mode', 'direct')
        self.declare_parameter('release_lead', 0.20)     # A 提前发布释放时刻（补命令延迟）
        self.declare_parameter('ready_timeout', 0.5)     # B 就绪消息的新鲜度上限 (s)
        self.declare_parameter('settle_xy_tol', 0.15)    # A “已就位”水平容差 (m)
        self.declare_parameter('settle_alt_tol', 0.15)   # A “已就位”高度容差 (m)
        self.declare_parameter('settle_vtol', 0.20)      # A “已就位”速度上限 (m/s)
        self.declare_parameter('release_xy_tol', 0.12)   # 释放前独立性门限：B 报的对正误差上限
        self.declare_parameter('release_spd_tol', 0.15)  # 释放前独立性门限：B 报的自身速度上限
        # 释放前落点余量闸：rel_xy + |风漂移| + kσ ≤ eff_r − min_margin
        self.declare_parameter('funnel_eff_radius', 0.25)
        self.declare_parameter('min_release_margin', 0.05)
        self.declare_parameter('release_sigma_k', 1.0)
        self.declare_parameter('release_sigma', 0.03)
        # ── 意图升级：A 广播【预测落点】= 目标点 + 风漂移，B 直接对齐落点 ──
        self.declare_parameter('wind_est', [0.0, 0.0, 0.0])   # A 的风估计 (NED)；可由 PX4 EKF 提供
        self.declare_parameter('payload_drag_k', 0.0)          # 载荷线性阻力 1/s（与 config 一致）
        self.declare_parameter('payload_fall_t', 0.45)         # 标称下落时间 (s)，用于预测漂移
        self.declare_parameter('use_px4_wind', False)          # 用 PX4 EKF 风估计（/fmu/out/wind）代替 wind_est 参数
        self.declare_parameter('wind_topic', '/fmu/out/wind')
        # ── 安全层：释放后 A 定向清场（离开 B 的空域）──
        self.declare_parameter('clear_offset', [2.0, 0.0, 0.0])  # NED 水平清场偏移（从悬停点）
        self.declare_parameter('clear_enable', True)
        self.hover = np.asarray(self.get_parameter('hover_world').value, float).reshape(3)
        self.publish_state = bool(self.get_parameter('publish_state').value)
        self.auto_land = bool(self.get_parameter('auto_land').value)
        self.land_after_catch_s = float(self.get_parameter('land_after_catch_s').value)
        self.land_xy = np.asarray(self.get_parameter('land_xy').value, float).reshape(2)
        self.land_xy_tol = float(self.get_parameter('land_xy_tol').value)
        self.formation_vel = np.asarray(self.get_parameter('formation_vel').value,
                                        float).reshape(3)
        self.formation_vel[2] = 0.0
        self._formation = float(np.linalg.norm(self.formation_vel[:2])) > 1e-6
        self._form_t0 = None
        self._form_p0 = None
        self._caught = False
        self._caught_t = None
        self._aborted = False
        self.coord_mode = str(self.get_parameter('coord_mode').value)
        self.release_lead = float(self.get_parameter('release_lead').value)
        self.ready_timeout = float(self.get_parameter('ready_timeout').value)
        self.settle_xy_tol = float(self.get_parameter('settle_xy_tol').value)
        self.settle_alt_tol = float(self.get_parameter('settle_alt_tol').value)
        self.settle_vtol = float(self.get_parameter('settle_vtol').value)
        self._ready = None
        self._ready_t = 0.0
        self._released = False
        self.release_xy_tol = float(self.get_parameter('release_xy_tol').value)
        self.release_spd_tol = float(self.get_parameter('release_spd_tol').value)
        self.funnel_eff_radius = float(self.get_parameter('funnel_eff_radius').value)
        self.min_release_margin = float(self.get_parameter('min_release_margin').value)
        self.release_sigma_k = float(self.get_parameter('release_sigma_k').value)
        self.release_sigma = float(self.get_parameter('release_sigma').value)
        self._margin_warned = False
        self.wind_est = np.asarray(self.get_parameter('wind_est').value, float).reshape(3)
        self.payload_drag_k = float(self.get_parameter('payload_drag_k').value)
        self.payload_fall_t = float(self.get_parameter('payload_fall_t').value)
        self.clear_offset = np.asarray(self.get_parameter('clear_offset').value,
                                       float).reshape(3)
        self.clear_enable = bool(self.get_parameter('clear_enable').value)
        self._payload_released = False
        self.pub_state = self.create_publisher(Float64MultiArray, '/drone_a/state', 10)
        self.pub_intent = self.create_publisher(Float64MultiArray, '/drone_a/intent', 10)
        self.pub_release_cmd = self.create_publisher(Float64MultiArray, '/drone_a/release_cmd', 10)
        self.pub_release_at = self.create_publisher(Float64, '/payload/release_at', 10)
        self.create_subscription(Bool, '/payload/caught', self._on_caught, 10)
        self.create_subscription(Bool, '/payload/released', self._on_released, 10)
        self.pub_pong = self.create_publisher(Float64MultiArray, '/coord/pong', 10)
        self.create_subscription(Float64MultiArray, '/coord/ping', self._on_ping, 10)
        self.use_px4_wind = bool(self.get_parameter('use_px4_wind').value)
        if self.use_px4_wind:
            try:
                from px4_msgs.msg import Wind as Px4Wind
                self.create_subscription(Px4Wind, str(self.get_parameter('wind_topic').value),
                                         self._on_px4_wind, 10)
                self.get_logger().warn('A: use_px4_wind=True（用 PX4 EKF 风估计做预测落点）')
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'A: px4_msgs Wind 不可用（忽略）: {e}')
        if self.coord_mode == 'handshake':
            self.create_subscription(Float64MultiArray, '/drone_b/ready', self._on_b_ready, 10)
            self.get_logger().warn('A: coord_mode=handshake（B 报就绪 → A 作释放权威）')
        if self._formation:
            self.create_subscription(Bool, str(self.get_parameter('formation_topic').value),
                                     self._on_formation_start, 10)
            self.create_subscription(Bool, str(self.get_parameter('formation_abort_topic').value),
                                     self._on_abort, 10)
            self.get_logger().warn(f'A: 编队模式 formation_vel={self.formation_vel}')

    def _on_formation_start(self, msg):
        if msg.data and self._form_t0 is None:
            self._form_t0 = self.get_clock().now().nanoseconds * 1e-9
            self._form_p0 = self.pos_world[:2].copy()
            self.get_logger().warn(
                f'A: /formation/start → 编队巡航 vel={self.formation_vel[:2]} from {self._form_p0.round(2)}')

    def _on_px4_wind(self, msg):
        """PX4 EKF 风估计（NED 水平）→ wind_est，用于意图里的预测落点。"""
        self.wind_est[0] = float(msg.windspeed_north)
        self.wind_est[1] = float(msg.windspeed_east)

    def _on_released(self, msg):
        if msg.data and not self._payload_released:
            self._payload_released = True
            self.get_logger().warn('A: /payload/released → 开始定向清场（离开 B 空域）')

    def _on_abort(self, msg):
        """编队释放超时中止：A 停止巡航，保留载荷并进入降落流程。"""
        if msg.data and not self._aborted:
            self._aborted = True
            self._caught_t = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().error('A: 收到 /formation/abort → 停止巡航，保留载荷并降落')

    def _on_caught(self, msg):
        if msg.data and not self._caught:
            self._caught = True
            self._caught_t = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().warn('A: 收到 /payload/caught')

    def _on_b_ready(self, msg):
        """B 报就绪：data=[ready(0/1), rel_xy, spd_xy, stamp]。"""
        self._ready = list(msg.data)
        self._ready_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_ping(self, msg):
        """时钟同步：回显 B 的发送时刻 + A 的接收时刻。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        r = Float64MultiArray()
        r.data = [float(msg.data[0]), float(now)]
        self.pub_pong.publish(r)

    def _coord_tick(self, now):
        """广播意图（悬停目标 / 编队移动参考点）；handshake 时作释放权威发释放时刻 + ack。"""
        in_formation = self._formation and self._form_t0 is not None
        if in_formation:
            ref_xy = self._form_p0 + self.formation_vel[:2] * (now - self._form_t0)
            ix, iy = float(ref_xy[0]), float(ref_xy[1])
        else:
            ix, iy = float(self.hover[0]), float(self.hover[1])
        # 预测落点 = 目标点 + 风漂移（载荷从 A 释放、历时 fall_t）
        dx = _horiz_drift(float(self.wind_est[0]), 'linear', self.payload_drag_k, self.payload_fall_t)
        dy = _horiz_drift(float(self.wind_est[1]), 'linear', self.payload_drag_k, self.payload_fall_t)
        m = Float64MultiArray()
        m.data = [now, ix, iy, float(-self.hover[2]),
                  float(self.formation_vel[0]), float(self.formation_vel[1]),
                  ix + dx, iy + dy]
        self.pub_intent.publish(m)
        if self.coord_mode != 'handshake' or self._released:
            return
        # 协议健壮性：B 就绪必须【新鲜】且为真
        if self._ready is None or (now - self._ready_t) > self.ready_timeout:
            return
        if float(self._ready[0]) < 0.5:
            return
        # 释放前一致性/余量门限：B 报告的对正质量需达 A 的独立门限（不只信 B 自己的阈值）
        if len(self._ready) >= 3 and (float(self._ready[1]) > self.release_xy_tol
                                      or float(self._ready[2]) > self.release_spd_tol):
            return
        # 落点余量闸：预测 miss = rel_xy + |风漂移| + kσ，必须 ≤ eff_r − min_margin，否则暂不释放
        drift_mag = float(np.hypot(dx, dy))
        pred_miss = float(self._ready[1]) + drift_mag + self.release_sigma_k * self.release_sigma
        if pred_miss > self.funnel_eff_radius - self.min_release_margin:
            if not self._margin_warned:
                self._margin_warned = True
                self.get_logger().warn(
                    f'A: 落点余量不足 pred_miss={pred_miss:.3f} > '
                    f'{self.funnel_eff_radius - self.min_release_margin:.3f}，暂不释放')
            return
        # A 自身已就位：编队=速度匹配编队速度；悬停=在悬停点且静止（用 A 的精确状态）
        if in_formation:
            settled = float(np.linalg.norm(self.vel[:2] - self.formation_vel[:2])) < self.settle_vtol
        else:
            settled = (float(np.linalg.norm(self.pos_world[:2] - self.hover[:2])) < self.settle_xy_tol
                       and abs(-self.pos_world[2] - (-self.hover[2])) < self.settle_alt_tol
                       and float(np.linalg.norm(self.vel)) < self.settle_vtol)
        if not settled:
            return
        t_rel = now + self.release_lead
        self.pub_release_at.publish(Float64(data=t_rel))
        c = Float64MultiArray()
        c.data = [1.0, t_rel, now]
        self.pub_release_cmd.publish(c)
        self._released = True
        self.get_logger().warn(
            f'A: 收到 B 就绪 → 释放权威发布 release@{t_rel:.2f}s (lead={self.release_lead})')

    def control(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._coord_tick(now)
        # 捕获后：先飞到自己的着陆点（保持高度），到位后再落地
        if self.auto_land and (self._caught or self._aborted) and not self._landing:
            if now - self._caught_t >= self.land_after_catch_s:
                if float(np.linalg.norm(self.pos_world[:2] - self.land_xy)) < self.land_xy_tol:
                    self.land()
                else:
                    v = self.hover_velocity(self.land_xy, -self.hover[2], kp_xy=1.2,
                                            max_speed=1.5, kp_z=1.2, max_climb=1.0)
                    self.publish_velocity(v, yaw=self.yaw)
                if self.publish_state:
                    self._pub_state()
                return
            # 等待降落期间：原地保持（编队模式下不要飞回原点）
            v = self.hover_velocity(self.pos_world[:2], -self.pos_world[2], kp_xy=1.2,
                                    max_speed=1.0, kp_z=1.2, max_climb=1.0)
            self.publish_velocity(v, yaw=self.yaw)
            if self.publish_state:
                self._pub_state()
            return
        # 编队同速巡航：沿 formation_vel 方向直线飞行（位置参考 + 速度前馈）
        if self._formation and self._form_t0 is not None:
            t = now - self._form_t0
            ref_xy = self._form_p0 + self.formation_vel[:2] * t
            v = self.hover_velocity(ref_xy, -self.hover[2], kp_xy=1.2,
                                    max_speed=1.5 * max(1.0, float(np.linalg.norm(self.formation_vel[:2])) * 2.0),
                                    kp_z=1.2, max_climb=1.0)
            v[:2] += self.formation_vel[:2]
            self.publish_velocity(v, yaw=self.yaw)
            if self.publish_state:
                self._pub_state()
            return
        # 世界系目标（A 原点在世界 (0,0,0)，world_offset 默认 0）
        # 安全层：释放后向 clear_offset 方向清场，给 B 让出空域
        tgt_xy = self.hover[:2]
        if self.clear_enable and self._payload_released:
            tgt_xy = self.hover[:2] + self.clear_offset[:2]
        v = self.hover_velocity(tgt_xy, -self.hover[2],
                                kp_xy=1.0, max_speed=1.5, kp_z=1.0, max_climb=1.0)
        self.publish_velocity(v, yaw=self.yaw)
        if self.publish_state:
            self._pub_state()

    def _pub_state(self):
        pw = self.pos_world
        m = Float64MultiArray()
        m.data = [float(self.get_clock().now().nanoseconds * 1e-9),
                  float(pw[0]), float(pw[1]), float(pw[2]),
                  float(self.vel[0]), float(self.vel[1]), float(self.vel[2])]
        self.pub_state.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

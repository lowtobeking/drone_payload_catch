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
from .coord_cert import cert_threshold


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
        self.declare_parameter('commit_hold_s', 0.20)    # 就绪门限需持续多久才提交释放
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
        self.declare_parameter('release_sigma_max', 0.15)  # σ 上限（防重噪声把闸门卡死）
        self.declare_parameter('use_b_sigma', True)   # 用 B 上报的在线 σ 做余量闸
        self.declare_parameter('release_gate_mode', 'heuristic')  # heuristic | certificate
        self.declare_parameter('cert_eps', 0.05)        # 证书保证水平（certificate 模式）
        self.declare_parameter('cert_sigma_track', 0.02)  # 释放后跟踪残差 σ (m)
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
        self.commit_hold_s = float(self.get_parameter('commit_hold_s').value)
        self.ready_timeout = float(self.get_parameter('ready_timeout').value)
        self.settle_xy_tol = float(self.get_parameter('settle_xy_tol').value)
        self.settle_alt_tol = float(self.get_parameter('settle_alt_tol').value)
        self.settle_vtol = float(self.get_parameter('settle_vtol').value)
        self._ready = None
        self._ready_t = 0.0
        self._released = False
        self._released_at = None      # 已提交释放的绝对时刻（lead 窗口内复核用）
        self._commit_t0 = None        # 提交窗口起点
        self.release_xy_tol = float(self.get_parameter('release_xy_tol').value)
        self.release_spd_tol = float(self.get_parameter('release_spd_tol').value)
        self.funnel_eff_radius = float(self.get_parameter('funnel_eff_radius').value)
        self.min_release_margin = float(self.get_parameter('min_release_margin').value)
        self.release_sigma_k = float(self.get_parameter('release_sigma_k').value)
        self.release_sigma = float(self.get_parameter('release_sigma').value)
        self.release_sigma_max = float(self.get_parameter('release_sigma_max').value)
        self.use_b_sigma = bool(self.get_parameter('use_b_sigma').value)
        self._b_sigma = None
        self._b_sigma_rel = None
        self.release_gate_mode = str(self.get_parameter('release_gate_mode').value).lower()
        self.cert_eps = float(self.get_parameter('cert_eps').value)
        self.cert_sigma_track = float(self.get_parameter('cert_sigma_track').value)
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
        self.pub_release_abort = self.create_publisher(Bool, '/payload/release_abort', 10)
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
        """B 报就绪：data=[ready(0/1), rel_xy, spd_xy, stamp, (sigma_abs), (sigma_rel)]。"""
        self._ready = list(msg.data)
        self._ready_t = self.get_clock().now().nanoseconds * 1e-9
        if len(msg.data) >= 5:
            self._b_sigma = float(msg.data[4])
        if len(msg.data) >= 6:
            self._b_sigma_rel = float(msg.data[5])

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
        if self.coord_mode != 'handshake':
            return
        # A 自身就位（独立于 B 的 ready；提交窗口与 lead 窗口复核都用它）
        if in_formation:
            settled = float(np.linalg.norm(self.vel[:2] - self.formation_vel[:2])) < self.settle_vtol
        else:
            settled = (float(np.linalg.norm(self.pos_world[:2] - self.hover[:2])) < self.settle_xy_tol
                       and abs(-self.pos_world[2] - (-self.hover[2])) < self.settle_alt_tol
                       and float(np.linalg.norm(self.vel)) < self.settle_vtol)
        # ── 释放门限评估（提交窗口用）──
        drift_mag = float(np.hypot(dx, dy))
        gate_ok = True
        reason = ''
        sigma = self.release_sigma
        if self._ready is None or (now - self._ready_t) > self.ready_timeout:
            gate_ok, reason = False, 'ready 不新鲜'
        elif float(self._ready[0]) < 0.5:
            gate_ok, reason = False, 'B 未就绪'
        elif len(self._ready) >= 3 and (float(self._ready[1]) > self.release_xy_tol
                                        or float(self._ready[2]) > self.release_spd_tol):
            gate_ok, reason = False, 'B 对正质量超门限'
        else:
            sigma = self.release_sigma
            if self.release_gate_mode == 'certificate':
                # 证书闸：用**相对** σ_m，阈值 T(ε) 由精确 Rice 证书给
                s_rel = (self._b_sigma_rel if self._b_sigma_rel is not None
                         else (self._b_sigma if self._b_sigma is not None else self.release_sigma))
                sigma_m = float(np.sqrt(s_rel ** 2 + self.cert_sigma_track ** 2))
                T = cert_threshold('exact', self.cert_eps, self.funnel_eff_radius,
                                   self.min_release_margin, sigma_m)
                rel_est = float(self._ready[1]) + drift_mag
                if rel_est > T:
                    gate_ok = False
                    reason = (f'证书闸 rel={rel_est:.3f} > T={T:.3f} '
                              f'(σ_m={sigma_m:.3f}, ε={self.cert_eps})')
                    if not self._margin_warned:
                        self._margin_warned = True
                        self.get_logger().warn(f'A: {reason}，暂不释放')
                elif not settled:
                    gate_ok, reason = False, 'A 未就位'
                sigma = sigma_m
            else:
                # 启发式闸：σ 取 A 先验 / B 上报 / A 自身 EKF σ 的较大者，再封顶
                if self.use_b_sigma and self._b_sigma is not None:
                    sigma = max(sigma, self._b_sigma)
                if self.sensor_constraints_enable and self.sensor_use_ekf_sigma:
                    sigma = max(sigma, self.pos_sigma_h)
                sigma = min(sigma, self.release_sigma_max)
                pred_miss = float(self._ready[1]) + drift_mag + self.release_sigma_k * sigma
                if pred_miss > self.funnel_eff_radius - self.min_release_margin:
                    gate_ok = False
                    reason = f'落点余量不足 {pred_miss:.3f}'
                    if not self._margin_warned:
                        self._margin_warned = True
                        self.get_logger().warn(
                            f'A: 落点余量不足 pred_miss={pred_miss:.3f} > '
                            f'{self.funnel_eff_radius - self.min_release_margin:.3f}，暂不释放')
                elif not settled:
                    gate_ok, reason = False, 'A 未就位'

        # ── 已提交：lead 窗口内只复核 A 自身就位（B 已 ack 后会停发 ready）──
        if self._released:
            if (self._released_at is not None and now < self._released_at
                    and not settled):
                self._abort_release(now, 'A 释放前未就位')
            return

        if not gate_ok:
            self._commit_t0 = None
            return
        # ── 提交窗口：需持续 gate_ok 达 commit_hold_s，避免单拍抖动误释放 ──
        if self._commit_t0 is None:
            self._commit_t0 = now
            self.get_logger().warn(
                f'A: 就绪门限通过 → 进入释放提交窗口 ({self.commit_hold_s:.2f}s)')
            return
        if (now - self._commit_t0) < self.commit_hold_s:
            return
        # ── 提交：发布释放时刻 + ack ──
        t_rel = now + self.release_lead
        self._released_at = t_rel
        self.pub_release_at.publish(Float64(data=t_rel))
        c = Float64MultiArray()
        c.data = [1.0, t_rel, now]
        self.pub_release_cmd.publish(c)
        self._released = True
        self._commit_t0 = None
        self.get_logger().warn(
            f'A: 释放权威发布 release@{t_rel:.2f}s '
            f'(lead={self.release_lead}, σ={sigma:.3f})')

    def _abort_release(self, now, reason):
        """在真正释放前撤销已排定的释放（重置状态，可重新尝试）。"""
        self.get_logger().error(f'A: 释放前复核失败 → 取消释放: {reason}')
        self.pub_release_abort.publish(Bool(data=True))
        c = Float64MultiArray()
        c.data = [-1.0, 0.0, now]
        self.pub_release_cmd.publish(c)
        self._released = False
        self._released_at = None
        self._commit_t0 = None

    def control(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._coord_tick(now)
        if self._tick_count % int(self.hz) == 0:
            self.get_logger().info(
                f'A safe={self._safety_state} pos_w={self.pos_world.round(2)} '
                f'vel={self.vel.round(2)} released={self._released}')
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

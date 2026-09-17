#!/usr/bin/env python3
"""B（接收方）节点：起飞 → 待命悬停 → 载荷释放后闭环会合 → 捕获。

流程：
  HOLD  : 悬停在待命点，等 /payload/released
  RENDEZ : 每 replan_dt 用 solve_inflight(载荷状态, B 状态) 重解会合参考；
           速度 setpoint = 参考速度前馈 + 位置 P 纠偏（限幅）
  DONE  : 捕获后悬停
捕获判据：|p_B−p_p| < r_c 且 |v_B−v_p| < v_c（世界系 NED）
"""
from __future__ import annotations

import numpy as np
import rclpy
from std_msgs.msg import Bool, Float64, Float64MultiArray

from .px4_iface import Px4Drone
from .rendezvous import RendezvousPlanner
from .stack_drop import plan_stack_drop, _stack_ref


class BNode(Px4Drone):
    def __init__(self):
        super().__init__('b_node', default_id=1)
        self.declare_parameter('standby_world', [1.2, 0.0, -3.5])
        self.declare_parameter('capture_radius', 0.30)
        self.declare_parameter('capture_rel_speed', 1.50)
        self.declare_parameter('replan_dt', 0.10)
        self.declare_parameter('inflight_tau_max', 2.0)
        self.declare_parameter('catch_alt_min', 0.8)
        self.declare_parameter('catch_alt_max', 2.8)
        self.declare_parameter('b_max_speed', 5.0)
        self.declare_parameter('b_max_accel', 6.0)
        self.declare_parameter('kp_pos', 1.2)
        self.declare_parameter('controller', 'pd')      # pd | mpc
        self.declare_parameter('a_release_world', [0.0, 0.0, -3.0])   # A 的悬停/释放点(世界系)
        self.declare_parameter('start_delay', 18.0)                   # 起飞稳定后开始规划(s)
        self.declare_parameter('plan_tr_max', 3.0)
        # ── M6 垂直堆叠模块 ──
        self.declare_parameter('mode', 'rendezvous')      # rendezvous | stack
        self.declare_parameter('a_state_topic', '/drone_a/state')
        self.declare_parameter('rel_pos_sigma', 0.0)      # mesh 相对定位噪声 (m)
        self.declare_parameter('rel_latency', 0.0)        # mesh 相对定位延迟 (s)
        self.declare_parameter('rel_jitter', 0.0)         # 额外延迟抖动 (s，均匀)
        self.declare_parameter('rel_dropout', 0.0)        # 相对定位丢包率 [0,1]
        self.declare_parameter('rel_bias', 0.0)           # 慢变偏置（随机游走幅度, m）
        self.declare_parameter('rel_seed', 0)             # 噪声种子（可复现）
        self.declare_parameter('est_lpf_alpha', 0.30)     # 相对/载荷估计 EMA 系数(0~1, 1=不滤波)
        self.declare_parameter('payload_meas_sigma', 0.0) # 载荷测量噪声 (m)
        self.declare_parameter('payload_meas_latency', 0.0)
        self.declare_parameter('payload_dropout', 0.0)
        self.declare_parameter('track_payload', True)     # DIVE 时跟踪载荷(闭环)而非 A
        self.declare_parameter('align_xy_tol', 0.12)      # 水平对正阈值 (m)
        self.declare_parameter('align_vel_tol', 0.12)     # 水平速度阈值 (m/s)
        self.declare_parameter('align_alt_tol', 0.20)     # 高度到位阈值 (m)
        self.declare_parameter('align_hold_s', 1.0)       # 稳定保持多久才释放
        self.declare_parameter('approach_alt_tol', 0.15)  # 垂直爬升到位的容差 (m)
        self.declare_parameter('min_ab_gap', 0.80)        # 横移/对正时 B 至少比 A 低多少 (m)
        self.declare_parameter('release_lead', 0.20)      # 提前广播释放时刻
        self.declare_parameter('a_dive', 3.0)             # B 下潜加速度 m/s²
        self.declare_parameter('a_brake', 6.0)            # B 刹车加速度 m/s²
        self.declare_parameter('funnel_mouth_radius', 0.20)
        self.declare_parameter('funnel_eff_radius', 0.15)  # mouth − object_radius
        self.declare_parameter('funnel_mount_height', 0.10)
        self.declare_parameter('payload_release_offset', 0.15)  # 载荷释放点相对 A 向下偏移 (m)
        self.declare_parameter('px4_z_bias', 0.24)   # PX4 pos_world.z 比模型绝对高度低的量(x500 base_link 在模型 z=0.24)
        self.declare_parameter('catch_z_tol', 0.12)  # 捕获时载荷可高出漏斗口平面的容差 (m)
        self.declare_parameter('auto_land', False)          # 捕获后自动降落
        self.declare_parameter('land_after_catch_s', 6.0)   # 捕获后再悬停多久开始着陆流程
        self.declare_parameter('land_xy', [5.0, 0.0])       # 世界系 NED 着陆点 x,y（与 A 分开）
        self.declare_parameter('land_xy_tol', 0.25)         # 到达着陆点的水平容差
        self.declare_parameter('funnel_depth', 0.30)
        self.declare_parameter('funnel_restitution', 0.60)
        self.declare_parameter('v_retain', 4.04)          # 刚性漏斗保持速度 m/s
        self.declare_parameter('stack_kp_xy', 1.5)
        self.declare_parameter('stack_kp_z', 1.5)

        self.standby = np.asarray(self.get_parameter('standby_world').value, float).reshape(3)
        self.r_c = float(self.get_parameter('capture_radius').value)
        self.v_c = float(self.get_parameter('capture_rel_speed').value)
        self.replan_dt = float(self.get_parameter('replan_dt').value)
        self.tau_max = float(self.get_parameter('inflight_tau_max').value)
        self.calt = (float(self.get_parameter('catch_alt_min').value),
                     float(self.get_parameter('catch_alt_max').value))
        self.v_max = float(self.get_parameter('b_max_speed').value)
        self.kp = float(self.get_parameter('kp_pos').value)

        self.a_release = np.asarray(self.get_parameter('a_release_world').value,
                                    float).reshape(3)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.plan_tr_max = float(self.get_parameter('plan_tr_max').value)
        # M6
        self.mode = str(self.get_parameter('mode').value).lower()
        self.a_state_topic = str(self.get_parameter('a_state_topic').value)
        self.rel_pos_sigma = float(self.get_parameter('rel_pos_sigma').value)
        self.rel_latency = float(self.get_parameter('rel_latency').value)
        self.rel_jitter = float(self.get_parameter('rel_jitter').value)
        self.rel_dropout = float(self.get_parameter('rel_dropout').value)
        self.rel_bias = float(self.get_parameter('rel_bias').value)
        self.payload_meas_sigma = float(self.get_parameter('payload_meas_sigma').value)
        self.payload_meas_latency = float(self.get_parameter('payload_meas_latency').value)
        self.payload_dropout = float(self.get_parameter('payload_dropout').value)
        self.track_payload = bool(self.get_parameter('track_payload').value)
        self._rng = np.random.default_rng(int(self.get_parameter('rel_seed').value))
        self.est_lpf_alpha = float(self.get_parameter('est_lpf_alpha').value)
        self._rel_bias_vec = np.zeros(3)
        self._a_est_f = None
        self._pay_est_f = None
        self._rel_last = None
        self._pay_hist = []
        self._pay_last_est = None
        self.align_xy_tol = float(self.get_parameter('align_xy_tol').value)
        self.align_vel_tol = float(self.get_parameter('align_vel_tol').value)
        self.align_alt_tol = float(self.get_parameter('align_alt_tol').value)
        self.align_hold_s = float(self.get_parameter('align_hold_s').value)
        self.approach_alt_tol = float(self.get_parameter('approach_alt_tol').value)
        self.min_ab_gap = float(self.get_parameter('min_ab_gap').value)
        self.release_lead = float(self.get_parameter('release_lead').value)
        self.a_dive = float(self.get_parameter('a_dive').value)
        self.a_brake = float(self.get_parameter('a_brake').value)
        self.funnel_mouth_radius = float(self.get_parameter('funnel_mouth_radius').value)
        self.funnel_eff_radius = float(self.get_parameter('funnel_eff_radius').value)
        self.funnel_mount_height = float(self.get_parameter('funnel_mount_height').value)
        self.payload_release_offset = float(self.get_parameter('payload_release_offset').value)
        self.px4_z_bias = float(self.get_parameter('px4_z_bias').value)
        self.catch_z_tol = float(self.get_parameter('catch_z_tol').value)
        self.auto_land = bool(self.get_parameter('auto_land').value)
        self.land_after_catch_s = float(self.get_parameter('land_after_catch_s').value)
        self.land_xy = np.asarray(self.get_parameter('land_xy').value, float).reshape(2)
        self.land_xy_tol = float(self.get_parameter('land_xy_tol').value)
        self._caught_t = None
        self.funnel_depth = float(self.get_parameter('funnel_depth').value)
        self.funnel_restitution = float(self.get_parameter('funnel_restitution').value)
        self.v_retain = float(self.get_parameter('v_retain').value)
        self.stack_kp_xy = float(self.get_parameter('stack_kp_xy').value)
        self.stack_kp_z = float(self.get_parameter('stack_kp_z').value)
        self.a_state_hist = []          # [(t, pos_world_NED, vel_NED)]
        self.a_est = None
        self.a_vel_est = None
        self.stack_plan = None
        self.release_ref_t0 = None
        self.align_t0 = None
        self.stack_hover = None         # 捕获后锁定的悬停点（防止重锚漂移靠近 A）
        self._min_relA = float('inf')   # 全程最小 A-B 间距（碰撞监测）
        self._t_node0 = None
        self.planned = False
        self.plan = None
        self.pub_release_at = self.create_publisher(Float64, '/payload/release_at', 10)
        self.controller = str(self.get_parameter('controller').value).lower()
        self.mpc = None
        if self.controller == 'mpc':
            from .mpc_terminal import TerminalMPC
            self.mpc = TerminalMPC(
                N=30, dt=0.02,
                a_max=float(self.get_parameter('b_max_accel').value),
                v_max=float(self.get_parameter('b_max_speed').value))
            self.get_logger().warn('b_node: 控制器 = acados 终端 MPC')
        self.cur_pc = None
        self.cur_vc = None
        self.planner = RendezvousPlanner(
            g=9.81, b_max_speed=self.v_max,
            b_max_accel=float(self.get_parameter('b_max_accel').value),
            capture_radius=self.r_c, capture_rel_speed=self.v_c)

        self.p_pay = None
        self.v_pay = None
        self.released = False
        self.caught = False
        self.phase = 'HOLD'
        self.ref_t = None
        self.ref_p = None
        self.ref_v = None
        self.ref_t0 = 0.0
        self.last_replan = -1e9
        self.t_sim = 0.0

        self.create_subscription(Float64MultiArray, '/payload/state', self._on_payload, 10)
        self.create_subscription(Bool, '/payload/released', self._on_released, 10)
        self.pub_caught = self.create_publisher(Bool, '/payload/caught', 10)
        if self.mode == 'stack':
            self.create_subscription(Float64MultiArray, self.a_state_topic, self._on_a_state, 10)
            self.phase = 'CLIMB'
            self.get_logger().warn('b_node: MODE=stack（垂直堆叠投放：对正→释放→温和下潜）')
        self.get_logger().info(f'b_node: standby={self.standby} offset={self.world_offset}')

    def _on_payload(self, msg):
        if len(msg.data) >= 7:
            self.p_pay = np.array([msg.data[1], msg.data[2], msg.data[3]])
            self.v_pay = np.array([msg.data[4], msg.data[5], msg.data[6]])
            self.t_sim = float(msg.data[0])
            self._pay_hist.append((self.get_clock().now().nanoseconds * 1e-9,
                                   self.p_pay.copy(), self.v_pay.copy()))
            if len(self._pay_hist) > 4000:
                self._pay_hist.pop(0)

    def _on_released(self, msg):
        if msg.data and not self.released:
            self.released = True
            if self.mode != 'stack':
                self.phase = 'RENDEZ'
            self.get_logger().warn('B: payload released → rendezvous')

    def _on_a_state(self, msg):
        if len(msg.data) >= 7:
            self.a_state_hist.append((float(msg.data[0]),
                                      np.array(msg.data[1:4], float),
                                      np.array(msg.data[4:7], float)))
            if len(self.a_state_hist) > 4000:
                self.a_state_hist.pop(0)

    def _relnav_a(self, now):
        """相对定位（mesh 替身）：A 广播位姿 + 延迟(含抖动) + 丢包 + 慢变偏置 + 白噪声。"""
        if not self.a_state_hist:
            return None, None
        if self.rel_bias > 0:      # 慢变偏置（随机游走，模拟标定漂移/多径）
            self._rel_bias_vec = np.clip(
                self._rel_bias_vec + self._rng.normal(0.0, self.rel_bias * 0.02, 3),
                -3.0 * self.rel_bias, 3.0 * self.rel_bias)
        j = len(self.a_state_hist) - 1
        lat = self.rel_latency + (self._rng.uniform(0.0, self.rel_jitter)
                                  if self.rel_jitter > 0 else 0.0)
        tgt = now - lat
        while j > 0 and self.a_state_hist[j][0] > tgt:
            j -= 1
        _t, p, v = self.a_state_hist[j]
        if (self.rel_dropout > 0 and self._rel_last is not None
                and self._rng.random() < self.rel_dropout):
            return self._rel_last          # 丢包：沿用上一帧
        p = p + self._rel_bias_vec
        if self.rel_pos_sigma > 0:
            p = p + self._rng.normal(0.0, self.rel_pos_sigma, 3)
        # EMA 低通：抑制白噪声/抖动，否则对正门限会被噪声卡住
        if self._a_est_f is None or self.est_lpf_alpha >= 1.0:
            self._a_est_f = np.asarray(p, float)
        else:
            a = self.est_lpf_alpha
            self._a_est_f = (1.0 - a) * self._a_est_f + a * np.asarray(p, float)
        self._rel_last = (self._a_est_f.copy(), v.copy())
        return self._rel_last

    def _payload_est(self, now):
        """载荷状态估计（延迟 + 丢包 + 白噪声），供 DIVE 阶段闭环跟踪。"""
        if not self._pay_hist:
            return None
        j = len(self._pay_hist) - 1
        lat = self.payload_meas_latency + (self._rng.uniform(0.0, self.rel_jitter)
                                           if self.rel_jitter > 0 else 0.0)
        tgt = now - lat
        while j > 0 and self._pay_hist[j][0] > tgt:
            j -= 1
        _t, p, v = self._pay_hist[j]
        if (self.payload_dropout > 0 and self._pay_last_est is not None
                and self._rng.random() < self.payload_dropout):
            return self._pay_last_est
        p = p.copy()
        if self.payload_meas_sigma > 0:
            p = p + self._rng.normal(0.0, self.payload_meas_sigma, 3)
        if self._pay_est_f is None or self.est_lpf_alpha >= 1.0:
            self._pay_est_f = p
        else:
            a = self.est_lpf_alpha
            self._pay_est_f = (1.0 - a) * self._pay_est_f + a * p
        self._pay_last_est = (self._pay_est_f.copy(), v.copy())
        return self._pay_last_est

    def _capture_check(self):
        if self.p_pay is None or self.caught:
            return
        d = float(np.linalg.norm(self.pos_world - self.p_pay))
        rv = float(np.linalg.norm(self.vel - self.v_pay))
        if d < self.r_c and rv < self.v_c:
            self.caught = True
            self.phase = 'DONE'
            self.pub_caught.publish(Bool(data=True))
            self.get_logger().warn(
                f'*** CAPTURED *** d={d:.3f}m rel_v={rv:.3f}m/s '
                f'p_B={self.pos_world.round(2)} p_p={self.p_pay.round(2)}')

    def control(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._tick_count % int(self.hz) == 0:
            pp = None if self.p_pay is None else self.p_pay.round(2)
            ra = None if self.a_est is None else round(float(np.linalg.norm(self.a_est - self.pos_world)), 3)
            mr = None if self._min_relA == float('inf') else round(self._min_relA, 3)
            self.get_logger().info(
                f'B phase={self.phase} pos_w={self.pos_world.round(2)} vel={self.vel.round(2)} '
                f'relA={ra} min_relA={mr} pay={pp} caught={self.caught}')
        if self.mode == 'stack':
            self.control_stack(now)
            return
        # ⚠️ 早退只能看 DONE；不能因 p_pay is None 早退——否则 HOLD 阶段的规划永远执行不到
        if self.phase == 'DONE':
            v = self.hover_velocity(self.standby[:2], -self.standby[2],
                                    kp_xy=1.0, max_speed=self.v_max, kp_z=1.0, max_climb=1.5)
            self.publish_velocity(v, yaw=self.yaw)
            return

        if self.phase == 'HOLD':
            # 起飞稳定 start_delay 后做一次规划，并立即开始执行 B 的会合参考（预位）
            if self._t_node0 is None:
                self._t_node0 = now
            if (not self.planned) and (now - self._t_node0) >= self.start_delay:
                plan = self.planner.solve(
                    self.a_release, self.pos_world, a_vel=(0.0, 0.0, 0.0),
                    b_v0=self.vel,
                    t_r_range=(1.0, self.plan_tr_max), tau_range=(0.05, 1.5),
                    t_r_step=0.05, tau_step=0.01, catch_alt_range=self.calt)
                self.planned = True
                if plan.feasible:
                    self.plan = plan
                    self.ref_t, self.ref_p, self.ref_v, _ = RendezvousPlanner.resample(plan, n=401)
                    self.ref_t0 = now
                    self.phase = 'RENDEZ'
                    self.pub_release_at.publish(Float64(data=now + plan.t_r))
                    self.get_logger().warn(
                        f'B: PLAN ok t_r={plan.t_r:.2f}s p_c={plan.p_c.round(2)} '
                        f'v_p={plan.v_p.round(2)} h_c={plan.h_c:.2f} → 广播释放时刻，开始预位')
                else:
                    self.get_logger().error(f'B: PLAN infeasible: {plan.reason}')
            v = self.hover_velocity(self.standby[:2], -self.standby[2],
                                    kp_xy=1.0, max_speed=self.v_max, kp_z=1.0, max_climb=1.5)
            self.publish_velocity(v, yaw=self.yaw)
            return

        # RENDEZ：闭环重规划 + 速度前馈 + 位置 P
        if self.p_pay is None:                       # 载荷状态还没来：先悬停
            v = self.hover_velocity(self.standby[:2], -self.standby[2],
                                    kp_xy=1.0, max_speed=self.v_max, kp_z=1.0, max_climb=1.5)
            self.publish_velocity(v, yaw=self.yaw)
            return
        if (now - self.last_replan) >= self.replan_dt:
            self.last_replan = now
            rp = self.planner.solve_inflight(
                self.p_pay, self.v_pay, self.pos_world, self.vel,
                tau_range=(0.05, self.tau_max), tau_step=0.01,
                catch_alt_range=self.calt)
            if rp.feasible:
                self.ref_t, self.ref_p, self.ref_v, _ = RendezvousPlanner.resample(rp, n=201)
                self.cur_pc, self.cur_vc = rp.p_c.copy(), rp.v_p.copy()
                self.ref_t0 = now

        if (self.ref_t is not None) and (now - self.ref_t0) < float(self.ref_t[-1]):
            tl = max(now - self.ref_t0, 0.0)
            j = int(round(tl / float(self.ref_t[-1]) * (len(self.ref_t) - 1)))
            p_ref = self.ref_p[j]
            v_ref = self.ref_v[j].copy()
            if self.mpc is not None:
                x0 = np.concatenate([self.pos_world, self.vel])
                _u0, st, v_pred = self.mpc.solve(x0, self.ref_t, self.ref_p, self.ref_v,
                                                 self.cur_pc, self.cur_vc, t_start=tl)
                if st in (0, 2):
                    v_sp = v_pred + 0.5 * self.kp * (p_ref - self.pos_world)
                else:
                    v_sp = v_ref + self.kp * (p_ref - self.pos_world)
            else:
                v_sp = v_ref + self.kp * (p_ref - self.pos_world)
        else:
            # 参考执行完（或已过会合时刻）→ 在会合点悬停。
            # ⚠️ 不能继续用末点 v_ref(=载荷速度)：那会让 B 一直俯冲砸地。
            tgt = self.ref_p[-1] if self.ref_t is not None else self.standby
            v_sp = self.hover_velocity(tgt[:2], -tgt[2],
                                       kp_xy=1.5, max_speed=self.v_max, kp_z=1.5, max_climb=2.0)
        n = float(np.linalg.norm(v_sp[:2]))
        if n > self.v_max:
            v_sp[:2] *= self.v_max / n
        v_sp[2] = float(np.clip(v_sp[2], -self.v_max, self.v_max))
        self.publish_velocity(v_sp, yaw=self.yaw)
        self._capture_check()


    # ---------------------------------------------------------------- M6 stack
    def _stack_capture_check(self):
        if self.caught or self.p_pay is None:
            return
        pos = self.pos_world
        # 漏斗口平面的 NED z（px4_z_bias 把 PX4 世界系换算回模型绝对高度）
        z_mouth = pos[2] - self.px4_z_bias - self.funnel_mount_height
        horiz = float(np.linalg.norm(pos[:2] - self.p_pay[:2]))
        rv = float(np.linalg.norm(self.vel - self.v_pay))
        if (self.p_pay[2] >= z_mouth - self.catch_z_tol and horiz <= self.funnel_eff_radius
                and rv <= self.v_retain):
            self.caught = True
            self.phase = 'DONE'
            self.stack_hover = pos.copy()   # 锁定此刻位置为悬停点
            self._caught_t = self.get_clock().now().nanoseconds * 1e-9
            self.pub_caught.publish(Bool(data=True))
            self.get_logger().warn(
                f'*** STACK CAPTURED *** horiz={horiz:.3f}m rel_v={rv:.3f}m/s '
                f'z_mouth={z_mouth:.2f} p_B={pos.round(2)} p_p={self.p_pay.round(2)}')

    def control_stack(self, now):
        pos = self.pos_world
        p_est, v_est = self._relnav_a(now)
        if p_est is not None:
            self.a_est, self.a_vel_est = p_est, v_est
            self._min_relA = min(self._min_relA,
                                 float(np.linalg.norm(self.a_est - pos)))
        standby = self.standby
        a_alt = -float(self.a_release[2])          # A 的悬停高度（应从 launch 传入 a_hover）

        if self.phase == 'CLIMB':
            # 1) 只在本机 x/y **垂直爬升**到待命高度：绝不平移，避免斜插进 A 的爬升通道
            tgt_alt = -standby[2]
            v = self.hover_velocity([pos[0], pos[1]], tgt_alt, kp_xy=1.0, max_speed=0.8,
                                    kp_z=1.4, max_climb=1.2)
            if abs(-pos[2] - tgt_alt) < self.approach_alt_tol:
                self.phase = 'WAIT_A'
                self.get_logger().warn('B: CLIMB done → WAIT_A（保持机位等 A 爬到顶）')
            self.publish_velocity(v, yaw=self.yaw)
            return

        if self.phase == 'WAIT_A':
            # 2) 原地悬停，等 A 到位且比 B 高出 min_ab_gap，才开始横移
            tgt_alt = -standby[2]
            v = self.hover_velocity([pos[0], pos[1]], tgt_alt, kp_xy=1.0, max_speed=0.8,
                                    kp_z=1.4, max_climb=1.2)
            self.publish_velocity(v, yaw=self.yaw)
            if self.a_est is None:
                return
            a_alt_now = -self.a_est[2]
            a_vz = abs(float(self.a_vel_est[2])) if self.a_vel_est is not None else 9.9
            clear = a_alt_now - (-pos[2])
            if (a_alt_now >= a_alt - self.approach_alt_tol and a_vz < self.align_vel_tol
                    and clear >= self.min_ab_gap):
                self.phase = 'TRANSLATE'
                self.get_logger().warn(
                    f'B: WAIT_A done (A_alt={a_alt_now:.2f}, clear={clear:.2f}) → TRANSLATE')
            return

        if self.phase == 'TRANSLATE':
            # 3) 保持高度平移到 A 正下方；全程比 A 低 min_ab_gap
            tgt_alt = -standby[2]
            if self.a_est is not None:
                tgt_alt = min(tgt_alt, (-self.a_est[2]) - self.min_ab_gap)
            v = self.hover_velocity(standby[:2], tgt_alt, kp_xy=1.2, max_speed=1.5,
                                    kp_z=1.4, max_climb=1.0)
            self.publish_velocity(v, yaw=self.yaw)
            if float(np.linalg.norm(pos[:2] - standby[:2])) < self.align_xy_tol:
                self.phase = 'ALIGN'
                self.align_t0 = now
                self.get_logger().warn('B: TRANSLATE done → ALIGN')
            return

        if self.phase == 'ALIGN':
            tgt_alt = -standby[2]
            if self.a_est is not None:
                tgt_alt = min(tgt_alt, (-self.a_est[2]) - self.min_ab_gap)
            v = self.hover_velocity(standby[:2], tgt_alt, kp_xy=1.2, max_speed=1.5,
                                    kp_z=1.4, max_climb=1.2)
            self.publish_velocity(v, yaw=self.yaw)
            if self.a_est is None:
                self.align_t0 = now
                return
            rel_xy = float(np.linalg.norm(self.a_est[:2] - pos[:2]))
            spd_xy = float(np.linalg.norm(self.vel[:2]))
            alt_ok = abs(-pos[2] - (-standby[2])) < self.align_alt_tol
            a_slow = (self.a_vel_est is None
                      or float(np.linalg.norm(self.a_vel_est)) < self.align_vel_tol)
            aligned = (rel_xy < self.align_xy_tol and spd_xy < self.align_vel_tol
                       and alt_ok and a_slow)
            if aligned:
                if self.align_t0 is None:
                    self.align_t0 = now
                elif (now - self.align_t0) >= self.align_hold_s:
                    rel_t = now + self.release_lead
                    self.pub_release_at.publish(Float64(data=rel_t))
                    self.release_ref_t0 = rel_t
                    self.phase = 'DIVE'
                    self.get_logger().warn(
                        f'B: ALIGNED rel_xy={rel_xy:.3f}m spd_xy={spd_xy:.3f} → release@{rel_t:.2f}')
            else:
                self.align_t0 = now
            return

        if self.phase == 'DIVE':
            if self.release_ref_t0 is None:
                self.publish_velocity(np.zeros(3), yaw=self.yaw)
                return
            tl = now - self.release_ref_t0
            if tl < 0.0:
                v = self.hover_velocity(standby[:2], -standby[2], kp_xy=1.5,
                                        max_speed=self.v_max, kp_z=1.5, max_climb=1.5)
                self.publish_velocity(v, yaw=self.yaw)
                return
            if self.stack_plan is None:
                a_h = -self.a_est[2] if self.a_est is not None else -standby[2]
                # 载荷实际从 A 下方 offset 处释放；漏斗口在 B 机体上方 mount 处。
                # 让 plan 的有效 gap = 载荷→漏斗口的距离（_stack_ref 仍从 B 机体起步）。
                a_h = a_h - self.payload_release_offset - self.funnel_mount_height
                self.stack_plan = plan_stack_drop(
                    a_height=a_h, b_height=-pos[2], a_dive=self.a_dive, g=9.81,
                    a_brake=self.a_brake, funnel_depth=self.funnel_depth,
                    restitution=self.funnel_restitution)
                self.get_logger().warn(
                    f'B: DIVE plan t_c={self.stack_plan.t_c:.3f}s v_rel={self.stack_plan.v_rel:.3f} '
                    f'v_retain={self.stack_plan.v_retain:.3f} feasible={self.stack_plan.feasible}')
            xy_tgt = self.a_est[:2] if self.a_est is not None else pos[:2]
            vxy_ff = np.zeros(2)
            if self.track_payload:
                pe = self._payload_est(now)     # 闭环：跟踪载荷本身（含测量噪声/延迟）
                if pe is not None:
                    xy_tgt = pe[0][:2]
                    vxy_ff = pe[1][:2]
            pr, vr, _ar = _stack_ref(tl, self.stack_plan, (xy_tgt[0], xy_tgt[1]), 9.81)
            v_sp = np.zeros(3)
            v_sp[:2] = vxy_ff + self.stack_kp_xy * (xy_tgt - pos[:2])
            v_sp[2] = vr[2] + self.stack_kp_z * (pr[2] - pos[2])
            n = float(np.linalg.norm(v_sp[:2]))
            if n > self.v_max:
                v_sp[:2] *= self.v_max / n
            v_sp[2] = float(np.clip(v_sp[2], -self.v_max, self.v_max))
            self.publish_velocity(v_sp, yaw=self.yaw)
            self._stack_capture_check()
            return

        # DONE：捕获后在**固定点**悬停（不能每拍把目标重锚到当前位置，否则带载会漂移靠近 A）
        if self.auto_land and self._caught_t is not None and not self._landing:
            if now - self._caught_t >= self.land_after_catch_s:
                self.phase = 'LAND'
                # 先飞到自己的着陆点（保持高度），到位后再落地
                if float(np.linalg.norm(pos[:2] - self.land_xy)) < self.land_xy_tol:
                    self.land()
                else:
                    v = self.hover_velocity(self.land_xy, -pos[2], kp_xy=1.0,
                                            max_speed=1.0, kp_z=1.4, max_climb=1.0)
                    self.publish_velocity(v, yaw=self.yaw)
                return
        if self.stack_hover is None:
            self.stack_hover = pos.copy()
        tgt = self.stack_hover.copy()
        tgt_alt = -tgt[2]
        # 安全层：悬停高度不得高于 A−min_ab_gap，保证绝不靠近 A
        if self.a_est is not None:
            tgt_alt = min(tgt_alt, (-self.a_est[2]) - self.min_ab_gap)
        v = self.hover_velocity(tgt[:2], tgt_alt, kp_xy=1.5, max_speed=self.v_max,
                                kp_z=1.5, max_climb=1.5)
        self.publish_velocity(v, yaw=self.yaw)


def main(args=None):
    rclpy.init(args=args)
    node = BNode()
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

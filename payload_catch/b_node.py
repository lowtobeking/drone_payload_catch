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
from .stack_drop import plan_stack_drop, _stack_ref, _adaptive_dive, minimal_dive, retain_speed
from .contact_detect import ContactDetector
from .keepout import cbf_bound, project_cbf


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
        self.declare_parameter('safety_k', 2.0)           # 安全层：keep-out 额外 kσ（估计不确定度）
        self.declare_parameter('rel_sigma_floor', 0.0)    # σ 下限 (m)
        self.declare_parameter('release_lead', 0.20)      # 提前广播释放时刻
        # ── M6-moving：编队同速投放 ──
        self.declare_parameter('formation_vel', [0.0, 0.0, 0.0])   # 同向同速巡航速度（世界系 NED 水平）
        self.declare_parameter('formation_topic', '/formation/start')
        # ── 协同释放握手：'direct'(B 直接决定释放) | 'handshake'(B 报就绪→等 A 释放 ack) ──
        self.declare_parameter('coord_mode', 'direct')
        self.declare_parameter('handshake_timeout', 1.0)   # 等 A 释放 ack 的上限 (s)
        self.declare_parameter('clock_sync', True)         # 往返估 A/B 时钟偏移并换算释放时刻
        self.declare_parameter('use_intent', False)        # 用 A 广播的【预测落点】做对正目标
        self.declare_parameter('zem_gain', 0.0)            # 终端导引(ZEM) 增益：减终端 miss
        self.declare_parameter('form_kp_rel', 0.5)         # 编队：相对测量校正增益（死推算参考为主）
        self.declare_parameter('align_reset_tol', 0.25)    # 编队：短晩失配容忍（内不重置保持计时）
        self.declare_parameter('formation_timeout_s', 12.0)  # 编队释放超时→中止投放
        self.declare_parameter('formation_min_speed_ratio', 0.8)   # 编队速度达到该比例才允许释放
        self.declare_parameter('a_dive', 3.0)             # B 下潜加速度 m/s²（auto_min_dive=False 时直接用它）
        self.declare_parameter('auto_min_dive', True)      # 用最小必要下潜（gap≤v_retain²/2g 时免下潜）
        self.declare_parameter('a_brake', 6.0)            # B 刹车加速度 m/s²
        self.declare_parameter('funnel_mouth_radius', 0.20)
        self.declare_parameter('funnel_eff_radius', 0.15)  # mouth − object_radius
        self.declare_parameter('funnel_mount_height', 0.10)
        self.declare_parameter('payload_release_offset', 0.15)  # 载荷释放点相对 A 向下偏移 (m)
        self.declare_parameter('px4_z_bias', 0.24)   # PX4 pos_world.z 比模型绝对高度低的量(x500 base_link 在模型 z=0.24)
        self.declare_parameter('catch_z_tol', 0.10)  # 捕获时载荷可高出漏斗口平面的容差 (m)
        self.declare_parameter('dive_anchor_vz', 0.5)  # 载荷竖直速度超过此值视为"真正开始下落"(m/s)
        self.declare_parameter('capture_min_vz', 1.0)  # 捕获时载荷竖直速度下限（排除仍挂载/未下落）
        self.declare_parameter('auto_land', False)          # 捕获后自动降落
        self.declare_parameter('land_after_catch_s', 6.0)   # 捕获后再悬停多久开始着陆流程
        self.declare_parameter('land_xy', [5.0, 0.0])       # 世界系 NED 着陆点 x,y（与 A 分开）
        self.declare_parameter('land_xy_tol', 0.25)         # 到达着陆点的水平容差
        self.declare_parameter('funnel_depth', 0.30)
        self.declare_parameter('funnel_restitution', 0.60)
        # 主动保持（B 侧锁扣）：捕获后请求 payload_node 把载荷锁到 B 漏斗
        self.declare_parameter('lock_to_b', False)
        self.declare_parameter('lock_request_topic', '/payload/lock_request')
        self.declare_parameter('v_retain', 4.04)          # 刚性漏斗保持速度 m/s
        # 接触检测（真机）：用真实接触事件触发捕获/锁扣，替代纯软件判据
        self.declare_parameter('contact_detect', False)        # 开启后以接触事件为准（含超时兜底）
        self.declare_parameter('contact_accel_thresh', 15.0)   # 冲击加速度阈值 m/s²
        self.declare_parameter('contact_topic', '/payload/contact')  # 外部接触开关 (Bool)
        self.declare_parameter('contact_timeout_s', 0.30)      # 进入窗口后多久无接触则兜底
        self.declare_parameter('stack_kp_xy', 1.2)
        self.declare_parameter('stack_kp_z', 1.5)
        self.declare_parameter('a_ff_gain', 1.0)   # 加速度前馈增益（用 _stack_ref 的 ar）
        # 3D 反应式 keep-out：接近 A 时去掉向内速度 + 排斥（静态高度规则之外的兵底）
        self.declare_parameter('keepout_enable', True)
        self.declare_parameter('keepout_dist', 0.60)   # 触发距离 (m)
        self.declare_parameter('keepout_gain', 1.0)    # 排斥增益 (1/s)
        self.declare_parameter('keepout_mode', 'heuristic')  # heuristic | cbf（C5 速度级 CBF）
        self.declare_parameter('keepout_alpha', 1.0)   # CBF 指数增益 α
        self.declare_parameter('keepout_delay_s', 0.0)  # T3：延迟鲁棒收紧用的通信延迟 (s)
        # 在线自适应下潜：滚动重解 a_dive（抗垂直扰动，如下击暴流）
        self.declare_parameter('adaptive_dive', False)
        self.declare_parameter('adaptive_alt_floor', 0.35)   # 刹停后最小离地 (m)

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
        self.safety_k = float(self.get_parameter('safety_k').value)
        self.rel_sigma_floor = float(self.get_parameter('rel_sigma_floor').value)
        self._sigma_est = 0.0        # 在线估计的 A 相对位置误差 (m)
        self.release_lead = float(self.get_parameter('release_lead').value)
        self.formation_vel = np.asarray(self.get_parameter('formation_vel').value,
                                        float).reshape(3)
        self.formation_vel[2] = 0.0
        self.use_formation = float(np.linalg.norm(self.formation_vel[:2])) > 1e-6
        self.formation_min_speed_ratio = float(
            self.get_parameter('formation_min_speed_ratio').value)
        self._formation_sent = False
        self.form_t0 = None
        self.a_dive = float(self.get_parameter('a_dive').value)
        self.auto_min_dive = bool(self.get_parameter('auto_min_dive').value)
        self.a_brake = float(self.get_parameter('a_brake').value)
        self.funnel_mouth_radius = float(self.get_parameter('funnel_mouth_radius').value)
        self.funnel_eff_radius = float(self.get_parameter('funnel_eff_radius').value)
        self.funnel_mount_height = float(self.get_parameter('funnel_mount_height').value)
        self.payload_release_offset = float(self.get_parameter('payload_release_offset').value)
        self.px4_z_bias = float(self.get_parameter('px4_z_bias').value)
        self.catch_z_tol = float(self.get_parameter('catch_z_tol').value)
        self.dive_anchor_vz = float(self.get_parameter('dive_anchor_vz').value)
        self.capture_min_vz = float(self.get_parameter('capture_min_vz').value)
        self._dive_anchored = False
        # 接触检测状态
        self.contact_detect = bool(self.get_parameter('contact_detect').value)
        self.contact = ContactDetector(
            accel_thresh=float(self.get_parameter('contact_accel_thresh').value))
        self.contact_timeout_s = float(self.get_parameter('contact_timeout_s').value)
        self._contact_last_vz = 0.0
        self._contact_last_t = 0.0
        self._ext_contact = False
        self._near_since = None
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
        self.a_ff_gain = float(self.get_parameter('a_ff_gain').value)
        self.keepout_enable = bool(self.get_parameter('keepout_enable').value)
        self.keepout_dist = float(self.get_parameter('keepout_dist').value)
        self.keepout_gain = float(self.get_parameter('keepout_gain').value)
        self.keepout_mode = str(self.get_parameter('keepout_mode').value).lower()
        self.keepout_alpha = float(self.get_parameter('keepout_alpha').value)
        self.keepout_delay_s = float(self.get_parameter('keepout_delay_s').value)
        self.adaptive_dive = bool(self.get_parameter('adaptive_dive').value)
        self.adaptive_alt_floor = float(self.get_parameter('adaptive_alt_floor').value)
        self._a_adapt = None
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
        self.coord_mode = str(self.get_parameter('coord_mode').value)
        self.handshake_timeout = float(self.get_parameter('handshake_timeout').value)
        self.clock_sync = bool(self.get_parameter('clock_sync').value)
        self.use_intent = bool(self.get_parameter('use_intent').value)
        self.zem_gain = float(self.get_parameter('zem_gain').value)
        self.form_kp_rel = float(self.get_parameter('form_kp_rel').value)
        self.align_reset_tol = float(self.get_parameter('align_reset_tol').value)
        self.formation_timeout_s = float(self.get_parameter('formation_timeout_s').value)
        self.form_ref_p0 = None        # 编队：B 自身死推算参考起点
        self.form_ref_t0 = None
        self._rel0 = np.zeros(2)       # 进入编队时的 (A_est - B) 相对偏移
        self._form_timeout_t0 = None   # 进入编队时刻（用于释放超时）
        self._last_good_t = 0.0        # 上次对齐时刻（容忍短暂抖动）
        self._aborted = False
        self.pub_formation_abort = self.create_publisher(Bool, '/formation/abort', 10)
        self.clock_offset = 0.0        # A 时钟 − B 时钟 (s)
        self._ping_send = None
        self._last_ping = 0.0
        self._clock_logged = False
        self.pub_ping = self.create_publisher(Float64MultiArray, '/coord/ping', 10)
        self.create_subscription(Float64MultiArray, '/coord/pong', self._on_pong, 10)
        self._a_intent = None
        self._release_cmd = None
        self._release_cmd_t = 0.0
        self.pub_ready = self.create_publisher(Float64MultiArray, '/drone_b/ready', 10)
        if self.coord_mode == 'handshake':
            self.create_subscription(Float64MultiArray, '/drone_a/intent', self._on_a_intent, 10)
            self.create_subscription(Float64MultiArray, '/drone_a/release_cmd',
                                     self._on_release_cmd, 10)
            self.get_logger().warn('B: coord_mode=handshake（报就绪 → 等 A 释放 ack）')
        self.pub_formation = self.create_publisher(
            Bool, str(self.get_parameter('formation_topic').value), 10)
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
        self.create_subscription(Bool, str(self.get_parameter('contact_topic').value),
                                 self._on_contact, 10)
        self.create_subscription(Bool, '/payload/released', self._on_released, 10)
        self.pub_caught = self.create_publisher(Bool, '/payload/caught', 10)
        self.lock_to_b = bool(self.get_parameter('lock_to_b').value)
        self._lock_req_pub = None
        if self.lock_to_b:
            self._lock_req_pub = self.create_publisher(
                Float64MultiArray, str(self.get_parameter('lock_request_topic').value), 10)
            self.get_logger().warn('b_node: lock_to_b=True（捕获后请求在 B 漏斗处锁扣）')
        if self.mode == 'stack':
            self.create_subscription(Float64MultiArray, self.a_state_topic, self._on_a_state, 10)
            self.phase = 'CLIMB'
            self.get_logger().warn('b_node: MODE=stack（垂直堆叠投放：对正→释放→温和下潜）')
            if self.use_formation:
                self.get_logger().warn(
                    f'b_node: 编队同速投放 formation_vel={self.formation_vel[:2]}')
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

    def _on_contact(self, msg):
        """外部接触开关（力/微动/红外对射等）→ 置一次接触标志。"""
        if bool(msg.data):
            self._ext_contact = True

    def _on_released(self, msg):
        if msg.data and not self.released:
            self.released = True
            if self.mode != 'stack':
                self.phase = 'RENDEZ'
            elif (self.coord_mode == 'handshake'
                  and self.phase in ('ALIGN', 'FORMATION') and not self._dive_anchored):
                # C2 冗余：若 release_cmd 丢失，用 /payload/released（载荷实际分离）兜底触发 DIVE
                self.release_ref_t0 = self.get_clock().now().nanoseconds * 1e-9
                self._dive_anchored = False
                self.phase = 'DIVE'
                self.get_logger().warn(
                    'B: /payload/released → 冗余触发 DIVE（release_cmd 可能丢失）')
            self.get_logger().warn('B: payload released → rendezvous')

    def _on_a_intent(self, msg):
        self._a_intent = list(msg.data)

    def _on_release_cmd(self, msg):
        self._release_cmd = list(msg.data)
        self._release_cmd_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_pong(self, msg):
        """时钟同步：data=[t_b_send, t_a_recv] → 估 offset=(A−B)，EMA 平滑。"""
        if not self.clock_sync or self._ping_send is None:
            return
        t_b_send = float(msg.data[0])
        t_a_recv = float(msg.data[1])
        t_b_recv = self.get_clock().now().nanoseconds * 1e-9
        rtt = t_b_recv - t_b_send
        if rtt < 0.0 or rtt > 2.0:
            return
        off = t_a_recv + 0.5 * rtt - t_b_recv
        self.clock_offset = 0.9 * self.clock_offset + 0.1 * off
        if not self._clock_logged:
            self._clock_logged = True
            self.get_logger().warn(
                f'B: 时钟同步 offset(A−B)={self.clock_offset:+.4f}s rtt={rtt*1000:.1f}ms')

    def _pub_ready(self, now, ready, rel_xy, spd_xy):
        m = Float64MultiArray()
        # [ready, rel_xy, spd_xy, stamp, sigma_abs, sigma_rel]
        #   sigma_abs = 在线残差 ⊕ EKF 自身 σ（keep-out/启发式用）
        #   sigma_rel = 纯相对定位残差（证书闸用；与绝对 σ 分离）
        sigma_rel = float(max(self._sigma_est, self.rel_sigma_floor))
        sigma_abs = sigma_rel
        if self.sensor_constraints_enable and self.sensor_use_ekf_sigma:
            sigma_abs = float(np.sqrt(sigma_rel ** 2 + self.pos_sigma_h ** 2))
        m.data = [float(ready), float(rel_xy), float(spd_xy), float(now),
                  sigma_abs, sigma_rel]
        self.pub_ready.publish(m)

    def _gap_eff(self):
        """安全 keep-out：min_ab_gap + kσ（合并在线残差与 EKF 垂直 σ）。"""
        sigma = max(self._sigma_est, self.rel_sigma_floor)
        if self.sensor_constraints_enable and self.sensor_use_ekf_sigma:
            sigma = max(sigma, self.pos_sigma_v)
        return self.min_ab_gap + self.safety_k * sigma

    def _keepout_velocity(self, v_sp):
        """3D 反应式 keep-out：若接近 A，去掉向 A 的速度分量并叠加排斥。

        与“B 高度 ≤ A−gap_eff”的静态规则互补：后者靠高度，前者靠真实三维距离，
        在水平贴近/异常机动（例如估计跳变）时提供最后一道防碰。
        """
        if (not self.keepout_enable) or self.a_est is None:
            return v_sp
        d_vec = self.a_est - self.pos_world          # B→A
        dist = float(np.linalg.norm(d_vec))
        if dist > self.keepout_dist or dist < 1e-6:
            return v_sp
        u = d_vec / dist                             # 单位向量，指向 A
        v = np.asarray(v_sp, float).copy()
        v_in = float(np.dot(v, u))
        if v_in > 0.0:                               # 有朝 A 的分量 → 去掉
            v = v - v_in * u
        v = v - self.keepout_gain * (self.keepout_dist - dist) * u   # 排斥（远离 A）
        return v

    def _cbf_velocity(self, v_sp):
        """C5：速度级 CBF 防碰滤波（B 侧，用 B 对 A 的估计）。

        含 T3 延迟鲁棒：a 相对状态延迟 `keepout_delay_s` 时，把安全半径收紧
        `d_eff = keepout_dist + (‖v_A‖+‖v_B‖)·keepout_delay_s`（ρ 为延迟内相对位移界）。
        """
        if self.a_est is None:
            return v_sp
        r = self.a_est - self.pos_world
        v_A = self.a_vel_est if self.a_vel_est is not None else np.zeros(3)
        rho = (float(np.linalg.norm(v_A)) + float(np.linalg.norm(self.vel))) * self.keepout_delay_s
        d_eff = self.keepout_dist + rho
        h = float(r @ r) - d_eff ** 2
        c = cbf_bound(r, v_A, h, self.keepout_alpha)
        return project_cbf(np.asarray(v_sp, float), r, c, self.v_max)

    def publish_velocity(self, vel_ned, yaw=0.0, acc_ff=None):
        """B 的栈模式叠加 keep-out（heuristic 排斥 或 C5 CBF 滤波）。"""
        if self.mode == 'stack':
            if self.keepout_mode == 'cbf':
                vel_ned = self._cbf_velocity(vel_ned)
            else:
                vel_ned = self._keepout_velocity(vel_ned)
        super().publish_velocity(vel_ned, yaw=yaw, acc_ff=acc_ff)

    def _on_a_state(self, msg):
        if len(msg.data) >= 7:
            p_meas = np.array(msg.data[1:4], float)
            self.a_state_hist.append((float(msg.data[0]), p_meas,
                                      np.array(msg.data[4:7], float)))
            if len(self.a_state_hist) > 4000:
                self.a_state_hist.pop(0)
            # 在线估计 A 相对位置误差 σ（测量与平滑估计的残差 EMA），供安全 keep-out 用
            if self.a_est is not None:
                resid = float(np.linalg.norm(p_meas - self.a_est))
                self._sigma_est = 0.95 * self._sigma_est + 0.05 * resid

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
        if self.clock_sync and (now - self._last_ping) > 0.2:
            self._last_ping = now
            self._ping_send = now
            m = Float64MultiArray()
            m.data = [float(now)]
            self.pub_ping.publish(m)
        if self._tick_count % int(self.hz) == 0:
            pp = None if self.p_pay is None else self.p_pay.round(2)
            ra = None if self.a_est is None else round(float(np.linalg.norm(self.a_est - self.pos_world)), 3)
            mr = None if self._min_relA == float('inf') else round(self._min_relA, 3)
            self.get_logger().info(
                f'B phase={self.phase} safe={self._safety_state} att={self._att_gov:.2f} '
                f'pos_w={self.pos_world.round(2)} '
                f'vel={self.vel.round(2)} '
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
    def _abort_formation(self, now):
        """编队释放超时 → 中止投放：通知 A 停止巡航，B 安全悬停并（若开）降落。"""
        if self._aborted:
            return
        self._aborted = True
        self.pub_formation_abort.publish(Bool(data=True))
        self.get_logger().error(
            f'B: 编队释放超时 ({self.formation_timeout_s:.0f}s) → 中止投放，安全悬停→降落')
        self.stack_hover = self.pos_world.copy()
        self._caught_t = now          # 复用 DONE 的悬停+自动降落逻辑
        self.phase = 'DONE'

    def _stack_capture_check(self):
        if self.caught or self.p_pay is None:
            return
        pos = self.pos_world
        # 漏斗口平面的 NED z（px4_z_bias 把 PX4 世界系换算回模型绝对高度）
        z_mouth = pos[2] - self.px4_z_bias - self.funnel_mount_height
        horiz = float(np.linalg.norm(pos[:2] - self.p_pay[:2]))
        rv = float(np.linalg.norm(self.vel - self.v_pay))
        near = (z_mouth - self.catch_z_tol <= self.p_pay[2] <= z_mouth + self.catch_z_tol
                and horiz <= self.funnel_eff_radius and rv <= self.v_retain)
        if self.contact_detect:
            # 真机：以"接触事件"（加速度尖峰 / 速度反转 / 外部开关）触发；超时兜底
            now = self.get_clock().now().nanoseconds * 1e-9
            dt = max(now - self._contact_last_t, 1e-3)
            az = (float(self.vel[2]) - self._contact_last_vz) / dt
            self._contact_last_t = now
            self._contact_last_vz = float(self.vel[2])
            rel_vz = float(self.v_pay[2] - self.vel[2])
            if near:
                if self._near_since is None:
                    self._near_since = now
            else:
                self._near_since = None
            hit = self.contact.update(now, az, rel_vz, near, external=self._ext_contact)
            self._ext_contact = False
            timeout = (self._near_since is not None
                       and (now - self._near_since) > self.contact_timeout_s)
            trigger = hit or (near and timeout)
            if hit:
                self.get_logger().warn('B: 检测到接触事件 → 捕获')
        else:
            trigger = near
        if trigger:
            self.caught = True
            self.phase = 'DONE'
            self.stack_hover = pos.copy()   # 锁定此刻位置为悬停点
            self._caught_t = self.get_clock().now().nanoseconds * 1e-9
            self.pub_caught.publish(Bool(data=True))
            if self._lock_req_pub is not None:
                lock_pose = self.pos_world + np.array([0.0, 0.0, -0.20])  # 漏斗口上方≈0.2m
                self._lock_req_pub.publish(Float64MultiArray(data=lock_pose.tolist()))
                self.get_logger().warn('B: 请求主动保持（在 B 漏斗处重生成并锁定）')
            self.get_logger().warn(
                f'*** STACK CAPTURED *** horiz={horiz:.3f}m rel_v={rv:.3f}m/s '
                f'z_mouth={z_mouth:.2f} p_B={pos.round(2)} p_p={self.p_pay.round(2)}')

    def control_stack(self, now):
        pos = self.pos_world
        # 协同握手：A 的释放 ack 是【原子事件】——只要收到就切 DIVE，
        # 不依赖当前瞬时对齐（释放/分离会让对齐短暂抖动，否则会漏掉 ack 卡在 ALIGN/FORMATION）。
        if (self.coord_mode == 'handshake' and self.phase in ('ALIGN', 'FORMATION')
                and self._release_cmd is not None and float(self._release_cmd[0]) >= 0.5
                and (now - self._release_cmd_t) < self.handshake_timeout):
            t_rel = float(self._release_cmd[1]) - self.clock_offset
            self.release_ref_t0 = t_rel
            self._dive_anchored = False
            self.get_logger().warn(
                f'B: 收到 A 释放 ack@{t_rel:.2f}s（原 phase={self.phase}）→ DIVE')
            self.phase = 'DIVE'
        # 释放取消（A 在 lead 窗口复核失败）：若在 DIVE 且尚未真正下潜 → 回到 ALIGN
        if (self.coord_mode == 'handshake' and self.phase == 'DIVE'
                and not self._dive_anchored and self._release_cmd is not None
                and float(self._release_cmd[0]) < 0.0
                and (now - self._release_cmd_t) < self.handshake_timeout):
            self.phase = 'ALIGN'
            self.align_t0 = now
            self._dive_anchored = False
            self.release_ref_t0 = None
            self.get_logger().warn('B: 收到释放取消 → 回到 ALIGN（重新对正）')
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
            # 2) 原地悬停/必要时下降，等 A 到位且比 B 高出 gap_eff=min_ab_gap+kσ，才开始横移
            tgt_alt = -standby[2]
            if self.a_est is not None:
                tgt_alt = min(tgt_alt, (-self.a_est[2]) - self._gap_eff())
            v = self.hover_velocity([pos[0], pos[1]], tgt_alt, kp_xy=1.0, max_speed=0.8,
                                    kp_z=1.4, max_climb=1.2)
            self.publish_velocity(v, yaw=self.yaw)
            if self.a_est is None:
                return
            a_alt_now = -self.a_est[2]
            a_vz = abs(float(self.a_vel_est[2])) if self.a_vel_est is not None else 9.9
            clear = a_alt_now - (-pos[2])
            if (a_alt_now >= a_alt - self.approach_alt_tol and a_vz < self.align_vel_tol
                    and clear >= self._gap_eff()):
                self.phase = 'TRANSLATE'
                self.get_logger().warn(
                    f'B: WAIT_A done (A_alt={a_alt_now:.2f}, clear={clear:.2f}) → TRANSLATE')
            return

        if self.phase == 'TRANSLATE':
            # 3) 保持高度平移到 A 正下方；全程比 A 低 gap_eff=min_ab_gap+kσ
            tgt_alt = -standby[2]
            if self.a_est is not None:
                tgt_alt = min(tgt_alt, (-self.a_est[2]) - self._gap_eff())
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
                tgt_alt = min(tgt_alt, (-self.a_est[2]) - self._gap_eff())
            # 意图升级：用 A 广播的【预测落点】作对正目标（含风漂移）；否则用 standby 并比 A 实际位置
            use_land = (self.use_intent and self._a_intent is not None
                        and len(self._a_intent) >= 8)
            tgt_xy = np.array(self._a_intent[6:8], float) if use_land else standby[:2]
            v = self.hover_velocity(tgt_xy, tgt_alt, kp_xy=1.2, max_speed=1.5,
                                    kp_z=1.4, max_climb=1.2)
            self.publish_velocity(v, yaw=self.yaw)
            if self.a_est is None:
                self.align_t0 = now
                return
            if use_land:
                rel_xy = float(np.linalg.norm(tgt_xy - pos[:2]))
            else:
                rel_xy = float(np.linalg.norm(self.a_est[:2] - pos[:2]))
            spd_xy = float(np.linalg.norm(self.vel[:2]))
            alt_ok = abs(-pos[2] - (-standby[2])) < self.align_alt_tol
            a_slow = (self.a_vel_est is None
                      or float(np.linalg.norm(self.a_vel_est)) < self.align_vel_tol)
            aligned = (rel_xy < self.align_xy_tol and spd_xy < self.align_vel_tol
                       and alt_ok and a_slow)
            if aligned:
                if self.coord_mode == 'handshake' and not self.use_formation:
                    # 持续报就绪（含质量指标）；一旦不再对齐会置 0，A 就不会释放。
                    # 编队模式不发就绪：ALIGN 只负责发 /formation/start，握手在 FORMATION 阶段。
                    self._pub_ready(now, 1.0, rel_xy, spd_xy)
                if self.align_t0 is None:
                    self.align_t0 = now
                elif (now - self.align_t0) >= self.align_hold_s:
                    if self.use_formation:
                        # 编队同速：广播 /formation/start，A 开始巡航、载荷挂载到 A
                        if not self._formation_sent:
                            self.pub_formation.publish(Bool(data=True))
                            self._formation_sent = True
                            self.form_t0 = None
                            self.form_ref_p0 = None      # 重置死推算参考
                            self._form_timeout_t0 = now  # 释放超时计时起点
                            self._last_good_t = now
                            self._aborted = False
                            self._dive_anchored = False
                            self.phase = 'FORMATION'
                            self.get_logger().warn(
                                f'B: ALIGNED rel_xy={rel_xy:.3f}m → /formation/start '
                                f'（编队同速 {self.formation_vel[:2]}）')
                    elif self.coord_mode == 'handshake':
                        # 等 A（释放权威）的 ack；a = [release(0/1), t_rel, stamp]
                        cmd = self._release_cmd
                        if (cmd is not None and float(cmd[0]) >= 0.5
                                and (now - self._release_cmd_t) < self.handshake_timeout):
                            t_rel = float(cmd[1]) - self.clock_offset   # A 时钟 → B 时钟
                            self.release_ref_t0 = t_rel
                            self._dive_anchored = False
                            self.phase = 'DIVE'
                            self.get_logger().warn(
                                f'B: 收到 A 释放 ack@{t_rel:.2f}s（rel_xy={rel_xy:.3f}）→ DIVE')
                    else:
                        rel_t = now + self.release_lead
                        self.pub_release_at.publish(Float64(data=rel_t))
                        self.release_ref_t0 = rel_t
                        self._dive_anchored = False
                        self.phase = 'DIVE'
                        self.get_logger().warn(
                            f'B: ALIGNED rel_xy={rel_xy:.3f}m spd_xy={spd_xy:.3f} → release@{rel_t:.2f}')
            else:
                self.align_t0 = now
                if self.coord_mode == 'handshake' and not self.use_formation:
                    self._pub_ready(now, 0.0, rel_xy, spd_xy)
            return

        if self.phase == 'FORMATION':
            # 释放超时保护：长时间对不齐 → 中止投放（不投），安全悬停→降落
            if (self._form_timeout_t0 is not None and not self._aborted
                    and (now - self._form_timeout_t0) > self.formation_timeout_s):
                self._abort_formation(now)
                return
            # 与 A 同向同速巡航：目标= A 投影点，速度前馈= A 速度；位置+速度都对正才释放
            tgt_alt = -standby[2]
            if self.a_est is not None:
                tgt_alt = min(tgt_alt, (-self.a_est[2]) - self._gap_eff())
            if self.a_est is None:
                v = self.hover_velocity(standby[:2], tgt_alt, kp_xy=1.2, max_speed=1.5,
                                        kp_z=1.4, max_climb=1.0)
                self.publish_velocity(v, yaw=self.yaw)
                return
            # 编队跟踪（优化）：用【已知的 formation_vel】做死推算参考，+
            # 对目标小幅校正；不再拿延迟/带噪的 A 速度估计做前馈（那会在高速下发散）。
            if self.form_ref_p0 is None:
                self.form_ref_p0 = pos[:2].copy()
                self.form_ref_t0 = now
                self._rel0 = ((self.a_est[:2] - pos[:2]).copy()
                              if self.a_est is not None else np.zeros(2))
            ref_xy = self.form_ref_p0 + self.formation_vel[:2] * (now - self.form_ref_t0)
            a_vxy = self.formation_vel[:2]          # 前馈 = 已知编队速度
            rel_err = ((self.a_est[:2] - pos[:2] - self._rel0)
                       if self.a_est is not None else np.zeros(2))
            vxy = a_vxy + self.stack_kp_xy * (ref_xy - pos[:2]) + self.form_kp_rel * rel_err
            vz = -self.stack_kp_z * (tgt_alt - (-pos[2]))
            v_sp = np.array([vxy[0], vxy[1], vz])
            n = float(np.linalg.norm(v_sp[:2]))
            if n > self.v_max:
                v_sp[:2] *= self.v_max / n
            v_sp[2] = float(np.clip(v_sp[2], -self.v_max, self.v_max))
            self.publish_velocity(v_sp, yaw=self.yaw)
            rel_xy = float(np.linalg.norm(self.a_est[:2] - pos[:2]))
            a_meas = self.a_vel_est[:2] if self.a_vel_est is not None else np.zeros(2)
            # 相对速度用【已知 formation_vel】比 B 自身速度（低噪、稳定）；
            # A 是否真的在动由下面的 a_spd（测量）把关。
            rel_vxy = float(np.linalg.norm(self.vel[:2] - self.formation_vel[:2]))
            alt_ok = abs(-pos[2] - (-standby[2])) < self.align_alt_tol
            a_spd = float(np.linalg.norm(a_meas))
            speed_ok = a_spd >= self.formation_min_speed_ratio * float(
                np.linalg.norm(self.formation_vel[:2]))
            aligned = (rel_xy < self.align_xy_tol and rel_vxy < self.align_vel_tol
                       and alt_ok and speed_ok)
            if aligned:
                self._last_good_t = now
                if self.coord_mode == 'handshake':
                    # 持续报就绪（质量=rel_xy / rel_vxy）；不对齐即置 0
                    self._pub_ready(now, 1.0, rel_xy, rel_vxy)
                if self.form_t0 is None:
                    self.form_t0 = now
                elif (now - self.form_t0) >= self.align_hold_s:
                    if self.coord_mode == 'handshake':
                        cmd = self._release_cmd
                        if (cmd is not None and float(cmd[0]) >= 0.5
                                and (now - self._release_cmd_t) < self.handshake_timeout):
                            t_rel = float(cmd[1]) - self.clock_offset
                            self.release_ref_t0 = t_rel
                            self._dive_anchored = False
                            self.phase = 'DIVE'
                            self.get_logger().warn(
                                f'B: FORMATION 收到 A 释放 ack@{t_rel:.2f}'
                                f'（rel_xy={rel_xy:.3f} rel_vxy={rel_vxy:.3f}）→ DIVE')
                    else:
                        rel_t = now + self.release_lead
                        self.pub_release_at.publish(Float64(data=rel_t))
                        self.release_ref_t0 = rel_t
                        self._dive_anchored = False
                        self.phase = 'DIVE'
                        self.get_logger().warn(
                            f'B: FORMATION aligned rel_xy={rel_xy:.3f}m rel_vxy={rel_vxy:.3f} '
                            f'a_spd={a_spd:.2f} → release@{rel_t:.2f}')
            else:
                # 容忍短暂抖动：只有连续失配超过 align_reset_tol 才重置保持计时
                # （否则噪声会不断重置 hold → 迟迟不释放、白飞很远）
                if self.coord_mode == 'handshake':
                    self._pub_ready(now, 0.0, rel_xy, rel_vxy)
                if (self.form_t0 is not None
                        and (now - self._last_good_t) > self.align_reset_tol):
                    self.form_t0 = None
            return

        if self.phase == 'DIVE':
            if self.release_ref_t0 is None:
                self.publish_velocity(np.zeros(3), yaw=self.yaw)
                return
            tl = now - self.release_ref_t0
            if tl < 0.0:
                # 释放前：原地悬停等（不能用 standby，编队模式下 B 已离开原点）
                v = self.hover_velocity(pos[:2], -pos[2], kp_xy=2.0,
                                        max_speed=2.0, kp_z=1.5, max_climb=1.5)
                self.publish_velocity(v, yaw=self.yaw)
                return
            # 分离有 ~0.1-0.2s 延迟：把 DIVE 参考起点重锚到载荷“真正开始下落”的时刻，
            # 否则 B 会比载荷早下潜，导致擦肩/落空。
            if not self._dive_anchored:
                falling = (self.v_pay is not None
                           and float(self.v_pay[2]) > self.dive_anchor_vz)
                if falling or tl > 0.5:
                    self.release_ref_t0 = now
                    self.stack_plan = None
                    self._a_adapt = None
                    self._dive_anchored = True
                    tl = 0.0
                    self.get_logger().warn(
                        f'B: DIVE 重锚（载荷开始下落 falling={falling}）')
                else:
                    # 分离延迟：载荷尚未真正下落 → 原地悬停等（不能用 standby）
                    v = self.hover_velocity(pos[:2], -pos[2], kp_xy=2.0,
                                            max_speed=2.0, kp_z=1.5, max_climb=1.5)
                    self.publish_velocity(v, yaw=self.yaw)
                    return
            if self.stack_plan is None:
                a_h = -self.a_est[2] if self.a_est is not None else -standby[2]
                # 载荷实际从 A 下方 offset 处释放；漏斗口在 B 机体上方 mount 处。
                # 让 plan 的有效 gap = 载荷→漏斗口的距离（_stack_ref 仍从 B 机体起步）。
                a_h = a_h - self.payload_release_offset - self.funnel_mount_height
                # 最小必要下潜：gap ≤ v_retain²/(2g) 时 a_dive=0（B 悬停接，下落时间最短、
                # 横风漂移最小）；否则取刚好使 v_rel≤v_retain 的最小 a_dive。
                if self.auto_min_dive:
                    gap_eff = a_h - (-pos[2])
                    v_ret = retain_speed(self.funnel_depth, self.funnel_restitution)
                    a_dive_use = minimal_dive(gap_eff, v_ret, 9.81,
                                              float(self.get_parameter('b_max_accel').value))
                else:
                    a_dive_use = self.a_dive
                self.stack_plan = plan_stack_drop(
                    a_height=a_h, b_height=-pos[2], a_dive=a_dive_use, g=9.81,
                    a_brake=self.a_brake, funnel_depth=self.funnel_depth,
                    restitution=self.funnel_restitution)
                self.get_logger().warn(
                    f'B: DIVE plan a_dive={a_dive_use:.2f}(auto={self.auto_min_dive}) '
                    f't_c={self.stack_plan.t_c:.3f}s v_rel={self.stack_plan.v_rel:.3f} '
                    f'v_retain={self.stack_plan.v_retain:.3f} feasible={self.stack_plan.feasible}')
                self._a_adapt = float(self.stack_plan.a_dive)
            xy_tgt = self.a_est[:2] if self.a_est is not None else pos[:2]
            vxy_ff = np.zeros(2)
            # 编队模式：物块就是从 A 释放的（无释放误差），直接跟踪 A 更稳
            # （避免跟踪带噪的物块估计引起横摆）；否则跟踪物块形成闭环纠正释放误差。
            if self.track_payload and not self.use_formation:
                pe = self._payload_est(now)     # 闭环：跟踪载荷本身（含测量噪声/延迟）
                if pe is not None:
                    xy_tgt = pe[0][:2]
                    vxy_ff = pe[1][:2]
            elif self.use_formation and self.a_vel_est is not None:
                vxy_ff = self.a_vel_est[:2]
            pr, vr, ar = _stack_ref(tl, self.stack_plan, (xy_tgt[0], xy_tgt[1]), 9.81)
            # 在线自适应下潜：用估计的载荷竖直状态滚动重解 a_dive（抗下击暴流等垂直扰动）
            if self.adaptive_dive and self._dive_anchored and self._a_adapt is not None:
                pe = self._payload_est(now) if self.track_payload else None
                if pe is not None:
                    p_z, v_z = float(pe[0][2]), float(pe[1][2])
                elif self.p_pay is not None:
                    p_z, v_z = float(self.p_pay[2]), float(self.v_pay[2])
                else:
                    p_z = None
                if p_z is not None:
                    a_target = _adaptive_dive(
                        p_z, v_z, float(pos[2]), float(self.vel[2]),
                        self.funnel_mount_height, 9.81,
                        float(self.get_parameter('b_max_accel').value), self.a_brake,
                        self.adaptive_alt_floor, self.v_retain)
                    slew = 20.0 / max(self.hz, 1e-6)
                    self._a_adapt = float(np.clip(
                        a_target, self._a_adapt - slew, self._a_adapt + slew))
                    look = 4.0 / max(self.hz, 1e-6)
                    pr = np.array(pr, float, copy=True)
                    vr = np.array(vr, float, copy=True)
                    ar = np.array(ar, float, copy=True)
                    pr[2] = pos[2] + self.vel[2] * look + 0.5 * self._a_adapt * look * look
                    vr[2] = self.vel[2] + self._a_adapt * look
                    ar[2] = self._a_adapt
            v_sp = np.zeros(3)
            v_sp[:2] = vxy_ff + self.stack_kp_xy * (xy_tgt - pos[:2])
            # 终端导引(ZEM)：把水平目标投影到接触时刻，用零控脱靶修正（减终端 miss）
            if self.zem_gain > 0.0 and self.p_pay is not None and self.stack_plan is not None:
                t_rem = max(0.05, self.stack_plan.t_c - tl)
                p_c = (self.p_pay + self.v_pay * t_rem
                       + 0.5 * np.array([0.0, 0.0, 9.81]) * t_rem * t_rem)
                v_sp[:2] += self.zem_gain * (p_c[:2] - (pos[:2] + self.vel[:2] * t_rem)) / t_rem
            v_sp[2] = vr[2] + self.stack_kp_z * (pr[2] - pos[2])
            n = float(np.linalg.norm(v_sp[:2]))
            if n > self.v_max:
                v_sp[:2] *= self.v_max / n
            v_sp[2] = float(np.clip(v_sp[2], -self.v_max, self.v_max))
            # 加速度前馈：把参考加速度 ar 交给 PX4（velocity 模式下直接叠加，减跟踪滞后）
            acc_ff = self.a_ff_gain * ar if self.a_ff_gain != 0.0 else None
            self.publish_velocity(v_sp, yaw=self.yaw, acc_ff=acc_ff)
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
        # 安全层：悬停高度不得高于 A−(min_ab_gap+kσ)，保证绝不靠近 A
        if self.a_est is not None:
            tgt_alt = min(tgt_alt, (-self.a_est[2]) - self._gap_eff())
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

#!/usr/bin/env python3
"""stack_drop.py —— 垂直堆叠空投（M6）核心层：纯 Python，无 ROS / Gazebo 依赖。

场景（第一版最简形态）：
    A 悬停在 B 的【严格正上方】gap 米处，携载重物；两机水平速度均为 0、
    投影重合时释放。载荷做纯垂直抛体（继承 A 速度 = 0，故为自由落体）；
    B 在下方以 a_dive 【温和下潜】（软着陆），接触后以 a_brake 刹停悬停。

捕获判据（比固定 v_c 更严谨，来自刚性漏斗物理）：
    位置：载荷下落到漏斗口平面时，水平偏差 < (mouth_radius − object_radius)；
    速度：接触相对速度 ≤ v_retain，其中
          v_retain = sqrt(2·g·depth) / e      （e = 恢复系数）
          由“反弹高度 e²v²/(2g) ≤ 漏斗深度 depth”导出。

关键物理结论（见 MEMORY/README）：
    B 只能往下压（a_B < g），故纯垂直下落的接触相对速度下界为
        v_rel = sqrt( 2·(g − a_dive)·gap )
    —— 这是 B 全时以 a_dive 下潜时的值。下潜越猛 v_rel 越小，但 B 冲得越低、
    刹车余量越少；a_dive≈3 是“冲击 / 余量”的折中。

坐标：世界系 NED（z 向下为正）；高度 = -z。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .payload_model import PayloadModel, PayloadParams
from .payload_filter import BallisticKF, BallisticDragKF
from .perception import CameraModel

Vec3 = np.ndarray


# ------------------------------------------------------------------ 物理公式
def retain_speed(depth: float, restitution: float, g: float = 9.81) -> float:
    """刚性漏斗能“兜住”的最大接触（垂直）速度。"""
    if depth <= 0.0:
        return 0.0
    e = max(float(restitution), 1e-6)
    return math.sqrt(2.0 * g * depth) / e


def contact_rel_speed(gap: float, a_dive: float, g: float = 9.81) -> float:
    """纯垂直下落、B 全时以 a_dive 下潜时的接触相对速度下界。"""
    if a_dive >= g:
        return 0.0
    return math.sqrt(2.0 * (g - a_dive) * gap)


def minimal_dive(gap: float, v_retain: float, g: float = 9.81,
                 a_dive_max: float = 6.0) -> float:
    """使接触速度恰好不超过 v_retain 的**最小**下潜加速度（风下应尽量小）。

    由 v_rel = √(2(g−a_dive)gap) ≤ v_retain 得
        a_dive ≥ g − v_retain²/(2·gap)
    取 max(0,·) 并封顶 a_dive_max。gap ≤ v_retain²/(2g) 时返回 0（无需下潜，B 悬停即可）——
    因为下潜会拉长下落时间、在横风下增大漂移（见 report/robust_geometry_and_retention.md）。
    """
    if gap <= 1e-9:
        return 0.0
    a_req = g - v_retain * v_retain / (2.0 * gap)
    return float(min(max(0.0, a_req), a_dive_max))


# ------------------------------------------------------------------ 规划
@dataclass
class StackPlan:
    feasible: bool = False
    reason: str = ''
    # 几何
    a_hover: Vec3 = field(default_factory=lambda: np.zeros(3))   # A 悬停点(NED)
    b_standby: Vec3 = field(default_factory=lambda: np.zeros(3))  # B 待命点(NED)
    gap: float = 0.0            # 释放时载荷到 B 的垂直间距 m
    a_dive: float = 0.0         # B 下潜加速度 m/s²（正=向下）
    a_brake: float = 0.0        # 接触后刹车加速度 m/s²
    # 时序 / 状态
    t_c: float = math.nan       # 释放到接触时长 s
    v_rel: float = math.nan     # 接触相对速度 m/s
    v_pay: float = math.nan     # 接触时载荷速度 m/s
    v_b: float = math.nan       # 接触时 B 速度 m/s
    z_c: float = math.nan       # 接触高度 m（离地）
    dive: float = math.nan      # B 下潜距离 m
    brake_dist: float = math.nan  # B 刹停距离 m
    post_brake_alt: float = math.nan  # 刹停后离地高度 m
    v_retain: float = math.nan  # 漏斗允许的最大接触速度 m/s


def plan_stack_drop(a_height: float, b_height: float,
                    a_dive: Optional[float] = 3.0,
                    g: float = 9.81, a_brake: float = 6.0,
                    ground_margin: float = 0.30,
                    funnel_depth: float = 0.30,
                    restitution: float = 0.60,
                    a_dive_max: float = 6.0,
                    a_xy: Tuple[float, float] = (0.0, 0.0),
                    b_xy: Tuple[float, float] = (0.0, 0.0)) -> StackPlan:
    """解析求出 B 温和下潜软捕获的标称时序与可行性。

    a_dive=None → 自动取【最小必要下潜】minimal_dive(gap, v_retain)（风下尽量不下潜）。
    """
    p = StackPlan()
    p.gap = float(a_height) - float(b_height)
    p.v_retain = retain_speed(funnel_depth, restitution, g)
    if a_dive is None:
        a_dive = minimal_dive(p.gap, p.v_retain, g, a_dive_max)
    p.a_dive = float(a_dive)
    p.a_brake = float(a_brake)
    p.a_hover = np.array([a_xy[0], a_xy[1], -float(a_height)])
    p.b_standby = np.array([b_xy[0], b_xy[1], -float(b_height)])
    if p.gap <= 1e-9:
        p.reason = f'gap {p.gap:.3f} <= 0（A 必须在 B 上方）'
        return p
    if a_dive < 0.0:
        p.reason = 'a_dive < 0（B 不能爬升去迎载荷）'
        return p
    if a_dive >= g:
        p.reason = f'a_dive {a_dive:.2f} >= g（B 无法追上自由落体）'
        return p
    p.t_c = math.sqrt(2.0 * p.gap / (g - a_dive))
    p.v_rel = (g - a_dive) * p.t_c
    p.v_pay = g * p.t_c
    p.v_b = a_dive * p.t_c
    p.dive = 0.5 * a_dive * p.t_c * p.t_c
    p.z_c = float(b_height) - p.dive
    p.brake_dist = p.v_b * p.v_b / (2.0 * p.a_brake) if p.a_brake > 0 else math.inf
    p.post_brake_alt = p.z_c - p.brake_dist
    if p.z_c < ground_margin:
        p.reason = f'接触高度 {p.z_c:.2f} < 余量 {ground_margin:.2f}'
        return p
    if p.post_brake_alt < ground_margin:
        p.reason = (f'B 刹停后 {p.post_brake_alt:.2f} < 余量 {ground_margin:.2f}'
                    f'（需抬高 B 待命高度或增大 a_brake）')
        return p
    if p.v_rel > p.v_retain + 1e-9:
        p.reason = (f'接触相对速度 {p.v_rel:.2f} > 漏斗保持速度 {p.v_retain:.2f}'
                    f'（需加深漏斗 / 降低恢复系数 / 缩小 gap / 加大 a_dive）')
        return p
    p.feasible = True
    p.reason = 'ok'
    return p


# ------------------------------------------------------------------ 仿真
@dataclass
class StackResult:
    success: bool = False
    t_capture: float = math.nan
    miss_dist: float = math.inf
    rel_speed_at_capture: float = math.nan
    horiz_miss_at_capture: float = math.nan
    capture_margin: float = math.nan      # eff_r − 捕获时水平偏差（正=在口内有余量）
    peak_accel: float = 0.0
    peak_speed: float = 0.0
    b_min_alt: float = math.inf
    b_final_alt: float = math.nan
    t: List[float] = field(default_factory=list)
    b_pos: List[Vec3] = field(default_factory=list)
    p_pos: List[Vec3] = field(default_factory=list)
    rel_dist: List[float] = field(default_factory=list)


@dataclass
class StackNoise:
    release_pos_sigma: float = 0.0    # m，A 释放点水平误差
    b_pos_sigma: float = 0.0          # m，B 待命点误差
    rel_pos_sigma: float = 0.0        # m，相对定位（mesh/UWB）噪声
    rel_latency: float = 0.0          # s，相对定位延迟
    seed: int = 0


def _stack_ref(t: float, plan: StackPlan, p_xy: Tuple[float, float],
               g: float, v_xy: Tuple[float, float] = (0.0, 0.0)
               ) -> Tuple[Vec3, Vec3, Vec3]:
    """B 的参考 (p, v, a)；z 走“下潜→刹车→悬停”，xy 跟随载荷水平估计。

    v_xy: 载荷水平速度估计，作为 B 的水平速度前馈参考（消除追尾滞后）。
    """
    zb0 = plan.b_standby[2]
    if t <= plan.t_c:
        z = zb0 + 0.5 * plan.a_dive * t * t
        vz = plan.a_dive * t
        az = plan.a_dive
    else:
        tb = t - plan.t_c
        zc = zb0 + 0.5 * plan.a_dive * plan.t_c * plan.t_c
        vb = plan.a_dive * plan.t_c
        if vb - plan.a_brake * tb > 0.0:
            z = zc + vb * tb - 0.5 * plan.a_brake * tb * tb
            vz = vb - plan.a_brake * tb
            az = -plan.a_brake
        else:
            z = zc + vb * vb / (2.0 * plan.a_brake)
            vz = 0.0
            az = 0.0
    pr = np.array([p_xy[0], p_xy[1], z], float)
    vr = np.array([v_xy[0], v_xy[1], vz], float)
    ar = np.array([0.0, 0.0, az], float)
    return pr, vr, ar


def _adaptive_dive(z_p: float, v_pz: float, z_b: float, v_bz: float,
                   mount_h: float, g: float, a_max: float, a_brake: float,
                   alt_floor: float, v_retain: float) -> float:
    """滚动重解 B 的下潜加速度（1D 垂直会合，世界系 NED，z 向下为正）。

    接触条件：载荷落到漏斗口平面 z_p = z_b − mount_h。令相对间隙
        r(s) = (z_p − z_b + mount_h) + (v_pz − v_bz)·s + ½(g − a_b)·s²
    求最小正根 s*（P载荷到达口平面），并在候选 a_b∈[0,a_max] 中选使
        margin = min(v_retain − v_rel, 刹车后离地 − alt_floor)
    最大者。用于抗下击暴流（载荷下落更快）等垂直扰动。
    """
    r0 = z_p - z_b + mount_h
    vr0 = v_pz - v_bz
    best_a, best_m = 0.0, -1e9
    for a_b in np.linspace(0.0, a_max, 31):
        A = 0.5 * (g - a_b)
        if A <= 1e-9:
            continue
        disc = vr0 * vr0 - 4.0 * A * r0
        if disc < 0.0:
            continue
        sq = math.sqrt(disc)
        cand = [s for s in ((-vr0 + sq) / (2 * A), (-vr0 - sq) / (2 * A)) if s > 1e-6]
        if not cand:
            continue
        s = min(cand)
        v_b = v_bz + a_b * s
        if v_b < 0.0:                    # B 不应已向上运动
            continue
        v_p = v_pz + g * s
        v_rel = abs(v_p - v_b)
        z_contact = z_b + v_bz * s + 0.5 * a_b * s * s
        brake_alt = -(z_contact + v_b * v_b / (2.0 * a_brake))
        m = min(v_retain - v_rel, brake_alt - alt_floor)
        if m > best_m:
            best_m, best_a = m, a_b
    return best_a


def _horiz_drift(w_h: float, drag_mode: str, k: float, tau: float,
                 n: int = 400) -> float:
    """单位：单轴水平风 w_h 下、从 v=0 开始、历时 tau 的漂移量（数值积分）。

    linear    : a = −k(v−w)
    quadratic : a = −k·|v−w|·(v−w)
    """
    if tau <= 0.0:
        return 0.0
    v = 0.0
    x = 0.0
    h = tau / n
    for _ in range(n):
        if drag_mode == 'quadratic':
            a = -k * abs(v - w_h) * (v - w_h)
        elif drag_mode == 'linear':
            a = -k * (v - w_h)
        else:
            a = 0.0
        v += a * h
        x += v * h
    return x


def _gust_wind(gust: Dict, t: float, base: Vec3) -> Vec3:
    """阵风（时变风）模型：在常值风 base 上叠加一个单轴扰动。

    gust 字段：type('sin'|'step'|'ramp')、amp、period、t_start、axis(0/1/2)。
    """
    w = np.asarray(base, float).reshape(3).copy()
    amp = float(gust.get('amp', 0.0))
    t0 = float(gust.get('t_start', 0.0))
    kind = str(gust.get('type', 'sin'))
    ax = int(gust.get('axis', 0))
    if t < t0:
        v = 0.0
    elif kind == 'step':
        v = amp
    elif kind == 'ramp':
        v = amp
    else:  # sin
        T = max(float(gust.get('period', 0.5)), 1e-6)
        v = amp * math.sin(2.0 * math.pi * (t - t0) / T)
    w[ax] += v
    return w


def simulate_stack(defaults: Dict, layout: Dict, scenario: Dict,
                   plan: Optional[StackPlan] = None,
                   noise: Optional[StackNoise] = None,
                   kp: float = 9.0, kd: float = 6.0,
                   lead: float = 0.0,
                   vel_ff: bool = True,
                   vel_alpha: float = 0.3,
                   meas_lpf_alpha: float = 1.0,
                   est_mode: str = 'raw',
                   kf_q: float = 2.0, kf_sigma: float = 0.05,
                   kf_drag_k: Optional[float] = None, kf_qw: float = 0.05,
                   zem_gain: float = 0.0, zem_lead: float = 1.0,
                   vert_mode: str = 'open',
                   vert_margin: float = 0.50
                   ) -> Tuple[StackResult, StackPlan]:
    """跑一次垂直堆叠投放，返回 (指标, 规划)。

    干扰鲁棒（默认开启）：
      · 速度前馈：B 的水平速度参考 = 载荷水平速度估计 v̂（消除追尾滞后）；
      · 预测式对正 lead：B 的水平位置目标 = 测量位置 + lead·v̂·剩余时间，
        去"拦截"侧风漂移的载荷，而不是"追尾"。
    lead: 预测对正增益（0=纯速度前馈追尾；1~1.5 为推荐拦截档）。
    vel_ff: 是否把载荷水平速度估计作为 B 的速度前馈（False=原始追尾+零速
        阻尼，即未优化的基线；True=消除追尾滞后）。
    vel_alpha: 有限差分速度估计的 EMA 平滑系数。
    meas_lpf_alpha: 相对定位测量一阶低通（EMA）系数；1.0=不滤波，
        0.3≈SITL 实测值。重噪声下必须滤波，否则噪声直接驱动 B。
    est_mode: 载荷状态估计方式。'raw'=测量直接用（有限差分速度）；
        'kf'=BallisticKF（纯弹道模型）；'windkf'=BallisticDragKF
        （含已知线性阻力 + 估计常值风，消除模型失配滞后，推荐）。
    kf_q / kf_sigma: KF 过程噪声 std (m/s²) / 测量噪声 std (m)。
    vert_mode: 垂直控制。'open'=固定规划下潜剖面；'adaptive'=滚动重解；
        'minimal'=最小必要下潜 max(0, g−v_retain²/2gap)，横风下尽量不潜；
        下潜加速度（抗下击暴流等垂直扰动）。
    vert_margin: adaptive 模式下刹车后高度相对 ground_margin 额外留的余量 (m)。
    """
    noise = noise or StackNoise()
    rng = np.random.default_rng(noise.seed)

    g = float(defaults['g'])
    hz = float(defaults['control_hz'])
    dt = 1.0 / hz
    dur = float(defaults.get('duration_s', 20.0))
    pcfg = {**defaults.get('planner', {}), **scenario.get('planner', {})}
    bcfg = {**defaults['drone_b'], **scenario.get('drone_b', {})}
    ccfg = {**defaults['capture'], **scenario.get('capture', {})}
    funnel = {**defaults['capture'].get('funnel', {}), **ccfg.get('funnel', {})}
    stk = {**defaults.get('stack', {}), **scenario.get('stack', {})}
    pay = {**defaults['payload'], **scenario.get('payload', {})}
    vert_mode = str(scenario.get('vert_mode', vert_mode))   # 允许 scenario 覆盖
    # 视觉感知（可选）：用相机模型替换"真值+高斯噪声"的载荷测量
    perc = scenario.get('perception')
    cam = None
    if perc:
        cam = CameraModel(fov_deg=float(perc.get('fov_deg', 60.0)),
                          lateral_k=float(perc.get('lateral_k', 0.005)),
                          depth_sigma=float(perc.get('depth_sigma', 0.02)),
                          dropout=float(perc.get('dropout', 0.0)),
                          boresight=perc.get('boresight', (0.0, 0.0, -1.0)),
                          rng=np.random.default_rng(noise.seed + 7))

    a_init = np.asarray(scenario.get('a_init', layout.get('a_init')), float).reshape(3)
    b0 = np.asarray(scenario.get('b_standby', layout['b_standby']), float).reshape(3)
    if noise.b_pos_sigma > 0:
        b0 = b0 + rng.normal(0.0, noise.b_pos_sigma, 3)

    a_dive = float(stk.get('a_dive', 3.0))
    a_brake = float(stk.get('a_brake', bcfg['max_accel']))
    mouth_r = float(funnel.get('mouth_radius', 0.20))
    depth = float(funnel.get('depth', 0.30))
    rest = float(funnel.get('restitution', 0.60))
    # 漏斗口平面在 B 机体中心上方 mount_height；载荷有体积，须留 object_radius 余量
    mount_h = float(funnel.get('mount_height', 0.10))
    obj_r = float(funnel.get('object_radius', 0.05))
    # 末端机械臂：reach 扩展口内有效半径；吸收提升等效保持速度
    arm_reach = float(ccfg.get('arm_reach', 0.0))
    arm_absorb = float(ccfg.get('arm_absorb', 0.0))
    eff_r = max(0.0, mouth_r - obj_r) + arm_reach

    if plan is None:
        plan = plan_stack_drop(a_height=-a_init[2], b_height=-b0[2],
                               a_dive=(None if vert_mode == 'minimal' else a_dive),
                               g=g, a_brake=a_brake,
                               a_dive_max=float(bcfg['max_accel']),
                               ground_margin=float(pcfg.get('ground_margin', 0.30)),
                               funnel_depth=depth, restitution=rest,
                               a_xy=(a_init[0], a_init[1]), b_xy=(b0[0], b0[1]))
    if not plan.feasible:
        # 机械臂吸收可把"接触速度超 v_retain"从不可行变可行
        if arm_absorb > 0.0 and plan.v_rel == plan.v_rel \
                and plan.v_rel <= plan.v_retain + arm_absorb + 1e-9:
            plan.feasible = True
            plan.reason = 'ok (arm_absorb)'
        else:
            return StackResult(success=False), plan
    v_retain_eff = plan.v_retain + arm_absorb

    base_wind = np.asarray(scenario.get('wind', defaults.get('wind', (0, 0, 0))), float)
    gust = scenario.get('gust')
    payload = PayloadModel(PayloadParams(
        mass=float(pay['mass']), gravity=g,
        drag_mode=str(pay.get('drag_mode', 'none')),
        drag_k=float(pay.get('drag_k', 0.0)),
        wind=tuple(base_wind)))

    # A 端迎风预补偿量（需 plan.t_c；plan 已就绪）
    a_comp = np.zeros(2)
    _comp = scenario.get('a_wind_comp')
    if _comp:
        frac = 1.0 if _comp is True else float(_comp)   # 可为风估计误差比例
        tau = plan.t_c if plan.t_c == plan.t_c else 0.4
        dm = str(pay.get('drag_mode', 'none'))
        kk = float(pay.get('drag_k', 0.0))
        a_comp = frac * np.array([-_horiz_drift(float(base_wind[0]), dm, kk, tau),
                                  -_horiz_drift(float(base_wind[1]), dm, kk, tau)])

    # 编队同速投放（可选）：载荷继承 A 的水平速度 v_release；B 以 b_v0 初速跟飞。
    v_release = np.asarray(scenario.get('v_release', (0.0, 0.0, 0.0)), float).reshape(3)
    b_v0 = np.asarray(scenario.get('b_v0', (0.0, 0.0, 0.0)), float).reshape(3)

    p_b = b0.copy()
    v_b = b_v0.copy()
    res = StackResult()
    peak_a = peak_v = 0.0
    lat_steps = int(round(noise.rel_latency / dt))
    p_hist: List[Vec3] = []                      # 载荷真实位置历史（供测量）
    v_xy_est = np.zeros(2)                       # 载荷水平速度估计（raw 模式）
    p_filt: Optional[np.ndarray] = None          # 测量 EMA 滤波状态（raw 模式）
    p_meas_prev: Optional[Vec3] = None
    if est_mode == 'windkf':
        kf_k = float(pay.get('drag_k', 0.0)) if kf_drag_k is None else float(kf_drag_k)
        kf = BallisticDragKF(g=g, drag_k=kf_k, q_accel=kf_q, q_wind=kf_qw,
                             meas_sigma=kf_sigma)
    elif est_mode == 'kf':
        kf = BallisticKF(g=g, q_accel=kf_q, meas_sigma=kf_sigma)
    else:
        kf = None
    a_adapt = a_dive                            # adaptive 模式下的当前下潜加速度
    captured = False
    p_p = a_init.copy()
    v_p = np.zeros(3)
    released = False
    t = 0.0
    n_steps = int(math.ceil(dur / dt))

    for _ in range(n_steps):
        if not released and t >= 0.0:
            pr = a_init.copy()
            pr[:2] += a_comp                       # A 端迎风预补偿
            if noise.release_pos_sigma > 0:
                pr[:2] += rng.normal(0.0, noise.release_pos_sigma, 2)
            payload.release(pr, v_release)
            released = True
        if released:
            p_p, v_p = payload.pos.copy(), payload.vel.copy()
            p_hist.append(p_p.copy())  # noqa: E501

        # 相对定位：3D 位置测量（延迟 + 噪声）
        p_hat = np.zeros(3)                       # 载荷状态估计（位置）
        v_hat = np.zeros(3)                       # 载荷状态估计（速度）
        if released and p_hist:
            j = max(0, len(p_hist) - 1 - lat_steps)
            z_meas = p_hist[j].copy()
            if cam is not None:
                # 相机感知：FOV 门控 + 距离相关误差 + 丢帧。
                # 无效时保持上一帧；**从未捕获 → 无信息**（B 按"载荷在自己位置"行动，会漏接）。
                z_meas = cam.track(p_hist[j], p_b)
                z_meas = p_b.copy() if z_meas is None else z_meas.copy()
            elif noise.rel_pos_sigma > 0:
                # raw 模式只对 xy 加噪（与旧行为/随机流一致）；kf 需 3D 观测。
                if est_mode == 'kf':
                    z_meas = z_meas + rng.normal(0.0, noise.rel_pos_sigma, 3)
                else:
                    z_meas[:2] += rng.normal(0.0, noise.rel_pos_sigma, 2)
            t_meas = t - lat_steps * dt
        else:
            z_meas = p_b.copy()
            t_meas = t

        p_contact_pred = None                     # 预测接触时刻的载荷位置
        if est_mode in ('kf', 'windkf'):
            kf.process(z_meas, t_meas)
            est = kf.estimate_at(t)
            est_c = kf.estimate_at(max(t, plan.t_c))
            if est is not None:
                p_hat, v_hat = est[0], est[1]
                if est_c is not None:
                    p_contact_pred = est_c[0]
        else:
            # raw：位置用测量（可选 EMA），速度用有限差分 + EMA
            if meas_lpf_alpha < 1.0:
                if p_filt is None:
                    p_filt = z_meas.copy()
                else:
                    p_filt = p_filt + meas_lpf_alpha * (z_meas - p_filt)
                p_hat = p_filt.copy()
            else:
                p_hat = z_meas.copy()
            if released and p_meas_prev is not None:
                dv = (p_hat - p_meas_prev) / dt
                v_xy_est += vel_alpha * (dv[:2] - v_xy_est)
                v_hat = np.array([v_xy_est[0], v_xy_est[1], dv[2]])
            p_meas_prev = p_hat.copy()

        # 水平目标：预测式对正（KF 直接给接触时刻预测；raw 用 v̂ 外推）
        if lead > 0.0 and released:
            if p_contact_pred is not None:
                p_xy = (float(p_contact_pred[0]), float(p_contact_pred[1]))
            else:
                t_to_contact = max(0.0, plan.t_c - t)
                p_xy = (p_hat[0] + lead * v_hat[0] * t_to_contact,
                        p_hat[1] + lead * v_hat[1] * t_to_contact)
        else:
            p_xy = (float(p_hat[0]), float(p_hat[1]))

        vr_xy = (v_hat[0], v_hat[1]) if vel_ff else (0.0, 0.0)
        pr, vr, ar = _stack_ref(t, plan, p_xy, g, vr_xy)
        # 垂直：自适应下潜（可选）——用估计的载荷竖直状态重解下潜加速度，
        # 采用"后退视野"：从当前状态出发用 a_adapt 前推一个小视界做参考。
        if vert_mode == 'adaptive' and released:
            a_target = _adaptive_dive(p_hat[2], v_hat[2], p_b[2], v_b[2], mount_h,
                                      g, float(bcfg['max_accel']), a_brake,
                                      float(pcfg.get('ground_margin', 0.30)) + vert_margin,
                                      plan.v_retain)
            slew = 20.0 * dt
            a_adapt = float(np.clip(a_target, a_adapt - slew, a_adapt + slew))
            look = 4.0 * dt
            pr = np.array([pr[0], pr[1],
                           p_b[2] + v_b[2] * look + 0.5 * a_adapt * look * look])
            vr = np.array([vr[0], vr[1], v_b[2] + a_adapt * look])
            ar = np.array([ar[0], ar[1], a_adapt])
        a_cmd = ar + kp * (pr - p_b) + kd * (vr - v_b)
        # 终端导引（ZEM/零控脱靶）：预测接触时刻载荷与 B“滑翔”位置之差，按 1/τ² 修正。
        # 额外用上 B 自身速度分量，减终端 miss。
        if zem_gain > 0.0 and released:
            t_rem = max(0.05, plan.t_c - t)
            p_c_pred = p_hat + zem_lead * v_hat * t_rem \
                + 0.5 * np.array([0.0, 0.0, g]) * t_rem * t_rem
            zem = p_c_pred - (p_b + v_b * t_rem)
            a_cmd = a_cmd + zem_gain * zem / (t_rem * t_rem)
        # 只允许向下下潜/刹车，不允许 B 主动爬升去迎载荷（会破坏“正下方”）
        a_norm = float(np.linalg.norm(a_cmd))
        if a_norm > float(bcfg['max_accel']):
            a_cmd = a_cmd * (float(bcfg['max_accel']) / a_norm)
        v_b = v_b + a_cmd * dt
        vn = float(np.linalg.norm(v_b))
        if vn > float(bcfg['max_speed']):
            v_b = v_b * (float(bcfg['max_speed']) / vn)
        p_b = p_b + v_b * dt
        peak_a = max(peak_a, float(np.linalg.norm(a_cmd)))
        peak_v = max(peak_v, float(np.linalg.norm(v_b)))
        res.b_min_alt = min(res.b_min_alt, -p_b[2])

        if released:
            if gust is not None:
                payload.wind = _gust_wind(gust, t, base_wind)   # 时变风（阵风）
            payload.step(dt)
            p_p, v_p = payload.pos.copy(), payload.vel.copy()

        rel = float(np.linalg.norm(p_b - p_p))
        res.t.append(t)
        res.b_pos.append(p_b.copy())
        res.p_pos.append(p_p.copy())
        res.rel_dist.append(rel)

        if released and not captured:
            rel_v = float(np.linalg.norm(v_b - v_p))
            z_mouth = p_b[2] - mount_h          # NED：z 越小越高，口平面在 B 中心上方
            horiz = float(np.linalg.norm(p_b[:2] - p_p[:2]))
            # 接触 = 载荷下落到漏斗口平面之内，水平偏差够小、且速度可被兜住
            if (p_p[2] >= z_mouth and horiz <= eff_r
                    and rel_v <= v_retain_eff + 1e-9):
                captured = True
                res.success = True
                res.t_capture = t
                res.rel_speed_at_capture = rel_v
                res.horiz_miss_at_capture = horiz
                res.capture_margin = eff_r - horiz
                break
        # 载荷落地则失败退出
        if released and -p_p[2] < 0.0:
            break
        t += dt

    res.miss_dist = float(min(res.rel_dist)) if res.rel_dist else math.inf
    res.peak_accel = peak_a
    res.peak_speed = peak_v
    res.b_final_alt = float(-p_b[2])
    return res, plan


if __name__ == '__main__':
    p = plan_stack_drop(a_height=4.5, b_height=3.5, a_dive=3.0)
    print('plan:', p.feasible, p.reason)
    print(f'  gap={p.gap:.2f} a_dive={p.a_dive} t_c={p.t_c:.3f}s v_rel={p.v_rel:.3f} '
          f'接触高度={p.z_c:.2f}m 刹停后={p.post_brake_alt:.2f}m v_retain={p.v_retain:.3f}')
    defaults = {'g': 9.81, 'control_hz': 50.0, 'duration_s': 10.0,
                'planner': {'ground_margin': 0.30},
                'drone_b': {'max_speed': 5.0, 'max_accel': 6.0},
                'capture': {'radius': 0.30, 'rel_speed': 1.5,
                            'funnel': {'mouth_radius': 0.20, 'depth': 0.30, 'restitution': 0.60,
                                       'mount_height': 0.10, 'object_radius': 0.05,
                                       'mount_height': 0.10, 'object_radius': 0.05}},
                'payload': {'mass': 0.30, 'drag_mode': 'none', 'drag_k': 0.0},
                'stack': {'a_dive': 3.0, 'a_brake': 6.0}}
    layout = {'a_init': [0, 0, -4.5], 'b_standby': [0, 0, -3.5]}
    r, _ = simulate_stack(defaults, layout, {})
    print('sim:', r.success, f't_cap={r.t_capture:.3f}s miss={r.miss_dist:.3f}m '
          f'rel_v={r.rel_speed_at_capture:.3f} B最低={r.b_min_alt:.2f}m')

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


def plan_stack_drop(a_height: float, b_height: float, a_dive: float,
                    g: float = 9.81, a_brake: float = 6.0,
                    ground_margin: float = 0.30,
                    funnel_depth: float = 0.30,
                    restitution: float = 0.60,
                    a_xy: Tuple[float, float] = (0.0, 0.0),
                    b_xy: Tuple[float, float] = (0.0, 0.0)) -> StackPlan:
    """解析求出 B 温和下潜软捕获的标称时序与可行性。"""
    p = StackPlan()
    p.gap = float(a_height) - float(b_height)
    p.a_dive = float(a_dive)
    p.a_brake = float(a_brake)
    p.a_hover = np.array([a_xy[0], a_xy[1], -float(a_height)])
    p.b_standby = np.array([b_xy[0], b_xy[1], -float(b_height)])
    p.v_retain = retain_speed(funnel_depth, restitution, g)
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
               g: float) -> Tuple[Vec3, Vec3, Vec3]:
    """B 的参考 (p, v, a)；z 走“下潜→刹车→悬停”，xy 跟随载荷水平估计。"""
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
    vr = np.array([0.0, 0.0, vz], float)
    ar = np.array([0.0, 0.0, az], float)
    return pr, vr, ar


def simulate_stack(defaults: Dict, layout: Dict, scenario: Dict,
                   plan: Optional[StackPlan] = None,
                   noise: Optional[StackNoise] = None,
                   kp: float = 9.0, kd: float = 6.0
                   ) -> Tuple[StackResult, StackPlan]:
    """跑一次垂直堆叠投放，返回 (指标, 规划)。"""
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
    eff_r = max(0.0, mouth_r - obj_r)

    if plan is None:
        plan = plan_stack_drop(a_height=-a_init[2], b_height=-b0[2],
                               a_dive=a_dive, g=g, a_brake=a_brake,
                               ground_margin=float(pcfg.get('ground_margin', 0.30)),
                               funnel_depth=depth, restitution=rest,
                               a_xy=(a_init[0], a_init[1]), b_xy=(b0[0], b0[1]))
    if not plan.feasible:
        return StackResult(success=False), plan

    payload = PayloadModel(PayloadParams(
        mass=float(pay['mass']), gravity=g,
        drag_mode=str(pay.get('drag_mode', 'none')),
        drag_k=float(pay.get('drag_k', 0.0)),
        wind=tuple(scenario.get('wind', defaults.get('wind', (0, 0, 0))))))

    p_b = b0.copy()
    v_b = np.zeros(3)
    res = StackResult()
    peak_a = peak_v = 0.0
    lat_steps = int(round(noise.rel_latency / dt))
    xy_hist: List[Tuple[float, float]] = []
    captured = False
    p_p = a_init.copy()
    v_p = np.zeros(3)
    released = False
    t = 0.0
    n_steps = int(math.ceil(dur / dt))

    for _ in range(n_steps):
        if not released and t >= 0.0:
            pr = a_init.copy()
            if noise.release_pos_sigma > 0:
                pr[:2] += rng.normal(0.0, noise.release_pos_sigma, 2)
            payload.release(pr, np.zeros(3))
            released = True
        if released:
            p_p, v_p = payload.pos.copy(), payload.vel.copy()
            xy_hist.append((p_p[0], p_p[1]))

        # 相对定位（含延迟/噪声）：B 用它对水平方向做正上方闭环
        if released and xy_hist:
            j = max(0, len(xy_hist) - 1 - lat_steps)
            mx, my = xy_hist[j]
            if noise.rel_pos_sigma > 0:
                mx += rng.normal(0.0, noise.rel_pos_sigma)
                my += rng.normal(0.0, noise.rel_pos_sigma)
            p_xy = (mx, my)
        else:
            p_xy = (p_b[0], p_b[1])

        pr, vr, ar = _stack_ref(t, plan, p_xy, g)
        a_cmd = ar + kp * (pr - p_b) + kd * (vr - v_b)
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
                    and rel_v <= plan.v_retain + 1e-9):
                captured = True
                res.success = True
                res.t_capture = t
                res.rel_speed_at_capture = rel_v
                res.horiz_miss_at_capture = horiz
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

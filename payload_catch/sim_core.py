#!/usr/bin/env python3
"""离线闭环仿真 —— 纯 Python，无 ROS / Gazebo 依赖。

用途（M1）：在接 PX4 SITL 之前，先把
    规划（rendezvous）→ B 跟踪会合 → 载荷自由落体 → 捕获判定
整条链路跑通并量化。坐标与世界系 NED 一致（z 向下为正）。

模型：
  · A 定点悬停（速度 0），在 t_r 释放载荷；载荷初速 = A 速度。
  · 载荷 = PayloadModel（第一版无阻力）。
  · B = 双积分器 ẍ=a，|a|≤a_max；跟踪规划参考：
        a_cmd = a_ref(t) + Kp·(p_ref−p_B) + Kd·(v_ref−v_B)，再限幅。
  · 捕获判定：|p_B−p_p| < r_c 且 |v_B−v_p| < v_c。

为后续鲁棒性预留：释放点/释放速度噪声、B 初值噪声、载荷状态测量噪声、
控制周期。M1 全部置 0，得到"标称必中"的基线。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .payload_model import PayloadModel, PayloadParams
from .rendezvous import HoverReleasePlanner, PlanResult

Vec3 = np.ndarray


# ------------------------------------------------------------------- config
@dataclass
class SimNoise:
    """噪声/扰动（M1 全 0）。"""
    release_pos_sigma: float = 0.0     # m，释放点位置噪声
    release_vel_sigma: float = 0.0     # m/s，释放速度噪声
    b_pos_sigma: float = 0.0           # m，B 初始位置噪声
    payload_meas_sigma: float = 0.0    # m，载荷状态测量噪声（给 B 用）
    seed: int = 0


@dataclass
class SimResult:
    success: bool = False
    t_capture: float = math.nan
    miss_dist: float = math.inf        # 全程 |p_B−p_p| 最小值
    rel_speed_at_capture: float = math.nan
    peak_accel: float = 0.0
    peak_speed: float = 0.0
    final_b: Vec3 = field(default_factory=lambda: np.zeros(3))
    final_p: Vec3 = field(default_factory=lambda: np.zeros(3))
    note: str = ''
    # 时间序列（画图/诊断）
    t: List[float] = field(default_factory=list)
    b_pos: List[Vec3] = field(default_factory=list)
    b_vel: List[Vec3] = field(default_factory=list)
    p_pos: List[Vec3] = field(default_factory=list)
    p_vel: List[Vec3] = field(default_factory=list)
    rel_dist: List[float] = field(default_factory=list)


# ------------------------------------------------------------------ helpers
def _interp_traj(ts: np.ndarray, xs: np.ndarray, t: float) -> Vec3:
    """在参考轨迹 (ts, xs[N,3]) 上线性插值（越界则取端点）。"""
    if t <= ts[0]:
        return xs[0].copy()
    if t >= ts[-1]:
        return xs[-1].copy()
    i = int(np.searchsorted(ts, t, side='right')) - 1
    i = max(0, min(i, len(ts) - 2))
    a = (t - ts[i]) / max(ts[i + 1] - ts[i], 1e-12)
    return xs[i] * (1.0 - a) + xs[i + 1] * a


def simulate(defaults: Dict, layout: Dict, scenario: Dict,
             plan: Optional[PlanResult] = None,
             noise: Optional[SimNoise] = None,
             kp: float = 9.0, kd: float = 6.0) -> Tuple[SimResult, PlanResult]:
    """跑一次完整任务，返回 (指标, 规划方案)。"""
    noise = noise or SimNoise()
    rng = np.random.default_rng(noise.seed)

    g = float(defaults['g'])
    hz = float(defaults['control_hz'])
    dt = 1.0 / hz
    dur = float(defaults.get('duration_s', 20.0))
    pcfg = defaults['planner']
    bcfg = defaults['drone_b']
    ccfg = defaults['capture']
    pay = defaults['payload']

    p_a = np.asarray(layout['a_hover'], float).reshape(3)
    p_b0 = np.asarray(layout['b_standby'], float).reshape(3)
    if noise.b_pos_sigma > 0:
        p_b0 = p_b0 + rng.normal(0.0, noise.b_pos_sigma, 3)
    v_b0 = np.zeros(3)

    # ── 规划 ─────────────────────────────────────────────────────────────
    if plan is None:
        planner = HoverReleasePlanner(
            g=g, b_max_speed=float(bcfg['max_speed']),
            b_max_accel=float(bcfg['max_accel']),
            capture_radius=float(ccfg['radius']),
            capture_rel_speed=float(ccfg['rel_speed']),
            w_time=float(pcfg['w_time']), w_accel=float(pcfg['w_accel']),
            w_vel=float(pcfg['w_vel']), ground_margin=float(pcfg['ground_margin']))
        plan = planner.solve(
            p_a, p_b0, v_b0,
            catch_alt_range=(pcfg['catch_alt_min'], pcfg['catch_alt_max']),
            release_delay_range=(pcfg['release_delay_min'], pcfg['release_delay_max']),
            catch_alt_step=pcfg['catch_alt_step'],
            release_delay_step=pcfg['release_delay_step'])
    if not plan.feasible:
        return SimResult(success=False, note=f'规划不可行: {plan.reason}'), plan

    # 规划参考（t ∈ [0, t_c]）
    ref_t, ref_p, ref_v, ref_a = HoverReleasePlanner.resample(plan, n=401)

    # ── 载荷 ─────────────────────────────────────────────────────────────
    payload_params = PayloadParams(
        mass=float(pay['mass']), gravity=g,
        drag_mode=str(pay.get('drag_mode', 'none')),
        drag_k=float(pay.get('drag_k', 0.0)),
        wind=tuple(scenario.get('wind', defaults.get('wind', (0, 0, 0)))))
    payload = PayloadModel(payload_params)

    # ── 状态 ─────────────────────────────────────────────────────────────
    p_b = p_b0.copy()
    v_b = v_b0.copy()
    res = SimResult(final_b=p_b.copy(), final_p=p_b.copy())
    t = 0.0
    captured = False
    peak_a = 0.0
    peak_v = 0.0
    n_steps = int(math.ceil(dur / dt))

    for _ in range(n_steps):
        # A 释放载荷（含释放噪声）
        if (not payload.released) and t >= plan.t_r:
            p_rel = p_a.copy()
            v_rel = np.zeros(3)
            if noise.release_pos_sigma > 0:
                p_rel = p_rel + rng.normal(0.0, noise.release_pos_sigma, 3)
            if noise.release_vel_sigma > 0:
                v_rel = v_rel + rng.normal(0.0, noise.release_vel_sigma, 3)
            payload.release(p_rel, v_rel)

        # B 控制：参考 + PD，限幅
        if t <= plan.t_c:
            pr = _interp_traj(ref_t, ref_p, t)
            vr = _interp_traj(ref_t, ref_v, t)
            ar = _interp_traj(ref_t, ref_a, t)
        else:
            pr = ref_p[-1]
            vr = ref_v[-1]
            ar = np.zeros(3)
        a_cmd = ar + kp * (pr - p_b) + kd * (vr - v_b)
        a_norm = float(np.linalg.norm(a_cmd))
        if a_norm > float(bcfg['max_accel']):
            a_cmd = a_cmd * (float(bcfg['max_accel']) / a_norm)
        v_b = v_b + a_cmd * dt
        # 速度范数限幅（与 B 限幅一致）
        v_norm = float(np.linalg.norm(v_b))
        if v_norm > float(bcfg['max_speed']):
            v_b = v_b * (float(bcfg['max_speed']) / v_norm)
        p_b = p_b + v_b * dt
        peak_a = max(peak_a, float(np.linalg.norm(a_cmd)))
        peak_v = max(peak_v, float(np.linalg.norm(v_b)))

        # 载荷推进
        if payload.released:
            payload.step(dt)
            p_p, v_p = payload.pos.copy(), payload.vel.copy()
        else:
            p_p = p_a.copy()
            v_p = np.zeros(3)

        # 诊断记录
        rel = float(np.linalg.norm(p_b - p_p))
        res.t.append(t)
        res.b_pos.append(p_b.copy())
        res.b_vel.append(v_b.copy())
        res.p_pos.append(p_p.copy())
        res.p_vel.append(v_p.copy())
        res.rel_dist.append(rel)

        # 捕获判定（载荷必须已释放）
        if payload.released and not captured:
            rel_v = float(np.linalg.norm(v_b - v_p))
            if rel < float(ccfg['radius']) and rel_v < float(ccfg['rel_speed']):
                captured = True
                res.success = True
                res.t_capture = t
                res.rel_speed_at_capture = rel_v
                break
        t += dt

    res.miss_dist = float(min(res.rel_dist)) if res.rel_dist else math.inf
    res.peak_accel = peak_a
    res.peak_speed = peak_v
    res.final_b = p_b.copy()
    res.final_p = p_p.copy()
    if not res.success:
        # 区分"到了但速度没匹配"与"根本没到"
        res.note = (f'未捕获: 最近 {res.miss_dist:.3f}m (阈 {ccfg["radius"]}m)'
                    f'，末相对速度 {float(np.linalg.norm(v_b - p_p)):.3f}')
    return res, plan


if __name__ == '__main__':
    import yaml, os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg = yaml.safe_load(open(os.path.join(here, 'config', 'catch_scenarios.yaml')))
    lay = cfg['layouts']['hover_catch']
    res, plan = simulate(cfg['defaults'], lay, cfg['scenarios']['M1_basic'])
    print(f'规划: t_r={plan.t_r:.2f}s t_c={plan.t_c:.3f}s h_c={plan.h_c:.2f}m '
          f'v_p={plan.v_p[2]:.2f}m/s peak_a={plan.peak_accel:.2f} delta_v={plan.delta_v:.3f}')
    print(f'仿真: success={res.success} t_capture={res.t_capture:.3f}s '
          f'miss={res.miss_dist:.4f}m rel_v@cap={res.rel_speed_at_capture:.4f}m/s '
          f'peak_a={res.peak_accel:.2f}')

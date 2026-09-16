#!/usr/bin/env python3
"""离线闭环仿真 —— 纯 Python，无 ROS / Gazebo 依赖。

用途：在接 PX4 SITL 之前，把
    规划（rendezvous）→ B 跟踪会合 → 载荷抛体 → 捕获判定
整条链路跑通并量化。坐标与世界系 NED 一致（z 向下为正）。

模型：
  · A 恒速直线飞行（a_vel=0 即悬停），在 t_r 释放；载荷初速 = A 速度。
  · 载荷 = PayloadModel（第一版无阻力）。
  · B = 双积分器 ẍ=a，|a|≤a_max；跟踪会合参考：
        a_cmd = a_ref(t) + Kp·(p_ref−p_B) + Kd·(v_ref−v_B)，再限幅。

两种模式：
  · 开环：规划一次，B 死跟这条参考（M1/M2）。
  · 闭环：载荷离手后每 replan_dt 用**当前观测的载荷状态**重解会合（M3），
          可抵抗释放误差/模型失配/状态噪声。

捕获判定：|p_B−p_p| < r_c 且 |v_B−v_p| < v_c（用载荷真值，测量噪声只影响 B 的规划）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .payload_model import PayloadModel, PayloadParams
from .rendezvous import RendezvousPlanner, PlanResult

Vec3 = np.ndarray


# ------------------------------------------------------------------- config
@dataclass
class SimNoise:
    """噪声/扰动（标称全 0）。"""
    release_pos_sigma: float = 0.0     # m，释放点位置噪声
    release_vel_sigma: float = 0.0     # m/s，释放速度噪声
    b_pos_sigma: float = 0.0           # m，B 初始位置噪声
    payload_pos_sigma: float = 0.0     # m，载荷位置测量噪声（给 B 的规划用）
    payload_vel_sigma: float = 0.0     # m/s，载荷速度测量噪声
    seed: int = 0


@dataclass
class SimResult:
    success: bool = False
    t_capture: float = math.nan
    miss_dist: float = math.inf        # 全程 |p_B−p_p| 最小值
    rel_speed_at_capture: float = math.nan
    peak_accel: float = 0.0
    peak_speed: float = 0.0
    replan_count: int = 0
    final_b: Vec3 = field(default_factory=lambda: np.zeros(3))
    final_p: Vec3 = field(default_factory=lambda: np.zeros(3))
    note: str = ''
    t: List[float] = field(default_factory=list)
    b_pos: List[Vec3] = field(default_factory=list)
    b_vel: List[Vec3] = field(default_factory=list)
    p_pos: List[Vec3] = field(default_factory=list)
    p_vel: List[Vec3] = field(default_factory=list)
    rel_dist: List[float] = field(default_factory=list)


# ------------------------------------------------------------------ helpers
def _interp_traj(ts: np.ndarray, xs: np.ndarray, t: float) -> Vec3:
    """在参考轨迹 (ts, xs[N,3]) 上线性插值（越界取端点）。"""
    if t <= ts[0]:
        return xs[0].copy()
    if t >= ts[-1]:
        return xs[-1].copy()
    i = int(np.searchsorted(ts, t, side='right')) - 1
    i = max(0, min(i, len(ts) - 2))
    a = (t - ts[i]) / max(ts[i + 1] - ts[i], 1e-12)
    return xs[i] * (1.0 - a) + xs[i + 1] * a


def _make_planner(g, pcfg: Dict, bcfg: Dict, ccfg: Dict) -> RendezvousPlanner:
    return RendezvousPlanner(
        g=float(g), b_max_speed=float(bcfg['max_speed']),
        b_max_accel=float(bcfg['max_accel']),
        capture_radius=float(ccfg['radius']), capture_rel_speed=float(ccfg['rel_speed']),
        w_time=float(pcfg['w_time']), w_accel=float(pcfg['w_accel']),
        w_vel=float(pcfg['w_vel']), ground_margin=float(pcfg['ground_margin']))


def simulate(defaults: Dict, layout: Dict, scenario: Dict,
             plan: Optional[PlanResult] = None,
             noise: Optional[SimNoise] = None,
             kp: float = 9.0, kd: float = 6.0,
             closed_loop: bool = False, replan_dt: float = 0.20,
             inflight_tau_max: float = 2.0) -> Tuple[SimResult, PlanResult]:
    """跑一次完整任务，返回 (指标, 规划方案)。"""
    noise = noise or SimNoise()
    rng = np.random.default_rng(noise.seed)

    g = float(defaults['g'])
    hz = float(defaults['control_hz'])
    dt = 1.0 / hz
    dur = float(defaults.get('duration_s', 20.0))
    pcfg = {**defaults['planner'], **scenario.get('planner', {})}
    bcfg = {**defaults['drone_b'], **scenario.get('drone_b', {})}
    ccfg = {**defaults['capture'], **scenario.get('capture', {})}
    pay = {**defaults['payload'], **scenario.get('payload', {})}

    a_init = np.asarray(
        scenario.get('a_init', layout.get('a_init', layout.get('a_hover'))), float).reshape(3)
    a_vel = np.asarray(
        scenario.get('a_vel', layout.get('a_vel', (0.0, 0.0, 0.0))), float).reshape(3)
    p_b0 = np.asarray(
        scenario.get('b_standby', layout['b_standby']), float).reshape(3)
    if noise.b_pos_sigma > 0:
        p_b0 = p_b0 + rng.normal(0.0, noise.b_pos_sigma, 3)
    v_b0 = np.zeros(3)

    # ── 规划 ─────────────────────────────────────────────────────────────
    planner = _make_planner(g, pcfg, bcfg, ccfg)
    if plan is None:
        plan = planner.solve(
            a_init, p_b0, a_vel=a_vel, b_v0=v_b0,
            t_r_range=(pcfg['t_r_min'], pcfg['t_r_max']),
            tau_range=(pcfg['tau_min'], pcfg['tau_max']),
            t_r_step=pcfg['t_r_step'], tau_step=pcfg['tau_step'],
            catch_alt_range=(pcfg['catch_alt_min'], pcfg['catch_alt_max']))
    if not plan.feasible:
        return SimResult(success=False, note=f'规划不可行: {plan.reason}'), plan

    ref_t, ref_p, ref_v, ref_a = RendezvousPlanner.resample(plan, n=401)
    ref_t0 = 0.0            # 当前参考的时间原点
    last_replan = -1e9

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
        # 1) A 释放载荷（含释放噪声）
        if (not payload.released) and t >= plan.t_r:
            p_rel = a_init + a_vel * plan.t_r
            v_rel = a_vel.copy()
            if noise.release_pos_sigma > 0:
                p_rel = p_rel + rng.normal(0.0, noise.release_pos_sigma, 3)
            if noise.release_vel_sigma > 0:
                v_rel = v_rel + rng.normal(0.0, noise.release_vel_sigma, 3)
            payload.release(p_rel, v_rel)

        # 当前载荷状态（未释放时用 A 的位置/速度）
        if payload.released:
            p_p, v_p = payload.pos.copy(), payload.vel.copy()
        else:
            p_p, v_p = a_init + a_vel * t, a_vel.copy()

        # 2) 闭环重规划：用观测到的载荷状态重解会合
        if closed_loop and payload.released and (t - last_replan) >= replan_dt:
            p_meas = p_p.copy(); v_meas = v_p.copy()
            if noise.payload_pos_sigma > 0:
                p_meas = p_meas + rng.normal(0.0, noise.payload_pos_sigma, 3)
            if noise.payload_vel_sigma > 0:
                v_meas = v_meas + rng.normal(0.0, noise.payload_vel_sigma, 3)
            rp = planner.solve_inflight(
                p_meas, v_meas, p_b, v_b,
                tau_range=(0.05, inflight_tau_max), tau_step=0.02,
                catch_alt_range=(pcfg['catch_alt_min'], pcfg['catch_alt_max']))
            last_replan = t
            if rp.feasible:
                ref_t, ref_p, ref_v, ref_a = RendezvousPlanner.resample(rp, n=201)
                ref_t0 = t
                res.replan_count += 1

        # 3) B 跟踪当前参考
        tl = t - ref_t0
        if tl <= ref_t[-1]:
            pr = _interp_traj(ref_t, ref_p, tl)
            vr = _interp_traj(ref_t, ref_v, tl)
            ar = _interp_traj(ref_t, ref_a, tl)
        else:
            pr, vr, ar = ref_p[-1], ref_v[-1], np.zeros(3)
        a_cmd = ar + kp * (pr - p_b) + kd * (vr - v_b)
        a_norm = float(np.linalg.norm(a_cmd))
        if a_norm > float(bcfg['max_accel']):
            a_cmd = a_cmd * (float(bcfg['max_accel']) / a_norm)
        v_b = v_b + a_cmd * dt
        v_norm = float(np.linalg.norm(v_b))
        if v_norm > float(bcfg['max_speed']):
            v_b = v_b * (float(bcfg['max_speed']) / v_norm)
        p_b = p_b + v_b * dt
        peak_a = max(peak_a, float(np.linalg.norm(a_cmd)))
        peak_v = max(peak_v, float(np.linalg.norm(v_b)))

        # 4) 载荷推进
        if payload.released:
            payload.step(dt)
            p_p, v_p = payload.pos.copy(), payload.vel.copy()

        # 5) 记录 + 捕获判定（用真值）
        rel = float(np.linalg.norm(p_b - p_p))
        res.t.append(t)
        res.b_pos.append(p_b.copy())
        res.b_vel.append(v_b.copy())
        res.p_pos.append(p_p.copy())
        res.p_vel.append(v_p.copy())
        res.rel_dist.append(rel)

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
        res.note = (f'未捕获: 最近 {res.miss_dist:.3f}m (阈 {ccfg["radius"]}m)'
                    f'，末相对速度 {float(np.linalg.norm(v_b - p_p)):.3f}')
    return res, plan


if __name__ == '__main__':
    import yaml, os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg = yaml.safe_load(open(os.path.join(here, 'config', 'catch_scenarios.yaml')))
    for name in ('M1_basic', 'M2_line_v10'):
        lay = cfg['layouts'][cfg['scenarios'][name]['layout']]
        res, plan = simulate(cfg['defaults'], lay, cfg['scenarios'][name])
        print(f'{name}: success={res.success} t_cap={res.t_capture:.3f}s '
              f'miss={res.miss_dist:.4f}m rel_v={res.rel_speed_at_capture:.3f}')

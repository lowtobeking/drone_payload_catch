#!/usr/bin/env python3
"""时空会合协调求解 —— 纯 Python，无 ROS 依赖。

问题（M1：A 定点悬停释放）：
   A 悬停在 p_A，载荷竖直落下（v_r≈0）。规划器要同时决定
       · 释放时刻 t_r
       · 捕获点高度 h_c（⇒ 下落时长 τ_c、载荷到达速度 v_p）
   使 B 从待命点 (p_B0, v_B0=0) 在 t_c = t_r + τ_c 时刻到达捕获点、且终端速度尽量等于 v_p。

为什么是"协调"：h_c 选得高 → 载荷慢、速度易匹配，但留给 B 的时间短；
选得低 → 时间足，但载荷快。规划器在这个一维权衡上取最优。

B 的会合轨迹：对双积分器（ȧ=v, v̇=a）固定两端位置+速度、最小化 ∫|a|²dt，
每轴解析解是三次多项式：
    x(t) = c0 + c1 t + c2 t² + c3 t³
    c0 = x0, c1 = v0
    c2 = 3D/T² − Δv/T,  c3 = −2D/T³ + Δv/T²,  D = x1 − x0 − v0 T
代价：J = w_time·t_r + w_accel·(峰值加速度/a_max) + w_vel·|Δv(T)|²
约束：|v(t)| ≤ v_max、|a(t)| ≤ a_max、离地余量 ≥ ground_margin。

坐标：世界系 NED（z 向下为正），离地高度 = -z。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .payload_model import PayloadModel

Vec3 = np.ndarray


# --------------------------------------------------------------- cubic tools
def min_energy_cubic(x0: float, v0: float, x1: float, v1: float,
                     T: float) -> Optional[Tuple[float, float, float, float]]:
    """单轴：两端位置/速度固定、min ∫a² 的三次多项式系数 (c0,c1,c2,c3)。"""
    if T <= 1e-9:
        return None
    D = x1 - x0 - v0 * T
    dv = v1 - v0
    c0 = x0
    c1 = v0
    c2 = 3.0 * D / (T * T) - dv / T
    c3 = -2.0 * D / (T ** 3) + dv / (T * T)
    return (c0, c1, c2, c3)


def eval_cubic(c: Tuple[float, float, float, float], t: float
               ) -> Tuple[float, float, float]:
    """返回 (x, v, a)。"""
    c0, c1, c2, c3 = c
    x = c0 + c1 * t + c2 * t * t + c3 * t ** 3
    v = c1 + 2.0 * c2 * t + 3.0 * c3 * t * t
    a = 2.0 * c2 + 6.0 * c3 * t
    return x, v, a


def min_energy_cubic_soft(x0: float, v0: float, x1: float, v_ref: float,
                          T: float, w: float
                          ) -> Tuple[float, float, float, float]:
    """单轴：位置两端硬，终端速度软——min ∫a²dt + w·(v(T)−v_ref)²。

    仍是对双积分器、三次多项式解；`w→∞` 退化为终端速度精确匹配
    （等价 min_energy_cubic），`w→0` 则终端速度自由（自然边界 a(T)=0）。
    推导：x(t)=c0+c1t+c2t²+c3t³，位置约束 c2T²+c3T³=D 消去 c2，
    对 q:=c3 求导得 q = −(2D/T² + w·A0) / (T·(4 + w·T))，
    其中 D=x1−x0−v0T、A0=v0+2D/T−v_ref。

    用途：当载荷到达速度超过 B 限速、精确匹配不可行时，规划器用它在
    “位置到达”与“速度接近”之间取折中（位置始终硬约束）。
    """
    if T <= 1e-9:
        return (x0, v0, 0.0, 0.0)
    D = x1 - x0 - v0 * T
    A0 = v0 + 2.0 * D / T - v_ref
    q = -(2.0 * D / (T * T) + w * A0) / (T * (4.0 + w * T))
    c3 = q
    c2 = D / (T * T) - q * T
    return (x0, v0, c2, c3)


# ------------------------------------------------------------------- results
@dataclass
class PlanResult:
    feasible: bool = False
    t_r: float = 0.0          # 释放时刻（从规划时刻算起）
    t_c: float = 0.0          # 捕获时刻
    tau_c: float = 0.0        # 下落时长
    h_c: float = 0.0          # 捕获高度（离地）
    p_c: Vec3 = field(default_factory=lambda: np.zeros(3))   # 捕获点 NED
    v_p: Vec3 = field(default_factory=lambda: np.zeros(3))   # 载荷到达速度 NED
    peak_accel: float = 0.0
    peak_speed: float = 0.0
    min_alt: float = 0.0
    delta_v: float = 0.0       # 捕获时刻 B 与载荷的速度失配 |v_B−v_p|
    cost: float = math.inf
    reason: str = ''
    # 采样后的 B 参考轨迹（t, pos(N,K,3), vel(N,K,3), acc(N,K,3)）
    traj_t: Optional[np.ndarray] = None
    traj_p: Optional[np.ndarray] = None
    traj_v: Optional[np.ndarray] = None
    traj_a: Optional[np.ndarray] = None


# ------------------------------------------------------------------ planner
class HoverReleasePlanner:
    """A 悬停释放场景下的协调求解器。"""

    def __init__(self, g: float = 9.81,
                 b_max_speed: float = 3.0, b_max_accel: float = 4.0,
                 capture_radius: float = 0.30, capture_rel_speed: float = 1.50,
                 w_time: float = 0.20, w_accel: float = 1.0, w_vel: float = 5.0,
                 ground_margin: float = 0.30, n_check: int = 200):
        self.g = float(g)
        self.v_max = float(b_max_speed)
        self.a_max = float(b_max_accel)
        self.r_c = float(capture_radius)
        self.v_c = float(capture_rel_speed)
        self.w_time = float(w_time)
        self.w_accel = float(w_accel)
        self.w_vel = float(w_vel)
        self.ground_margin = float(ground_margin)
        self.n_check = int(n_check)

    # ------------------------------------------------------------- evaluate
    def _eval_candidate(self, p_a: Vec3, p_b0: Vec3, v_b0: Vec3,
                        h_c: float, t_r: float, mode: str = 'auto') -> PlanResult:
        res = PlanResult()
        h_a = -float(p_a[2])
        fall = h_a - h_c
        if fall <= 1e-6:
            res.reason = 'fall<=0'
            return res
        # A 悬停 → 载荷竖直初速 0；若将来 A 带速抛投，此处换成 v_z0 = v_a_z
        tau_c = PayloadModel.fall_time(fall, self.g, v_z0=0.0)
        t_c = t_r + tau_c
        T = t_c
        if T <= 1e-6:
            res.reason = 'T<=0'
            return res
        p_c = np.array([p_a[0], p_a[1], -h_c], dtype=float)
        v_p = np.array([0.0, 0.0, self.g * tau_c], dtype=float)

        # ① 先试“终端速度精确匹配”（位置硬+速度硬）：理想接住
        if mode in ('auto', 'exact'):
            cub = [min_energy_cubic(p_b0[k], v_b0[k], p_c[k], v_p[k], T)
                   for k in range(3)]
            if all(c is not None for c in cub):
                r = self._finish_candidate(res, cub, t_r, t_c, tau_c, h_c,
                                           p_c, v_p, 0.0)
                if r.feasible or mode == 'exact':
                    return r

        # ② 精确不可行（常见于载荷速度 > B 限速）→ 终端速度软代价折中
        #    位置仍硬到；失配超过捕获阈值 v_c 则判不可行。
        if mode in ('auto', 'soft'):
            cub = [min_energy_cubic_soft(p_b0[k], v_b0[k], p_c[k], v_p[k],
                                         T, self.w_vel)
                   for k in range(3)]
            vT = np.array([eval_cubic(cub[k], T)[1] for k in range(3)])
            dv = float(np.linalg.norm(vT - v_p))
            r = self._finish_candidate(res, cub, t_r, t_c, tau_c, h_c,
                                       p_c, v_p, dv)
            if r.feasible and dv > self.v_c:
                r.feasible = False
                r.reason = f'rel_speed {dv:.2f}>{self.v_c:.2f}'
            return r
        return res

    def _finish_candidate(self, res: PlanResult,
                          cubics: List[Tuple[float, float, float, float]],
                          t_r: float, t_c: float, tau_c: float, h_c: float,
                          p_c: Vec3, v_p: Vec3, dv: float) -> PlanResult:
        """给定各轴多项式，采样评估峰值/离地/代价并填充 PlanResult。"""
        T = t_c
        ts = np.linspace(0.0, T, self.n_check)
        pos = np.zeros((self.n_check, 3))
        vel = np.zeros((self.n_check, 3))
        acc = np.zeros((self.n_check, 3))
        for k in range(3):
            for i, t in enumerate(ts):
                x, v, a = eval_cubic(cubics[k], t)
                pos[i, k], vel[i, k], acc[i, k] = x, v, a
        peak_speed = float(np.max(np.linalg.norm(vel, axis=1)))
        peak_accel = float(np.max(np.linalg.norm(acc, axis=1)))
        min_alt = float(-np.max(pos[:, 2]))   # 最高 z = 最低高度

        res.t_r, res.t_c, res.tau_c, res.h_c = t_r, t_c, tau_c, h_c
        res.p_c, res.v_p = p_c, v_p
        res.peak_accel, res.peak_speed, res.min_alt = peak_accel, peak_speed, min_alt
        res.delta_v = dv
        res.traj_t, res.traj_p, res.traj_v, res.traj_a = ts, pos, vel, acc

        if peak_speed > self.v_max + 1e-9:
            res.reason = f'peak_speed {peak_speed:.2f}>{self.v_max:.2f}'
            return res
        if peak_accel > self.a_max + 1e-9:
            res.reason = f'peak_accel {peak_accel:.2f}>{self.a_max:.2f}'
            return res
        if min_alt < self.ground_margin - 1e-9:
            res.reason = f'min_alt {min_alt:.2f}<{self.ground_margin:.2f}'
            return res

        res.cost = (self.w_time * t_r
                    + self.w_accel * (peak_accel / self.a_max)
                    + self.w_vel * dv * dv)
        res.feasible = True
        res.reason = 'ok'
        return res

    # ---------------------------------------------------------------- solve
    def solve(self, p_a: Sequence[float], p_b0: Sequence[float],
              v_b0: Sequence[float] = (0.0, 0.0, 0.0),
              catch_alt_range: Tuple[float, float] = (0.8, 2.0),
              release_delay_range: Tuple[float, float] = (0.0, 8.0),
              catch_alt_step: float = 0.02,
              release_delay_step: float = 0.02,
              ) -> PlanResult:
        """网格搜索 (h_c, t_r)，返回代价最小的可行方案。"""
        p_a = np.asarray(p_a, float).reshape(3)
        p_b0 = np.asarray(p_b0, float).reshape(3)
        v_b0 = np.asarray(v_b0, float).reshape(3)

        h_lo, h_hi = float(catch_alt_range[0]), float(catch_alt_range[1])
        r_lo, r_hi = float(release_delay_range[0]), float(release_delay_range[1])
        # 捕获高度不能高于 A（否则载荷不会下落）
        h_hi = min(h_hi, -float(p_a[2]) - 1e-3)
        n_h = max(1, int(round((h_hi - h_lo) / catch_alt_step)) + 1)
        n_r = max(1, int(round((r_hi - r_lo) / release_delay_step)) + 1)

        best: Optional[PlanResult] = None
        n_feasible = 0
        for i in range(n_h):
            h_c = h_lo + i * catch_alt_step
            if h_c > h_hi + 1e-9:
                break
            for j in range(n_r):
                t_r = r_lo + j * release_delay_step
                if t_r > r_hi + 1e-9:
                    break
                r = self._eval_candidate(p_a, p_b0, v_b0, h_c, t_r)
                if not r.feasible:
                    continue
                n_feasible += 1
                if best is None or r.cost < best.cost:
                    best = r

        if best is None:
            # 不可行：回一个空结果并给出最有信息量的原因（取"最接近可行"的候选）
            r = PlanResult(feasible=False, reason='no feasible (h_c, t_r) in range')
            return r
        return best

    # -------------------------------------------------- reference resampling
    @staticmethod
    def resample(res: PlanResult, n: int = 101) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """把方案按等间隔重采样成 n 点 (t, p, v, a)，供控制器当参考。"""
        if res.traj_t is None:
            raise RuntimeError('方案没有轨迹（不可行）')
        T = float(res.traj_t[-1])
        ts = np.linspace(0.0, T, n)
        cubics = [min_energy_cubic(res.traj_p[0, k], res.traj_v[0, k],
                                   res.p_c[k], res.v_p[k], T) for k in range(3)]
        p = np.zeros((n, 3)); v = np.zeros((n, 3)); a = np.zeros((n, 3))
        for k in range(3):
            for i, t in enumerate(ts):
                x, vv, aa = eval_cubic(cubics[k], t)
                p[i, k], v[i, k], a[i, k] = x, vv, aa
        return ts, p, v, a


if __name__ == '__main__':
    # 自测：cubic 边界条件 + 一次悬停释放规划
    c = min_energy_cubic(0.0, 0.0, 1.0, 0.0, 2.0)
    x0, v0, _ = eval_cubic(c, 0.0)
    x1, v1, _ = eval_cubic(c, 2.0)
    assert abs(x0) < 1e-12 and abs(v0) < 1e-12
    assert abs(x1 - 1.0) < 1e-9 and abs(v1) < 1e-9
    print('min_energy_cubic 边界自测通过')

    pl = HoverReleasePlanner()
    plan = pl.solve(p_a=[0, 0, -2.5], p_b0=[1.2, 0, -3.5],
                    catch_alt_range=(0.8, 2.0), release_delay_range=(0.0, 8.0))
    print(f'feasible={plan.feasible} reason={plan.reason}')
    if plan.feasible:
        print(f'  t_r={plan.t_r:.2f}s t_c={plan.t_c:.2f}s tau_c={plan.tau_c:.3f}s '
              f'h_c={plan.h_c:.2f}m')
        print(f'  p_c={np.round(plan.p_c,3)} v_p={np.round(plan.v_p,3)}')
        print(f'  peak_accel={plan.peak_accel:.2f} peak_speed={plan.peak_speed:.2f} '
              f'min_alt={plan.min_alt:.2f} cost={plan.cost:.3f}')

#!/usr/bin/env python3
"""时空会合协调求解 —— 纯 Python，无 ROS 依赖。

任务：A 携带载荷飞行，在算法求出的 (释放时刻 t_r, 捕获时刻 t_c) 抛投；
载荷离手后做抛体运动；B 从待命点到空间会合点，位置到达 + 相对速度尽量小。

A 的运动（第一版）：恒定速度直线飞行
    p_A(t) = a_init + a_vel·t,   v_A(t) = a_vel
释放时：p_r = p_A(t_r), v_r = v_A(t_r)。
载荷（无阻力）：
    p_p(τ) = p_r + v_r·τ + ½g·τ²,   v_p(τ) = v_r + g·τ   (τ = t − t_r)
捕获于 τ_c ⇒ 会合点 p_c = p_p(τ_c)、载荷速度 v_c = v_p(τ_c)，捕获时刻 t_c = t_r + τ_c。

B 的会合轨迹：对双积分器、固定两端位置+速度、min∫|a|²dt，每轴解析解是三次多项式
    x(t) = c0 + c1 t + c2 t² + c3 t³
    c0=x0, c1=v0, c2=3D/T²−Δv/T, c3=−2D/T³+Δv/T², D=x1−x0−v0T
若"终端速度精确匹配"不可行（如载荷速度超 B 限速），退为终端速度软代价的解析解
（位置仍硬到，速度按权重折中）。

协调搜索：在 (t_r, τ_c) 二维网格上取使代价最小者
    J = w_time·t_r + w_accel·(峰值加速度/a_max) + w_vel·|Δv|²
约束：|v(t)|≤v_max、|a(t)|≤a_max、离地余量、捕获高度在给定区间。

坐标：世界系 NED（z 向下为正），离地高度 = -z。悬停释放 = a_vel 全零的特例。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

Vec3 = np.ndarray


# --------------------------------------------------------------- cubic tools
def min_energy_cubic(x0: float, v0: float, x1: float, v1: float,
                     T: float) -> Optional[Tuple[float, float, float, float]]:
    """单轴：两端位置/速度固定、min ∫a² 的三次多项式系数 (c0,c1,c2,c3)。"""
    if T <= 1e-9:
        return None
    D = x1 - x0 - v0 * T
    dv = v1 - v0
    return (x0, v0, 3.0 * D / (T * T) - dv / T, -2.0 * D / (T ** 3) + dv / (T * T))


def min_energy_cubic_soft(x0: float, v0: float, x1: float, v_ref: float,
                          T: float, w: float) -> Tuple[float, float, float, float]:
    """单轴：位置两端硬、终端速度软 —— min ∫a²dt + w·(v(T)−v_ref)²。

    w→∞ 退化为精确匹配（等价 min_energy_cubic）；w→0 终端速度自由（自然边界 a(T)=0）。
    """
    if T <= 1e-9:
        return (x0, v0, 0.0, 0.0)
    D = x1 - x0 - v0 * T
    A0 = v0 + 2.0 * D / T - v_ref
    q = -(2.0 * D / (T * T) + w * A0) / (T * (4.0 + w * T))
    return (x0, v0, D / (T * T) - q * T, q)


def eval_cubic(c: Tuple[float, float, float, float], t: float
               ) -> Tuple[float, float, float]:
    """返回单点 (x, v, a)。"""
    c0, c1, c2, c3 = c
    return (c0 + c1 * t + c2 * t * t + c3 * t ** 3,
            c1 + 2.0 * c2 * t + 3.0 * c3 * t * t,
            2.0 * c2 + 6.0 * c3 * t)


def eval_cubic_vec(c: Tuple[float, float, float, float],
                   ts: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """向量化：对时间数组 ts 求 (x, v, a)。"""
    c0, c1, c2, c3 = c
    t2 = ts * ts
    x = c0 + c1 * ts + c2 * t2 + c3 * t2 * ts
    v = c1 + 2.0 * c2 * ts + 3.0 * c3 * t2
    a = 2.0 * c2 + 6.0 * c3 * ts
    return x, v, a


# ------------------------------------------------------------- payload tools
def payload_state(p_r: Vec3, v_r: Vec3, tau: float, g: float = 9.81
                  ) -> Tuple[Vec3, Vec3]:
    """无阻力抛体：释放状态 (p_r, v_r)、经过 tau 秒后的 (位置, 速度)，世界系 NED。"""
    g_vec = np.array([0.0, 0.0, float(g)])
    p = np.asarray(p_r, float).reshape(3) + np.asarray(v_r, float).reshape(3) * tau \
        + 0.5 * g_vec * tau * tau
    v = np.asarray(v_r, float).reshape(3) + g_vec * tau
    return p, v


# ------------------------------------------------------------------- results
@dataclass
class PlanResult:
    feasible: bool = False
    t_r: float = 0.0          # 释放时刻
    t_c: float = 0.0          # 捕获时刻
    tau_c: float = 0.0        # 下落时长
    p_r: Vec3 = field(default_factory=lambda: np.zeros(3))   # 释放点 NED
    v_r: Vec3 = field(default_factory=lambda: np.zeros(3))   # 释放速度 NED
    h_c: float = 0.0          # 捕获高度（离地）
    p_c: Vec3 = field(default_factory=lambda: np.zeros(3))   # 会合点 NED
    v_p: Vec3 = field(default_factory=lambda: np.zeros(3))   # 载荷到达速度 NED
    peak_accel: float = 0.0
    peak_speed: float = 0.0
    min_alt: float = 0.0
    delta_v: float = 0.0      # 捕获时刻 B 与载荷的速度失配
    cost: float = math.inf
    reason: str = ''
    traj_t: Optional[np.ndarray] = None
    traj_p: Optional[np.ndarray] = None
    traj_v: Optional[np.ndarray] = None
    traj_a: Optional[np.ndarray] = None


# ------------------------------------------------------------------ planner
class RendezvousPlanner:
    """(t_r, τ_c) 协调求解器。A 恒速直线飞行；a_vel=0 即悬停释放。"""

    def __init__(self, g: float = 9.81,
                 b_max_speed: float = 4.0, b_max_accel: float = 5.0,
                 capture_radius: float = 0.30, capture_rel_speed: float = 1.50,
                 w_time: float = 0.20, w_accel: float = 1.0, w_vel: float = 5.0,
                 ground_margin: float = 0.30, n_check: int = 24):
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

    # ------------------------------------------------------------- internals
    def _finish(self, res: PlanResult,
                cubics: List[Tuple[float, float, float, float]],
                t_r: float, t_c: float, tau_c: float,
                p_r: Vec3, v_r: Vec3, p_c: Vec3, v_c: Vec3, dv: float
                ) -> PlanResult:
        T = t_c
        ts = np.linspace(0.0, T, self.n_check)
        POS = np.zeros((self.n_check, 3))
        VEL = np.zeros((self.n_check, 3))
        ACC = np.zeros((self.n_check, 3))
        for k in range(3):
            x, v, a = eval_cubic_vec(cubics[k], ts)
            POS[:, k], VEL[:, k], ACC[:, k] = x, v, a
        peak_speed = float(np.max(np.linalg.norm(VEL, axis=1)))
        peak_accel = float(np.max(np.linalg.norm(ACC, axis=1)))
        min_alt = float(-np.max(POS[:, 2]))

        res.t_r, res.t_c, res.tau_c = t_r, t_c, tau_c
        res.p_r, res.v_r = p_r, v_r
        res.p_c, res.v_p = p_c, v_c
        res.h_c = float(-p_c[2])
        res.peak_accel, res.peak_speed, res.min_alt = peak_accel, peak_speed, min_alt
        res.delta_v = dv
        res.traj_t, res.traj_p, res.traj_v, res.traj_a = ts, POS, VEL, ACC

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

    def eval_candidate(self, a_init: Vec3, a_vel: Vec3, b_p0: Vec3, b_v0: Vec3,
                       t_r: float, tau_c: float, mode: str = 'auto') -> PlanResult:
        """评估一个 (t_r, τ_c) 候选。mode: auto|exact|soft。"""
        res = PlanResult()
        if tau_c <= 1e-6 or t_r < -1e-9:
            res.reason = 'bad t_r/tau'
            return res
        p_r = np.asarray(a_init, float).reshape(3) + np.asarray(a_vel, float).reshape(3) * t_r
        v_r = np.asarray(a_vel, float).reshape(3).copy()
        p_c, v_c = payload_state(p_r, v_r, tau_c, self.g)
        t_c = t_r + tau_c
        if t_c <= 1e-6:
            res.reason = 'T<=0'
            return res
        # 载荷必须还在空中（离地 > ground_margin）
        if -float(p_c[2]) < self.ground_margin:
            res.reason = f'payload hit ground (h={-p_c[2]:.2f})'
            return res

        if mode in ('auto', 'exact'):
            cub = [min_energy_cubic(b_p0[k], b_v0[k], p_c[k], v_c[k], t_c)
                   for k in range(3)]
            if all(c is not None for c in cub):
                r = self._finish(res, cub, t_r, t_c, tau_c, p_r, v_r, p_c, v_c, 0.0)
                if r.feasible or mode == 'exact':
                    return r
        if mode in ('auto', 'soft'):
            cub = [min_energy_cubic_soft(b_p0[k], b_v0[k], p_c[k], v_c[k],
                                         t_c, self.w_vel) for k in range(3)]
            vT = np.array([eval_cubic(cub[k], t_c)[1] for k in range(3)])
            dv = float(np.linalg.norm(vT - v_c))
            r = self._finish(res, cub, t_r, t_c, tau_c, p_r, v_r, p_c, v_c, dv)
            if r.feasible and dv > self.v_c:
                r.feasible = False
                r.reason = f'rel_speed {dv:.2f}>{self.v_c:.2f}'
            return r
        return res

    # ---------------------------------------------------------------- solve
    def solve(self, a_init: Sequence[float], b_p0: Sequence[float],
              a_vel: Sequence[float] = (0.0, 0.0, 0.0),
              b_v0: Sequence[float] = (0.0, 0.0, 0.0),
              t_r_range: Tuple[float, float] = (0.0, 8.0),
              tau_range: Tuple[float, float] = (0.05, 1.2),
              t_r_step: float = 0.05, tau_step: float = 0.01,
              catch_alt_range: Tuple[float, float] = (0.8, 2.8),
              ) -> PlanResult:
        """在 (t_r, τ_c) 网格上找代价最小的可行方案。"""
        a_init = np.asarray(a_init, float).reshape(3)
        a_vel = np.asarray(a_vel, float).reshape(3)
        b_p0 = np.asarray(b_p0, float).reshape(3)
        b_v0 = np.asarray(b_v0, float).reshape(3)
        h_lo, h_hi = float(catch_alt_range[0]), float(catch_alt_range[1])

        n_tr = max(1, int(round((t_r_range[1] - t_r_range[0]) / t_r_step)) + 1)
        n_tau = max(1, int(round((tau_range[1] - tau_range[0]) / tau_step)) + 1)
        best: Optional[PlanResult] = None
        n_feasible = 0
        for i in range(n_tr):
            t_r = t_r_range[0] + i * t_r_step
            if t_r > t_r_range[1] + 1e-9:
                break
            for j in range(n_tau):
                tau_c = tau_range[0] + j * tau_step
                if tau_c > tau_range[1] + 1e-9:
                    break
                r = self.eval_candidate(a_init, a_vel, b_p0, b_v0, t_r, tau_c)
                if not r.feasible:
                    continue
                if not (h_lo - 1e-9 <= r.h_c <= h_hi + 1e-9):
                    continue
                n_feasible += 1
                if best is None or r.cost < best.cost:
                    best = r
        if best is None:
            return PlanResult(feasible=False, reason='no feasible (t_r, tau_c) in range')
        return best

    # ------------------------------------------- in-flight re-planning (M3)
    def solve_inflight(self, p_p: Sequence[float], v_p: Sequence[float],
                       b_p0: Sequence[float], b_v0: Sequence[float],
                       tau_range: Tuple[float, float] = (0.05, 1.5),
                       tau_step: float = 0.02,
                       catch_alt_range: Tuple[float, float] = (0.8, 2.8),
                       ) -> PlanResult:
        """载荷已离手、B 处于任意状态时的重规划：在剩余下落时间 τ 上搜索。

        给定载荷当前状态 (p_p, v_p)（可含测量噪声）与 B 当前状态，
        对未来 τ 求会合：p_c=p_p+v_p·τ+½gτ², v_c=v_p+gτ，B 从当前状态到 (p_c,v_c)。
        每拍调用即得到“从此刻起的最优会合参考”→ 闭环重规划。
        """
        p_p = np.asarray(p_p, float).reshape(3)
        v_p = np.asarray(v_p, float).reshape(3)
        b_p0 = np.asarray(b_p0, float).reshape(3)
        b_v0 = np.asarray(b_v0, float).reshape(3)
        h_lo, h_hi = float(catch_alt_range[0]), float(catch_alt_range[1])
        n_tau = max(1, int(round((tau_range[1] - tau_range[0]) / tau_step)) + 1)
        best: Optional[PlanResult] = None
        for j in range(n_tau):
            tau_c = tau_range[0] + j * tau_step
            if tau_c > tau_range[1] + 1e-9:
                break
            p_c, v_c = payload_state(p_p, v_p, tau_c, self.g)
            if -float(p_c[2]) < self.ground_margin:
                continue
            T = tau_c
            res = PlanResult()
            # ① 先试终端速度精确匹配
            r = None
            cub = [min_energy_cubic(b_p0[k], b_v0[k], p_c[k], v_c[k], T)
                   for k in range(3)]
            if all(c is not None for c in cub):
                r = self._finish(res, cub, 0.0, T, tau_c, p_p, v_p, p_c, v_c, 0.0)
                if not r.feasible:
                    r = None
            # ② 精确不可行 → 终端速度软代价（位置仍硬到）
            if r is None:
                res = PlanResult()
                cub = [min_energy_cubic_soft(b_p0[k], b_v0[k], p_c[k], v_c[k],
                                             T, self.w_vel) for k in range(3)]
                vT = np.array([eval_cubic(cub[k], T)[1] for k in range(3)])
                dv = float(np.linalg.norm(vT - v_c))
                r = self._finish(res, cub, 0.0, T, tau_c, p_p, v_p, p_c, v_c, dv)
                if r.feasible and dv > self.v_c:
                    r.feasible = False
                    r.reason = f'rel_speed {dv:.2f}>{self.v_c:.2f}'
            if r is None or not r.feasible:
                continue
            if not (h_lo - 1e-9 <= r.h_c <= h_hi + 1e-9):
                continue
            # 重规划里“释放时刻”已固定，代价用剩余时间代替 t_r
            r.cost = (self.w_time * tau_c
                      + self.w_accel * (r.peak_accel / self.a_max)
                      + self.w_vel * r.delta_v * r.delta_v)
            if best is None or r.cost < best.cost:
                best = r
        if best is None:
            return PlanResult(feasible=False, reason='in-flight no feasible tau')
        return best

    # -------------------------------------------------- reference resampling
    @staticmethod
    def resample(res: PlanResult, n: int = 401
                 ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """把方案等间隔重采样成 n 点 (t, p, v, a)，供控制器当参考。"""
        if res.traj_t is None:
            raise RuntimeError('方案没有轨迹（不可行）')
        T = float(res.t_c)
        ts = np.linspace(0.0, T, n)
        cubics = [min_energy_cubic(res.traj_p[0, k], res.traj_v[0, k],
                                   res.p_c[k], res.v_p[k], T) for k in range(3)]
        p = np.zeros((n, 3)); v = np.zeros((n, 3)); a = np.zeros((n, 3))
        for k in range(3):
            p[:, k], v[:, k], a[:, k] = eval_cubic_vec(cubics[k], ts)
        return ts, p, v, a


# 悬停释放场景的旧名兼容（a_vel=0）
HoverReleasePlanner = RendezvousPlanner


if __name__ == '__main__':
    # 自测 1：三次多项式边界
    c = min_energy_cubic(0.0, 0.0, 1.0, 0.0, 2.0)
    assert abs(eval_cubic(c, 0.0)[0]) < 1e-12 and abs(eval_cubic(c, 2.0)[0] - 1.0) < 1e-9
    # 自测 2：软终端速度 w→∞ 趋于精确
    cs = min_energy_cubic_soft(0.0, 0.0, 1.0, 0.5, 2.0, 1e8)
    assert abs(eval_cubic(cs, 2.0)[1] - 0.5) < 1e-4
    # 自测 3：悬停释放规划
    pl = RendezvousPlanner(b_max_speed=4.0, b_max_accel=5.0)
    plan = pl.solve([0, 0, -2.5], [1.2, 0, -3.5], a_vel=(0, 0, 0),
                    t_r_range=(0, 8), tau_range=(0.05, 1.2),
                    catch_alt_range=(0.8, 2.0))
    print(f'[hover] feasible={plan.feasible} t_r={plan.t_r:.2f} t_c={plan.t_c:.3f} '
          f'h_c={plan.h_c:.2f} v_p={plan.v_p.round(2)} dv={plan.delta_v:.3f}')
    # 自测 4：带速抛投规划
    plan2 = pl.solve([-4, 0, -3.0], [2.0, 0, -2.5], a_vel=(1.0, 0, 0),
                     t_r_range=(0, 8), tau_range=(0.05, 1.2),
                     catch_alt_range=(0.8, 2.8))
    print(f'[line ] feasible={plan2.feasible} t_r={plan2.t_r:.2f} t_c={plan2.t_c:.3f} '
          f'p_r={plan2.p_r.round(2)} h_c={plan2.h_c:.2f} v_p={plan2.v_p.round(2)} '
          f'dv={plan2.delta_v:.3f} reason={plan2.reason}')

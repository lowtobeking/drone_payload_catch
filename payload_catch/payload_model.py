#!/usr/bin/env python3
"""载荷（抛体）动力学模型 —— 纯 Python，无 ROS 依赖。

第一版（M1）：`drag_mode='none'`，载荷离手后只受重力，解析可解：
    p(τ) = p0 + v0·τ + ½·g·τ²
    v(τ) = v0 + g·τ
其中 g 为 NED 下的重力向量 (0, 0, +g)（z 向下为正）。

可选阻力与常值风（后续鲁棒性）：
    v_rel = v − v_wind
    linear    : a_drag = −k · v_rel
    quadratic : a_drag = −k · |v_rel| · v_rel
（k 的含义随模式：linear 为 1/s，quadratic 为 1/m。物理上
 a_drag = −½ρCdA/m·|v_rel|·v_rel，把常量系数并进 k 即可。）

坐标：世界系 NED（x=北, y=东, z=下），高度(离地) = -z。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

import numpy as np

Vec3 = np.ndarray


@dataclass
class PayloadParams:
    """载荷物理参数。"""
    mass: float = 0.30                 # kg
    gravity: float = 9.81              # m/s²
    drag_mode: str = 'none'            # none | linear | quadratic
    drag_k: float = 0.0                # 见模块 docstring
    wind: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # NED m/s


class PayloadModel:
    """单块载荷的抛体模型。

    生命周期：
        m = PayloadModel(PayloadParams(...))
        m.release(p0, v0)      # 离手，开始自由飞
        m.step(dt)             # 推进（有阻力时数值积分；无阻力时也走解析）
        m.predict(tau)         # 预测量（不改变内部状态，供规划/监控）
    """

    def __init__(self, params: Optional[PayloadParams] = None):
        self.p = params or PayloadParams()
        self.g_vec = np.array([0.0, 0.0, float(self.p.gravity)], dtype=float)
        self.wind = np.asarray(self.p.wind, dtype=float).reshape(3)
        self.reset()

    # ---------------------------------------------------------------- state
    def reset(self) -> None:
        self.released = False
        self.t = 0.0
        self._p0 = np.zeros(3)
        self._v0 = np.zeros(3)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)

    def release(self, p0: Sequence[float], v0: Sequence[float]) -> None:
        """在位置 p0、以速度 v0（通常 = A 离手瞬间的速度）释放。"""
        self._p0 = np.asarray(p0, dtype=float).reshape(3).copy()
        self._v0 = np.asarray(v0, dtype=float).reshape(3).copy()
        self.pos = self._p0.copy()
        self.vel = self._v0.copy()
        self.t = 0.0
        self.released = True

    # -------------------------------------------------------------- dynamics
    def accel(self, vel: Vec3) -> Vec3:
        """当前速度下的加速度（重力 + 可选阻力）。"""
        a = self.g_vec.copy()
        if self.p.drag_mode == 'none':
            return a
        v_rel = np.asarray(vel, dtype=float).reshape(3) - self.wind
        if self.p.drag_mode == 'linear':
            a -= self.p.drag_k * v_rel
        elif self.p.drag_mode == 'quadratic':
            a -= self.p.drag_k * float(np.linalg.norm(v_rel)) * v_rel
        else:
            raise ValueError(f'未知 drag_mode={self.p.drag_mode!r}')
        return a

    def step(self, dt: float) -> Tuple[Vec3, Vec3]:
        """推进 dt（RK4）。返回新的 (pos, vel)。"""
        if not self.released:
            return self.pos.copy(), self.vel.copy()

        def f(p, v):
            return v, self.accel(v)

        p, v = self.pos, self.vel
        k1p, k1v = f(p, v)
        k2p, k2v = f(p + 0.5 * dt * k1p, v + 0.5 * dt * k1v)
        k3p, k3v = f(p + 0.5 * dt * k2p, v + 0.5 * dt * k2v)
        k4p, k4v = f(p + dt * k3p, v + dt * k3v)
        self.pos = p + (dt / 6.0) * (k1p + 2 * k2p + 2 * k3p + k4p)
        self.vel = v + (dt / 6.0) * (k1v + 2 * k2v + 2 * k3v + k4v)
        self.t += dt
        return self.pos.copy(), self.vel.copy()

    # --------------------------------------------------------------- predict
    def predict(self, tau: float) -> Tuple[Vec3, Vec3]:
        """预测量：释放后经过 tau 秒的位置与速度（不改变状态）。

        无阻力时解析精确；有阻力时数值积分（不改变内部状态）。
        """
        if not self.released:
            raise RuntimeError('载荷尚未释放，无法预测')
        if tau <= 0.0:
            return self._p0.copy(), self._v0.copy()
        if self.p.drag_mode == 'none':
            pos = self._p0 + self._v0 * tau + 0.5 * self.g_vec * tau * tau
            vel = self._v0 + self.g_vec * tau
            return pos, vel
        # 有阻力：临时副本数值积分（步长 ≤ 2 ms 保精度）
        tmp = PayloadModel(self.p)
        tmp.release(self._p0, self._v0)
        n = max(1, int(math.ceil(tau / 2e-3)))
        h = tau / n
        for _ in range(n):
            tmp.step(h)
        return tmp.pos.copy(), tmp.vel.copy()

    @staticmethod
    def fall_time(fall_dist: float, g: float = 9.81, v_z0: float = 0.0) -> float:
        """无阻力、初速竖直分量 v_z0（NED 向下为正）时，落下 fall_dist 所需时间。"""
        # fall_dist = v_z0·t + ½ g t²  ⇒  ½ g t² + v_z0 t − fall_dist = 0
        disc = v_z0 * v_z0 + 2.0 * g * fall_dist
        return (-v_z0 + math.sqrt(max(0.0, disc))) / g


if __name__ == '__main__':
    # 自测：无阻力解析 vs 数值积分；fall_time 反解
    pm = PayloadModel(PayloadParams())
    pm.release([0, 0, -5.0], [1.0, 0.0, 0.0])
    p_a, v_a = pm.predict(1.0)
    for _ in range(int(1.0 / 1e-4)):
        pm.step(1e-4)
    err = np.linalg.norm(pm.pos - p_a)
    assert err < 1e-6, f'解析/积分不一致 err={err}'
    t = PayloadModel.fall_time(1.5)
    assert abs(0.5 * 9.81 * t * t - 1.5) < 1e-9
    print(f'payload_model 自测通过 (解析/积分 err={err:.2e}, fall_time(1.5m)={t:.4f}s)')

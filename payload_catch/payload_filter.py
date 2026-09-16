#!/usr/bin/env python3
"""载荷状态估计 —— 线性卡尔曼滤波（纯 Python）。

动机：闭环重规划现在直接吃**含噪、可能有延迟/丢包**的载荷状态测量。
而载荷动力学是已知的（无阻力时是抛体：ṗ=v, v̇=g），所以用一个线性 KF 很自然：
  · 平滑位置噪声；
  · 由位置序列**估计速度**（比有限差分稳得多）；
  · 测量延迟/丢包时**预测外推**（coast），不必等数据。

状态 x = [p(3), v(3)]，重力作为已知输入（非控制量）：
    p_{k+1} = p_k + v_k·dt + ½g·dt²
    v_{k+1} = v_k + g·dt
测量 z = p（3 维，带噪）。

延迟/丢包处理：测量按**自身时间戳** t_meas 进入滤波器（先 predict 到 t_meas 再 update），
所以延迟只是给滤波器更多"外推"时间；丢包期间不 update，下次测量到来时一次性 predict 过去。
对外提供 `estimate_at(t_now)`：把当前估计外推到"现在"，供规划器使用（不改内部状态）。
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


class BallisticKF:
    def __init__(self, g: float = 9.81, q_accel: float = 2.0,
                 meas_sigma: float = 0.05,
                 p0_sigma: float = 1.0, v0_sigma: float = 3.0):
        self.g_vec = np.array([0.0, 0.0, float(g)])
        self.q = float(q_accel)          # 过程噪声：未建模加速度(std, m/s²)，涵盖风/阻力
        self.R = (float(meas_sigma) ** 2) * np.eye(3)
        self._p0_sigma = float(p0_sigma)
        self._v0_sigma = float(v0_sigma)
        self.reset()

    def reset(self):
        self.x = np.zeros(6)
        self.P = np.eye(6) * 1e3
        self.t: Optional[float] = None
        self.initialized = False

    # ------------------------------------------------------------- dynamics
    def _FG(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        F = np.eye(6)
        F[0:3, 3:6] = np.eye(3) * dt
        G = np.zeros(6)
        G[0:3] = 0.5 * self.g_vec * dt * dt
        G[3:6] = self.g_vec * dt
        return F, G

    def _Q(self, dt: float) -> np.ndarray:
        """连续白噪声加速度模型（各轴独立）。"""
        q = self.q * self.q
        Q = np.zeros((6, 6))
        q11, q12, q22 = 0.25 * dt ** 4, 0.5 * dt ** 3, dt * dt
        for a in range(3):
            Q[a, a] = q11 * q
            Q[a, 3 + a] = q12 * q
            Q[3 + a, a] = q12 * q
            Q[3 + a, 3 + a] = q22 * q
        return Q

    def _predict_to(self, t: float):
        if self.t is None:
            return
        dt = t - self.t
        if dt <= 1e-12:
            return
        F, G = self._FG(dt)
        self.x = F @ self.x + G
        self.P = F @ self.P @ F.T + self._Q(dt)
        self.t = t

    # ---------------------------------------------------------------- update
    def _update(self, z: np.ndarray):
        H = np.zeros((3, 6))
        H[0:3, 0:3] = np.eye(3)
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def init_from(self, z: np.ndarray, t: float):
        self.x = np.concatenate([np.asarray(z, float).reshape(3), np.zeros(3)])
        self.P = np.diag([self._p0_sigma ** 2] * 3 + [self._v0_sigma ** 2] * 3)
        self.t = float(t)
        self.initialized = True

    def process(self, z, t_meas: float, v_hint: Optional[np.ndarray] = None):
        """处理一帧测量（z: 3 维位置，t_meas: 测量时刻）。

        v_hint：可选的速度先验（如 A 释放速度），用于首帧初始化，加速收敛。
        """
        z = np.asarray(z, float).reshape(3)
        if not self.initialized:
            self.init_from(z, t_meas)
            if v_hint is not None:
                self.x[3:6] = np.asarray(v_hint, float).reshape(3)
            return
        if t_meas < self.t - 1e-9:
            return                      # 乱序：忽略（等间隔延迟下不应发生）
        self._predict_to(t_meas)
        self._update(z)

    # -------------------------------------------------------------- estimate
    def estimate_at(self, t_now: float) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """把当前估计外推到 t_now（不改内部状态），返回 (p, v)。"""
        if not self.initialized:
            return None
        x = self.x.copy()
        if self.t is not None and t_now > self.t:
            F, G = self._FG(t_now - self.t)
            x = F @ x + G
        return x[0:3].copy(), x[3:6].copy()


class NaiveEstimator:
    """对照组：用最近一次测量位置 + 有限差分速度，再按弹道外推到 now。"""

    def __init__(self, g: float = 9.81):
        self.g_vec = np.array([0.0, 0.0, float(g)])
        self._z = None
        self._t = None
        self._v = np.zeros(3)

    def reset(self):
        self._z = None; self._t = None; self._v = np.zeros(3)

    def process(self, z, t_meas: float, v_hint=None):
        z = np.asarray(z, float).reshape(3)
        if self._z is not None and t_meas > self._t + 1e-9:
            self._v = (z - self._z) / (t_meas - self._t)
        elif self._z is None and v_hint is not None:
            self._v = np.asarray(v_hint, float).reshape(3)
        self._z, self._t = z.copy(), float(t_meas)

    def estimate_at(self, t_now: float):
        if self._z is None:
            return None
        dt = max(0.0, t_now - self._t)
        p = self._z + self._v * dt + 0.5 * self.g_vec * dt * dt
        v = self._v + self.g_vec * dt
        return p, v


if __name__ == '__main__':
    # 自测：真值抛体 + 位置噪声，KF 估计误差应远小于原始测量误差
    rng = np.random.default_rng(0)
    g = 9.81
    p = np.array([0.0, 0.0, -3.0]); v = np.array([1.0, 0.0, 0.0])
    kf = BallisticKF(g=g, q_accel=1.0, meas_sigma=0.05)
    dt = 0.02
    raw_err, kf_err = [], []
    for k in range(60):
        t = k * dt
        v = v + np.array([0, 0, g]) * dt
        p = p + v * dt
        z = p + rng.normal(0, 0.05, 3)
        kf.process(z, t)
        est = kf.estimate_at(t)
        raw_err.append(np.linalg.norm(z - p))
        kf_err.append(np.linalg.norm(est[0] - p))
    print(f'raw pos err mean={np.mean(raw_err):.4f}  KF pos err mean={np.mean(kf_err):.4f}')
    assert np.mean(kf_err) < np.mean(raw_err)
    print('payload_filter 自测通过')

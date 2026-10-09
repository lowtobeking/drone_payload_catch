#!/usr/bin/env python3
"""perception.py —— 视觉载荷感知模型（把"真值+高斯"替身升级为相机模型），纯 Python、可自测。

接收机 B 用一台**朝上**的相机看下落的载荷。相对"真值替身"，相机模型引入：
  · **FOV 门控**：载荷出视场（横向偏离太大/太近）→ 无有效测量；
  · **距离相关横向误差**：`σ_lat ≈ lateral_k · r`（角分辨率 × 距离，r=载荷到相机距离）；
  · **深度误差** `σ_depth`；
  · **丢帧**（dropout）+（可选）姿态耦合。

输出仍是 `/payload/state`（世界 NED 位置），供 B 的估计/控制使用；捕获判定仍用真值（不变）。

用法: python3 -m payload_catch.perception     # 自测
"""
from __future__ import annotations

import math

import numpy as np


class CameraModel:
    def __init__(self, fov_deg: float = 60.0, lateral_k: float = 0.005,
                 depth_sigma: float = 0.02, dropout: float = 0.0,
                 boresight=(0.0, 0.0, -1.0), rng: np.random.Generator | None = None):
        self.half = math.radians(fov_deg) / 2.0
        self.lateral_k = lateral_k          # rad ≈ 角分辨率
        self.depth_sigma = depth_sigma
        self.dropout = dropout
        self.bore = np.asarray(boresight, float)
        self.bore /= (np.linalg.norm(self.bore) + 1e-12)
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self._last = None

    def in_fov(self, p_pay: np.ndarray, p_b: np.ndarray) -> bool:
        rel = np.asarray(p_pay, float) - np.asarray(p_b, float)
        r = float(np.linalg.norm(rel))
        if r < 1e-6:
            return True
        cosang = float(np.dot(rel / r, self.bore))
        return cosang >= math.cos(self.half)

    def measure(self, p_pay: np.ndarray, p_b: np.ndarray):
        """返回测量位置（世界 NED）或 None（出视场/丢帧）。"""
        rel = np.asarray(p_pay, float) - np.asarray(p_b, float)
        r = float(np.linalg.norm(rel))
        if not self.in_fov(p_pay, p_b):
            return None
        if self.dropout > 0.0 and self.rng.random() < self.dropout:
            return None
        p = np.asarray(p_pay, float).copy()
        # 横向误差（相机 xy 平面）——距离相关
        sl = self.lateral_k * max(r, 1e-3)
        p[0] += self.rng.normal(0.0, sl)
        p[1] += self.rng.normal(0.0, sl)
        p[2] += self.rng.normal(0.0, self.depth_sigma)
        return p

    def track(self, p_pay: np.ndarray, p_b: np.ndarray):
        """带"保持"的测量：无效时沿用上一有效帧（模拟短时丢失）。"""
        m = self.measure(p_pay, p_b)
        if m is not None:
            self._last = m
        return self._last


def _selftest():
    ok = True
    rng = np.random.default_rng(1)
    cam = CameraModel(fov_deg=60.0, lateral_k=0.005, depth_sigma=0.02, rng=rng)
    p_b = np.array([0.0, 0.0, -3.5])
    # 1) 正上方（视场内）→ 有测量且接近真值
    p_pay = np.array([0.0, 0.0, -4.0])
    m = cam.measure(p_pay, p_b)
    ok1 = m is not None and np.linalg.norm(m - p_pay) < 0.1
    print(f'[1] 视场内测量 err={None if m is None else np.linalg.norm(m-p_pay):.4f}  '
          f'{"OK" if ok1 else "FAIL"}')
    ok &= ok1

    # 2) FOV 外（横向偏很远、贴近高度）→ None
    p_out = np.array([3.0, 0.0, -3.6])       # 横向 3m、竖直 0.1m → 角度很大
    m2 = cam.measure(p_out, p_b)
    ok2 = m2 is None
    print(f'[2] 视场外(横向3m/竖直0.1m) → {"None OK" if ok2 else "有误" if m2 is not None else "FAIL"}')
    ok &= ok2

    # 3) 距离相关：远处误差 > 近处误差
    errs_far, errs_near = [], []
    for _ in range(400):
        f = cam.measure(np.array([0.0, 0.0, -5.0]), p_b)
        n = cam.measure(np.array([0.0, 0.0, -3.6]), p_b)
        if f is not None:
            errs_far.append(np.linalg.norm(f - np.array([0, 0, -5.0])))
        if n is not None:
            errs_near.append(np.linalg.norm(n - np.array([0, 0, -3.6])))
    ok3 = np.mean(errs_far) > np.mean(errs_near)
    print(f'[3] 距离相关: 远 {np.mean(errs_far):.4f} > 近 {np.mean(errs_near):.4f}  '
          f'{"OK" if ok3 else "FAIL"}')
    ok &= ok3

    # 4) 丢帧 → 保持
    cam4 = CameraModel(dropout=1.0, rng=np.random.default_rng(0))
    ok4 = cam4.measure(p_pay, p_b) is None and cam4.track(p_pay, p_b) is None
    print(f'[4] 全丢帧 → {"OK" if ok4 else "FAIL"}')
    ok &= ok4

    print('perception 自测', '通过 ✅' if ok else '失败 ❌')
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)

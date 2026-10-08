#!/usr/bin/env python3
"""relnav.py —— 相对定位（真机 RTK/UWB/视觉接入层），纯 Python、可离线自测。

目标：把"仿真替身"换成**真实相对定位**：
  · 仿真：a_node 广播真值 /drone_a/state，b_node 在其上加噪声（替身）；
  · 真机：本模块把两机 RTK（大地坐标）+ 姿态 + 杆臂，换算成 A 在 B 的 PX4 本地 NED 中的状态，
    发布到**同一个** /drone_a/state（b_node 的 rel_* 噪声参数置 0 即可，接口不变）。

关键点：
  1. **大地坐标→本地 NED**（小基线平面近似，两机相距 <~1km 足够准）；
  2. **杆臂补偿**：RTK 天线 ≠ 末端/漏斗位置，需用姿态把机体系偏移旋到世界系；
  3. **相对化**：RTK 的公共原点未知也没关系——相对向量 (p_A_ref − p_B_ref) 与原点无关，
     故输出 p_A_world = p_B_px4_world + (p_A_ref − p_B_ref)，b_node 相减后即得真实相对量。

用法:
  python3 -m payload_catch.relnav          # 自测
"""
from __future__ import annotations

import math

import numpy as np

R_EARTH = 6378137.0     # WGS84 长半轴 (m)


# ---------------------------------------------------------------- 坐标/旋转
def geodetic_to_ned(lat: float, lon: float, alt: float,
                    lat0: float, lon0: float, alt0: float) -> np.ndarray:
    """大地坐标(deg,deg,m) → 以 (lat0,lon0,alt0) 为原点的本地 NED (m)。

    小基线平面近似：dn=Δlat·R, de=Δlon·R·cos(lat0), dd=-(Δalt)。
    """
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    dn = dlat * R_EARTH
    de = dlon * R_EARTH * math.cos(math.radians(lat0))
    dd = -(alt - alt0)
    return np.array([dn, de, dd], float)


def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """机体系(NED, x-前 y-右 z-下) → 世界系(NED) 的旋转矩阵 R_wb。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) Ry(pitch) Rx(roll)（NED 约定）
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], float)


def lever_arm_world(offset_body: np.ndarray, roll: float,
                    pitch: float, yaw: float) -> np.ndarray:
    """把机体系偏移(如 天线→末端)旋到世界 NED。"""
    return rpy_to_R(roll, pitch, yaw) @ np.asarray(offset_body, float).reshape(3)


def compensate_lever_arm(antenna_ned: np.ndarray, offset_body: np.ndarray,
                         roll: float, pitch: float, yaw: float) -> np.ndarray:
    """由天线点的世界 NED 位置，算参考点(末端/漏斗)的世界 NED 位置。"""
    return np.asarray(antenna_ned, float).reshape(3) + \
        lever_arm_world(offset_body, roll, pitch, yaw)


# ------------------------------------------------------------ 相对化 / 发布
def a_state_for_b(p_a_ref_ned: np.ndarray, p_b_ref_ned: np.ndarray,
                  b_px4_world: np.ndarray) -> np.ndarray:
    """输出 A 的 /drone_a/state 位置（对齐 b_node 的 pos_world 系）。

    相对向量与 RTK 公共原点无关：p_A_world = b_px4_world + (p_A_ref − p_B_ref)。
    """
    rel = np.asarray(p_a_ref_ned, float).reshape(3) - np.asarray(p_b_ref_ned, float).reshape(3)
    return np.asarray(b_px4_world, float).reshape(3) + rel


class VelEstimator:
    """由位置时间序列做有限差分估速（RTK 无速度时的兜底）。"""

    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self._prev = None      # (t, p)

    def update(self, t: float, p: np.ndarray):
        p = np.asarray(p, float).reshape(3)
        if self._prev is None:
            self._prev = (t, p)
            return np.zeros(3)
        t0, p0 = self._prev
        dt = max(t - t0, 1e-3)
        v = (p - p0) / dt
        self._prev = (t, p)
        return v


def inject_surrogate(p_true: np.ndarray, v_true: np.ndarray, sigma: float,
                     rng: np.random.Generator, bias: np.ndarray | None = None) -> tuple:
    """仿真替身：给真值加白噪声/偏置（供离线或 sim 模式对照）。"""
    p = np.asarray(p_true, float).reshape(3).copy()
    if bias is not None:
        p = p + np.asarray(bias, float).reshape(3)
    if sigma > 0:
        p = p + rng.normal(0.0, sigma, 3)
    return p, np.asarray(v_true, float).reshape(3).copy()


# ------------------------------------------------------------------ 自测
def _selftest():
    ok = True
    # 1) 大地坐标 → NED：相对距离
    lat0, lon0, alt0 = 30.0, 120.0, 10.0
    e = geodetic_to_ned(lat0 + 0.001, lon0, alt0 + 5.0, lat0, lon0, alt0)
    dn_exp = math.radians(0.001) * R_EARTH            # 0.001° 纬度 ≈ 111.32 m
    dn_err = abs(e[0] - dn_exp)
    ok &= dn_err < 0.01
    ok &= abs(e[1]) < 1e-6 and abs(e[2] + 5.0) < 1e-9
    print(f'[1] geodetic→NED: north={e[0]:.2f}m (期望 {dn_exp:.2f}) '
          f'down={e[2]:.1f}m  {"OK" if ok else "FAIL"}')

    # 2) 杆臂：yaw=90°，机体系 [1,0,0] → 世界 [0,1,0]
    w = lever_arm_world([1.0, 0.0, 0.0], 0.0, 0.0, math.pi / 2)
    ok2 = abs(w[0]) < 1e-9 and abs(w[1] - 1.0) < 1e-9
    print(f'[2] 杆臂 yaw90 [1,0,0]→[{w[0]:.2f},{w[1]:.2f},{w[2]:.2f}]  {"OK" if ok2 else "FAIL"}')
    ok &= ok2

    # 3) 抵消公共原点：RTK 加一个大偏置，相对量不变
    b_world = np.array([2.0, -1.0, -3.5])
    off = np.array([1000.0, -500.0, 30.0])          # RTK 公共原点偏置（任意）
    a_ref = np.array([1.0, 2.0, -4.0]) + off
    b_ref = np.array([0.0, 0.0, -3.5]) + off
    pA = a_state_for_b(a_ref, b_ref, b_world)
    rel = pA - b_world
    ok3 = np.allclose(rel, [1.0, 2.0, -0.5])
    print(f'[3] 原点抵消: rel={rel} (期望 [1,2,-0.5])  {"OK" if ok3 else "FAIL"}')
    ok &= ok3

    # 4) 杆臂补偿 + 姿态
    ant = np.array([0.0, 0.0, -5.0])
    ref = compensate_lever_arm(ant, [0.1, 0.0, 0.0], 0.0, 0.0, 0.0)
    ok4 = abs(ref[0] - 0.1) < 1e-9
    print(f'[4] 杆臂补偿 x+0.1 → {ref}  {"OK" if ok4 else "FAIL"}')
    ok &= ok4

    print('relnav 自测', '通过 ✅' if ok else '失败 ❌')
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)

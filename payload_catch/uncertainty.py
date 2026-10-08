#!/usr/bin/env python3
"""uncertainty.py —— 交接用的**相对不确定度模型**（修当前"绝对 σ 当相对 σ"的保守/错误）。

问题：现有代码把 A 的**绝对**位置误差（`eph`）或 B 的绝对 σ 直接当作交接的**相对** σ，
     忽略了 (a) 两机公共误差（GNSS/RTK 公共项）在相减时**抵消**；(b) 杆臂×姿态的额外误差。
     这会把 σ 估大（实测 σ_used≈0.15m），使释放闸过度保守。

正确模型：相对位置误差 `e_rel = e_A − e_B`（同一公共系），各向同性标量 σ：
    σ_rel² = σ_A² + σ_B² − 2·ρ·σ_A·σ_B          （ρ=公共误差相关系数；独立→ρ=0）
           + (|l_A|·σ_θA)² + (|l_B|·σ_θB)²      （杆臂×姿态误差，l=天线→参考点）
           + σ_meas²                            （相对测量/滤波残差：噪声+迟滞+时同步）

两种架构：
  · absolute_broadcast：B 用 A 广播的**绝对**状态 → 用上式（含 σ_A、σ_B、ρ）；
  · relative_sensor   ：B 用**相对**传感器（RTK 双差/UWB/视觉）→ σ = 传感器相对 σ（与绝对无关）。

用法:
  python3 -m payload_catch.uncertainty     # 自测
"""
from __future__ import annotations

import math


def relative_sigma(sigma_a: float, sigma_b: float, rho: float = 0.0,
                   lever_a: float = 0.0, sigma_att_a: float = 0.0,
                   lever_b: float = 0.0, sigma_att_b: float = 0.0,
                   sigma_meas: float = 0.0) -> float:
    """相对位置误差 σ（米）。

    sigma_a/b : A、B 各自绝对位置误差 σ；
    rho       : 公共误差相关系数 ∈ [0,1]（同源 GNSS/RTK nearby → 大）；
    lever_*   : 天线→参考点 杆臂长度（机体系，标量）；
    sigma_att_*: 姿态角误差（rad）；
    sigma_meas: 相对测量/估计残差 σ。
    """
    var = sigma_a ** 2 + sigma_b ** 2 - 2.0 * rho * sigma_a * sigma_b
    var = max(var, 0.0)
    var += (abs(lever_a) * sigma_att_a) ** 2
    var += (abs(lever_b) * sigma_att_b) ** 2
    var += sigma_meas ** 2
    return math.sqrt(var)


def absolute_sigma_naive(sigma_a: float, sigma_b: float,
                         lever_a: float = 0.0, lever_b: float = 0.0) -> float:
    """当前做法（错误/保守）：把绝对 σ 直接相加当相对 σ（等价 ρ=0 且不含测量项）。"""
    return math.sqrt(sigma_a ** 2 + sigma_b ** 2)


def common_mode_gain(sigma_a: float, sigma_b: float, rho: float) -> float:
    """公共误差抵消带来的 σ 降低倍数（相对 σ / 独立相加 σ）。"""
    base = sigma_a ** 2 + sigma_b ** 2
    if base <= 0:
        return 1.0
    return math.sqrt(max(base - 2.0 * rho * sigma_a * sigma_b, 0.0) / base)


def lever_attitude_sigma(lever: float, sigma_att: float) -> float:
    """杆臂×姿态引起的横向位置误差（保守取 |l|·σθ）。"""
    return abs(lever) * sigma_att


def sigma_rel_for_architecture(sigma_a: float, sigma_b: float, rho: float,
                               sensor_sigma: float, use_relative_sensor: bool,
                               **kw) -> float:
    """按架构选择：相对传感器时用 sensor_sigma（忽略绝对），否则用绝对模型。"""
    if use_relative_sensor:
        return relative_sigma(0.0, 0.0, 0.0, sigma_meas=sensor_sigma, **kw)
    return relative_sigma(sigma_a, sigma_b, rho, **kw)


def _selftest():
    ok = True
    # 1) 独立（ρ=0）：平方和
    s = relative_sigma(0.15, 0.15, rho=0.0)
    ok &= abs(s - math.sqrt(0.045)) < 1e-9
    print(f'[1] ρ=0: σ_rel={s:.4f} (期望 {math.sqrt(0.045):.4f})  {"OK" if ok else "FAIL"}')

    # 2) 完全相关（ρ=1, σ 相等）：抵消为 0
    s2 = relative_sigma(0.15, 0.15, rho=1.0)
    ok2 = abs(s2) < 1e-9
    print(f'[2] ρ=1: σ_rel={s2:.4f} → 0  {"OK" if ok2 else "FAIL"}')
    ok &= ok2

    # 3) 杆臂×姿态：l=0.2m, σθ=0.05rad → 0.01m
    s3 = relative_sigma(0, 0, lever_b=0.2, sigma_att_b=0.05)
    ok3 = abs(s3 - 0.01) < 1e-12
    print(f'[3] 杆臂: σ_rel={s3:.4f} (期望 0.01)  {"OK" if ok3 else "FAIL"}')
    ok &= ok3

    # 4) 公共抵消倍数：0.15,0.15,ρ=0.8 → 0.15*sqrt(0.2*? ) 计算
    g = common_mode_gain(0.15, 0.15, 0.8)
    print(f'[4] 公共抵消倍数(ρ=0.8)={g:.3f} → σ_rel={0.15*math.sqrt(2)*g:.4f}')
    ok &= (g < 1.0)

    # 5) 架构选择
    sr = sigma_rel_for_architecture(0.15, 0.15, 0.7, sensor_sigma=0.03,
                                    use_relative_sensor=True)
    ok5 = abs(sr - 0.03) < 1e-12
    print(f'[5] 相对传感器架构: σ_rel={sr:.3f} (忽略绝对)  {"OK" if ok5 else "FAIL"}')
    ok &= ok5

    print('uncertainty 自测', '通过 ✅' if ok else '失败 ❌')
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)

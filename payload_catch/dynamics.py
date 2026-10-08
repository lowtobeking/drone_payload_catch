#!/usr/bin/env python3
"""dynamics.py —— 四旋翼【聚合动力学限幅】（把姿态/推力约束纳入规划与安全），纯 Python、可自测。

现有规划器把执行层当作裸双积分器（|a|≤a_max）。真实四旋翼的加速度受**倾角**与**推力**共同约束：

  NED（z 向下），设倾角 θ、总推力 T、单机质量 m：
      a_x = (T/m)·sinθ          （水平）
      a_z = g − (T/m)·cosθ      （竖直，向下为正）
  ⇒ tanθ = a_x / (g − a_z)      （倾角约束：θ≤θ_max ⇒ a_x ≤ (g−a_z)·tanθ_max）
  ⇒ (T/m) = √(a_x² + (g−a_z)²) ≤ T_max/m   （推力约束）

由此得到可行加速度集（椭圆/锥）。关键结论：
  · **水平权限随下潜增大而减小**：a_x ≤ (g−a_z)·tanθ_max（俯冲时更难横移）；
  · **向下最多 a_z ≤ g**（推力不能为负）；
  · 悬停水平权限 = g·tanθ_max（θ=30°→5.66、40°→8.28 m/s²）。

用法: python3 -m payload_catch.dynamics     # 自测
"""
from __future__ import annotations

import math


def hover_horizontal_accel(g: float, tilt_max: float) -> float:
    """悬停（a_z=0）时的最大水平加速度 = g·tanθ_max。"""
    return g * math.tan(tilt_max)


def horizontal_accel_limit(a_z: float, g: float, tilt_max: float) -> float:
    """给定竖直加速度 a_z（NED 下为正）时的最大水平加速度 (g−a_z)·tanθ_max。"""
    return max(0.0, g - a_z) * math.tan(tilt_max)


def tilt_for_accel(a_x: float, a_z: float, g: float) -> float:
    """达到该 (a_x,a_z) 所需倾角 θ=atan2(a_x, g−a_z)。"""
    return math.atan2(a_x, g - a_z)


def thrust_over_mass(a_x: float, a_z: float, g: float) -> float:
    """所需的 T/m。"""
    return math.hypot(a_x, g - a_z)


def feasible_accel(a_x: float, a_z: float, g: float, tilt_max: float,
                   tmax_over_m: float) -> bool:
    """该加速度是否满足倾角+推力约束。"""
    if a_z > g + 1e-9:                      # 向下不能超重力
        return False
    if a_z < g - tmax_over_m - 1e-9:        # 向上受推力上限
        return False
    if abs(a_z) <= g and abs(tilt_for_accel(a_x, a_z, g)) > tilt_max + 1e-9:
        return False
    if thrust_over_mass(a_x, a_z, g) > tmax_over_m + 1e-9:
        return False
    return True


def max_axis_accel(a_z: float, g: float, tilt_max: float, tmax_over_m: float) -> float:
    """在给定 a_z 下，单轴水平可达的最大 |a_x|（倾角与推力同时满足）。"""
    ax_tilt = horizontal_accel_limit(a_z, g, tilt_max)
    # 推力约束：a_x² + (g-a_z)² ≤ (T/m)²
    rem = tmax_over_m ** 2 - (g - a_z) ** 2
    ax_thrust = math.sqrt(rem) if rem > 0 else 0.0
    return min(ax_tilt, ax_thrust)


def _selftest():
    g = 9.81
    ok = True
    # 1) 悬停水平权限
    a30 = hover_horizontal_accel(g, math.radians(30))
    ok1 = abs(a30 - 5.664) < 0.01
    print(f'[1] 悬停 θ=30°: a_h={a30:.2f} m/s² (期望 5.66)  {"OK" if ok1 else "FAIL"}')
    ok &= ok1

    # 2) 水平权限随下潜减小
    a0 = horizontal_accel_limit(0.0, g, math.radians(30))
    a5 = horizontal_accel_limit(5.0, g, math.radians(30))
    ok2 = a5 < a0 and abs(a5 - (g - 5.0) * math.tan(math.radians(30))) < 1e-9
    print(f'[2] 下潜时水平权限: a_z=0→{a0:.2f}, a_z=5→{a5:.2f}  {"OK" if ok2 else "FAIL"}')
    ok &= ok2

    # 3) 可行性：悬停 30° 内 5 m/s² 可行；10 m/s² 不可行（倾角/推力）
    f1 = feasible_accel(5.0, 0.0, g, math.radians(30), 2.5 * g)
    f2 = feasible_accel(10.0, 0.0, g, math.radians(30), 2.5 * g)
    ok3 = f1 and not f2
    print(f'[3] 可行(5)={f1} 可行(10)={f2}  {"OK" if ok3 else "FAIL"}')
    ok &= ok3

    # 4) 向下不能超重力
    ok4 = (not feasible_accel(0.0, g + 1.0, g, math.radians(30), 2.5 * g))
    print(f'[4] a_z>g 不可行: {"OK" if ok4 else "FAIL"}')
    ok &= ok4

    # 5) 项目 a_max=6 对应倾角
    th = math.degrees(math.atan(6.0 / g))
    print(f'[5] a_max=6 m/s² ↔ 倾角 ≈ {th:.1f}°（悬停）')
    ok &= (25 < th < 35)

    print('dynamics 自测', '通过 ✅' if ok else '失败 ❌')
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)

#!/usr/bin/env python3
"""impact.py —— 捕获【接触/冲击】对接收机 B 的影响与可恢复性（把 contact 纳入安全叙述）。

现有安全保证只到"飞行中不碰撞"，**接触瞬间**是几何判据。真机必须回答：
  1. 冲击冲量/峰值力多大？B 的末端/结构能否承受、能否把姿态拉回来？
  2. 偏心撞击产生的力矩 → 角速度，B 的力矩权限能否在阈值内恢复？
  3. 带载后推力是否还有余量（否则接住也悬不住）？

模型（保守）：
  冲量   J  = m_p · v_rel                         (N·s)
  峰值力 F  ≈ J²/(2·m_p·s) = ½·m_p·v_rel²/s        (缓冲行程 s)
  角速度 ω ≈ J · d_off / I_B                       (偏心 d_off)
  可恢复  τ_max·Δt ≥ I_B·ω  ⇒  ω ≤ τ_max/(I_B/T_rec)，取 ω ≤ ω_max
  推力余量 = T_max − (m_B+m_p)·g ≥ 0

用法: python3 -m payload_catch.impact     # 自测
"""
from __future__ import annotations

import math

G = 9.81


def impulse(m_p: float, v_rel: float) -> float:
    return m_p * v_rel


def energy(m_p: float, v_rel: float) -> float:
    return 0.5 * m_p * v_rel * v_rel


def peak_force(m_p: float, v_rel: float, stroke: float) -> float:
    """缓冲行程 s 上的平均力（能量守恒 ½mv²=F·s）。"""
    return energy(m_p, v_rel) / max(stroke, 1e-9)


def angular_velocity(m_p: float, v_rel: float, d_off: float, I_b: float) -> float:
    """偏心 d_off 撞击后的近似角速度。"""
    return impulse(m_p, v_rel) * abs(d_off) / max(I_b, 1e-9)


def recoverable(m_p: float, v_rel: float, d_off: float, I_b: float,
                tau_max: float, t_rec: float = 0.2, omega_max: float = 3.0) -> dict:
    """B 能否在 t_rec 内把偏心冲击的角速度拉回。返回 ω 与判定。"""
    w = angular_velocity(m_p, v_rel, d_off, I_b)
    w_cap = tau_max / max(I_b, 1e-9) * t_rec
    return dict(omega=w, omega_cap=w_cap, omega_max=omega_max,
                ok=(w <= min(w_cap, omega_max)))


def thrust_margin(m_b: float, m_p: float, tmax: float, g: float = G) -> float:
    """带载推力余量 (N)：T_max − (m_B+m_p)g；≥0 才能悬停。"""
    return tmax - (m_b + m_p) * g


def can_hover_carry(m_b: float, m_p: float, tmax: float, g: float = G) -> bool:
    return thrust_margin(m_b, m_p, tmax, g) >= 0.0


def _selftest():
    ok = True
    # 1) 冲量/能量
    ok &= abs(impulse(0.1, 3.7) - 0.37) < 1e-9
    print(f'[1] J(0.1kg,3.7m/s)={impulse(0.1,3.7):.3f} N·s  {"OK"}')

    # 2) 峰值力：0.1kg, 3.7m/s, 行程 0.05m → ½*0.1*13.7/0.05
    F = peak_force(0.1, 3.7, 0.05)
    print(f'[2] 峰值力(行程0.05m)={F:.1f} N (~{F/(1.5*G):.1f}×B自重)')

    # 3) 偏心角速度
    w = angular_velocity(0.1, 3.7, 0.1, 0.02)
    print(f'[3] ω(偏心0.1m,I=0.02)={w:.2f} rad/s')

    # 4) 可恢复性：100g 可、2kg 不可
    r1 = recoverable(0.1, 3.7, 0.1, 0.02, tau_max=4.0)
    r2 = recoverable(2.0, 3.7, 0.1, 0.02, tau_max=4.0)
    ok &= r1['ok'] and not r2['ok']
    print(f'[4] 可恢复: 100g={r1["ok"]} 2kg={r2["ok"]}  {"OK" if ok else "FAIL"}')

    # 5) 推力余量：x500 1.5kg, T_max=2.0*(1.5+?)g 用 30N
    m_b, tmax = 1.5, 30.0
    ok5 = can_hover_carry(m_b, 0.1, tmax) and not can_hover_carry(m_b, 5.0, tmax)
    print(f'[5] 带载悬停: +0.1kg={can_hover_carry(m_b,0.1,tmax)} +5kg={can_hover_carry(m_b,5.0,tmax)}  '
          f'{"OK" if ok5 else "FAIL"}')
    ok &= ok5

    print('impact 自测', '通过 ✅' if ok else '失败 ❌')
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)

#!/usr/bin/env python3
"""keepout.py —— 速度级 CBF 防碰（C5）与安全投影（无 ROS 依赖）。

  h(r)=‖r‖²−d_safe²，  r=p_A−p_B
  条件  ḣ+αh ≥ 0  ⇔  r·v_B ≤ r·v_A + (α/2)h          （B 的约束）
  ⇒ 指数 CBF：h(t) ≥ h(0)e^{−αt} ⇒ ‖r(t)‖ ≥ d_safe·e^{−αt/2}（永不碰撞）
  可行条件：  c ≥ −‖r‖·v_max
"""
from __future__ import annotations

import numpy as np


def cbf_bound(r, v_other, h, alpha):
    """B 的约束右端 c： r·v_B ≤ c。"""
    return float(np.dot(r, v_other)) + 0.5 * alpha * h


def project_cbf(v_nom, r, c, v_max):
    """精确投影到 {r·v ≤ c} ∩ {‖v‖≤v_max}（闭式）。"""
    v = np.asarray(v_nom, float).copy()
    n = float(np.linalg.norm(v))
    if n > v_max and n > 1e-12:
        v *= v_max / n
    rn = float(np.linalg.norm(r))
    if rn < 1e-12:
        return v
    if float(r @ v) <= c:              # 已满足
        return v
    e = r / rn
    c_e = c / rn
    if c_e < -v_max:                   # 不可行：取最接近的球面点（尽力）
        return -v_max * e
    perp = v - float(v @ e) * e
    w_max = float(np.sqrt(max(0.0, v_max ** 2 - c_e ** 2)))
    nw = float(np.linalg.norm(perp))
    w = perp if (nw <= w_max or nw < 1e-12) else perp * (w_max / nw)
    return c_e * e + w

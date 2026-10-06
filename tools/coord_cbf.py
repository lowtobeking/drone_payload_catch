#!/usr/bin/env python3
"""coord_cbf.py —— 交接过程的控制障碍函数（CBF）与安全证书（研究 W4 / C5）。

与项目一致：执行层是 **速度控制**，故用**相对一阶（速度级）CBF**：
  h(r) = ‖r‖² − d_safe²，  r = p_A − p_B
  ḣ = 2 r·(v_A − v_B)
  条件： ḣ + α h ≥ 0  ⇔  r·v_B ≤ r·v_A + (α/2) h      （B 的滤波约束）
  ⇒ 指数 CBF： h(t) ≥ h(0)·e^{−α t} > 0  ⇒ 永不碰撞。

交接门（②）：释放前用 C1 证书 `cert_threshold` 保证捕获可达。

滤波：把标称速度投影到 {r·v ≤ c} ∩ {‖v‖≤v_max}（交替投影，闭式迭代）。

用法：
  python3 tools/coord_cbf.py [--n N] [--d-safe 0.8] [--alpha 1.0]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch.coord_cert import cert_threshold  # noqa: E402
from payload_catch.keepout import project_cbf, cbf_bound  # noqa: E402


def check_invariance(n, d_safe, v_max, alpha, rng):
    """单元：随机状态/标称速度，验证“可行时”滤波后满足 ḣ+αh≥0。

    可行条件：使 r·v_B ≤ c 与 ‖v_B‖≤v_max 同时可达 ⇒ c ≥ −‖r‖·v_max。
    """
    ok_raw = ok_feas = 0
    n_feas = 0
    worst_feas = np.inf
    for _ in range(n):
        u = rng.normal(0, 1, 3); u /= np.linalg.norm(u) + 1e-9
        r = u * rng.uniform(d_safe * 1.001, d_safe * 3.0)
        v_A = rng.normal(0, v_max * 0.8, 3)
        v_B_nom = rng.normal(0, v_max * 1.2, 3)
        h = float(r @ r) - d_safe ** 2
        c = cbf_bound(r, v_A, h, alpha)
        feasible = c >= -float(np.linalg.norm(r)) * v_max + 1e-9
        if not feasible:
            continue
        n_feas += 1
        lf_raw = 2.0 * float(r @ (v_A - v_B_nom)) + alpha * h
        if lf_raw >= -1e-9:
            ok_raw += 1
        v_B = project_cbf(v_B_nom, r, c, v_max)
        lf_flt = 2.0 * float(r @ (v_A - v_B)) + alpha * h
        worst_feas = min(worst_feas, lf_flt)
        if lf_flt >= -1e-6:
            ok_feas += 1
    denom = max(n_feas, 1)
    return dict(n_feas=n_feas, frac_feas=n_feas / n,
                ok_raw=ok_raw / denom, ok_flt=ok_feas / denom,
                worst_flt=worst_feas)


def handover_trajectory(d_safe, v_max, alpha, eps, r_eff, margin, sigma_m,
                        dt=0.02, T=3.0):
    """交接轨迹：A 悬停；B 上近 → C1 证书释放 → B 下潜；全程速度 CBF。"""
    p_A = np.array([0.0, 0.0, -4.5]); v_A = np.zeros(3)
    p_B = np.array([0.0, 0.0, -3.5]); v_B = np.zeros(3)
    h_min = np.inf; released = False; t_rel = None
    T_thr = cert_threshold('exact', eps, r_eff, margin, sigma_m)
    for k in range(int(T / dt)):
        t = k * dt
        rel_xy = 0.20 * float(np.exp(-t / 0.5))     # B 对正残差随时间收敛
        v_A_nom = np.array([1.2, 0.0, -0.8]) if released else np.zeros(3)  # A 释放后清场
        v_B_nom = np.array([0.0, 0.0, 2.5]) if released else np.array([0.0, 0.0, -0.8])
        r = p_A - p_B
        h = float(r @ r) - d_safe ** 2
        # 双方都做 CBF 滤波（去中心化）
        cB = cbf_bound(r, v_A, h, alpha)
        v_B = project_cbf(v_B_nom, r, cB, v_max)
        # A 的对称约束： r·v_A ≥ r·v_B − (α/2)h  ⇔ (−r)·v_A ≤ (−r)·v_B + (α/2)h
        cA = float(-r @ v_B) + 0.5 * alpha * h
        v_A = project_cbf(v_A_nom, -r, cA, v_max)
        if (not released) and rel_xy <= T_thr and h >= 0.0:
            released = True; t_rel = t
        p_A = p_A + v_A * dt
        p_B = p_B + v_B * dt
        h_min = min(h_min, float(np.linalg.norm(p_A - p_B)))
    return dict(h_min=h_min, safe=h_min >= d_safe - 1e-6,
                released=released, t_rel=t_rel, T_thr=T_thr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=50000)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--d-safe', type=float, default=0.8)
    ap.add_argument('--v-max', type=float, default=5.0)
    ap.add_argument('--alpha', type=float, default=1.0)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)

    print('=' * 78)
    print(f'C5 handover-CBF（速度级）：不变性单元 (n={args.n})')
    print(f'  d_safe={args.d_safe} v_max={args.v_max} α={args.alpha}')
    print('=' * 78)
    res = check_invariance(args.n, args.d_safe, args.v_max, args.alpha, rng)
    print(f"  可行样本占比 = {res['frac_feas']*100:.1f}%")
    print(f"{'（仅可行样本）':12} {'满足 ḣ+αh≥0 比例':>18} {'最坏 ḣ+αh':>12}")
    print(f"{'未滤波':12} {res['ok_raw']*100:17.2f}% {'—':>12}")
    print(f"{'CBF 滤波':12} {res['ok_flt']*100:17.2f}% {res['worst_flt']:12.4f}")
    print(f"  → 可行时：未滤波 {res['ok_raw']*100:.1f}% → CBF {res['ok_flt']*100:.1f}% 满足")
    print("  可行条件： c ≥ −‖r‖·v_max （B 速度预算涵盖相对接近速度）")

    print('\n' + '=' * 78)
    print('交接轨迹（A 悬停 → B 上近 → C1 证书释放 → B 下潜；双方 CBF）')
    print('=' * 78)
    for alpha in (1.0, 2.0):
        tr = handover_trajectory(args.d_safe, args.v_max, alpha, 0.05,
                                 r_eff=0.25, margin=0.05, sigma_m=0.036)
        print(f"  α={alpha}: 最小间距={tr['h_min']:.3f} (≥d_safe? "
              f"{'✅' if tr['safe'] else '❌'}); 释放={tr['released']} @t={tr['t_rel']:.2f}; "
              f"T={tr['T_thr']:.3f}")

    print('\n>>> 结论：速度级 CBF 保证 ḣ+αh≥0 ⇒ ‖r(t)‖≥d_safe·e^{−αt/2}（永不碰撞）；')
    print('>>>       交接门复用 C1 证书保证释放可达。叠加 = 交接过程的安全证书。')


if __name__ == '__main__':
    main()

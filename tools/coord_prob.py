#!/usr/bin/env python3
"""coord_prob.py —— 释放决策的【概率模型 + 捕获概率证书】（研究 W2 / C1）。

把"空中交接的释放时刻决策"抽象为一个**二维脱靶的随机模型**，给出：

  1. 证书阈值 k(ε)：水平脱靶 ~ N(μ, σ_m² I₂) 时
         P(‖m‖ > R) ≤ exp(−(R−‖μ‖)² / (2σ_m²))     （非中心卡方的 Chernoff 界）
     ⇒ 取  ‖μ‖ + k(ε)·σ_m ≤ R − margin  保证 P(capture) ≥ 1−ε，
        其中 k(ε) = sqrt(2·ln(1/ε))。
  2. 释放策略：估计到 ‖δ̂‖ + k·σ_m ≤ eff_r − margin 才释放（否则继续等/中止）。
  3. 对比：certificate(k=k(ε)) vs heuristic(k=1 旧行为)；authority vs direct；不同 σ/延迟/ε/漏斗。

关键结论（可复现）：项目现用的 k=1 对应 ε≈0.61（几乎无保证）；k(ε) 才能给出标定保证。

用法：
  python3 tools/coord_prob.py                 # 全部表格
  python3 tools/coord_prob.py --n 400000
  python3 tools/coord_prob.py --seed 1
"""
from __future__ import annotations

import argparse
import math
import numpy as np


def k_cert(eps: float) -> float:
    """由 Chernoff 界导出的证书系数：P(‖m‖>R) ≤ exp(−(R−‖μ‖)²/(2σ²)) ⇒ k=√(2 ln(1/ε))。"""
    return math.sqrt(2.0 * math.log(1.0 / eps))


def eps_of_k(k: float) -> float:
    """k=1 对应的保证水平 ε = exp(−k²/2)（用于揭示旧启发式的实际保证）。"""
    return math.exp(-0.5 * k * k)


def sample(n: int, sigma0: float, sigma_e: float, bias: float,
           sigma_track: float, rng: np.random.Generator):
    """返回 (δ̂, m)：
       δ_true ~ N(0, σ0²I₂)          真实释放时水平残差（B 对正的真实误差）
       e      ~ N(bias·x̂, σ_e²I₂)    估计误差（相对定位/通信）
       δ̂ = δ_true + e                决策所用估计
       w      ~ N(0, σ_track²I₂)     释放后 B 的跟踪残差
       m = δ_true + w                真实脱靶（水平）
    """
    d_true = rng.normal(0.0, sigma0, (n, 2))
    e = rng.normal(0.0, sigma_e, (n, 2))
    e[:, 0] += bias
    d_hat = d_true + e
    w = rng.normal(0.0, sigma_track, (n, 2))
    m = d_true + w
    return d_hat, m


def sim_policy(n, r_eff, margin, sigma0, sigma_e, bias, sigma_track,
               k, rng):
    """给定证书系数 k，模拟一次"释放策略 → 结果"。
    返回 success / abort / cond_capture / released 等。"""
    d_hat, m = sample(n, sigma0, sigma_e, bias, sigma_track, rng)
    sigma_m = math.sqrt(sigma_e ** 2 + sigma_track ** 2)
    T = max(0.0, r_eff - margin - k * sigma_m)          # 释放阈值（估计范数）
    n_hat = np.linalg.norm(d_hat, axis=1)
    n_m = np.linalg.norm(m, axis=1)
    released = n_hat <= T
    captured = n_m <= r_eff
    success = released & captured
    n_rel = int(released.sum())
    return dict(
        r_eff=r_eff, k=k, T=T, sigma_m=sigma_m,
        success_rate=float(success.mean()),
        abort_rate=float(1.0 - released.mean()),
        release_rate=float(released.mean()),
        cond_capture=(float(success.sum()) / n_rel if n_rel else float('nan')),
    )


def fmt(x, nd=4):
    return 'nan' if isinstance(x, float) and math.isnan(x) else f'{x:.{nd}f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=300000)
    ap.add_argument('--eps', type=float, default=0.05)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    N = args.n

    print('=' * 78)
    print('C1 概率证书：k(ε) = sqrt(2 ln(1/ε))')
    print('=' * 78)
    print(f"{'ε':>7} {'k(ε)':>7}   |   {'k=1 → ε_eff':>14}")
    for e in (0.20, 0.10, 0.05, 0.01):
        print(f"{e:>7.3f} {k_cert(e):>7.3f}   |   {fmt(eps_of_k(1.0), 3):>14}")
    print(f"  → 项目现用 release_sigma_k=1.0 实际只对应 ε≈{eps_of_k(1.0):.2f}（几乎无保证）")

    # 典型参数（可按 bench_coord 校准）
    sigma0, sigma_track = 0.08, 0.02
    for r_eff, label in ((0.14, '标准漏斗 eff_r=0.14'), (0.25, '大漏斗 eff_r=0.25')):
        margin = 0.05
        print('\n' + '=' * 78)
        print(f'[{label}]  margin={margin}  未含延时偏置 (A 悬停)')
        print('=' * 78)
        print(f"{'策略':<26} {'σ_e':>5} {'T':>7} {'释放率':>7} {'中止率':>7} "
              f"{'成功率':>7} {'条件捕获':>8}")
        for name, sig_e in (('direct  (相对定位 σ_r)', 0.06),
                            ('authority (上报 σ_comm)', 0.02)):
            r1 = sim_policy(N, r_eff, margin, sigma0, sig_e, 0.0, sigma_track,
                            k_cert(args.eps), rng)
            r2 = sim_policy(N, r_eff, margin, sigma0, sig_e, 0.0, sigma_track,
                            1.0, rng)
            print(f"{name + '  [cert]':<26} {sig_e:>5.2f} {r1['T']:>7.3f} "
                  f"{r1['release_rate']:>7.3f} {r1['abort_rate']:>7.3f} "
                  f"{r1['success_rate']:>7.3f} {r1['cond_capture']:>8.3f}")
            print(f"{name + '  [heur k=1]':<26} {sig_e:>5.2f} {r2['T']:>7.3f} "
                  f"{r2['release_rate']:>7.3f} {r2['abort_rate']:>7.3f} "
                  f"{r2['success_rate']:>7.3f} {r2['cond_capture']:>8.3f}")

    # 延迟偏置扫描（A 以 v_A 运动，delay 造成估计偏置 v_A·d）
    print('\n' + '=' * 78)
    print('延时偏置扫描（authority, eff_r=0.25, ε=0.05）bias = v_A·d')
    print('=' * 78)
    print(f"{'v_A':>5} {'delay':>6} {'bias':>6} | {'cert 成功率':>10} {'cert 条件捕获':>12} "
          f"| {'heur 成功率':>10} {'heur 条件捕获':>12}")
    for vA in (0.0, 0.5, 1.0):
        for d in (0.0, 0.1, 0.2):
            bias = vA * d
            rc = sim_policy(N, 0.25, 0.05, sigma0, 0.03, bias, sigma_track,
                            k_cert(args.eps), rng)
            rh = sim_policy(N, 0.25, 0.05, sigma0, 0.03, bias, sigma_track,
                            1.0, rng)
            print(f"{vA:>5.2f} {d:>6.2f} {bias:>6.2f} | {rc['success_rate']:>10.3f} "
                  f"{rc['cond_capture']:>12.3f} | {rh['success_rate']:>10.3f} "
                  f"{rh['cond_capture']:>12.3f}")

    # 证书覆盖性：cert 的"条件捕获"应 ≥ 1−ε
    print('\n' + '=' * 78)
    print('证书覆盖性检查（authority, eff_r=0.25）：条件捕获应 ≥ 1−ε')
    print('=' * 78)
    print(f"{'ε':>7} {'k(ε)':>7} {'条件捕获(cert)':>14} {'≥1−ε?':>7}")
    for e in (0.20, 0.10, 0.05, 0.01):
        r = sim_policy(N, 0.25, 0.05, sigma0, 0.03, 0.0, sigma_track, k_cert(e), rng)
        print(f"{e:>7.3f} {k_cert(e):>7.3f} {r['cond_capture']:>14.4f} "
              f"{'✅' if r['cond_capture'] >= 1 - e - 1e-3 else '❌':>7}")


if __name__ == '__main__':
    main()

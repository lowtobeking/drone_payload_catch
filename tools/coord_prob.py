#!/usr/bin/env python3
"""coord_prob.py —— 释放决策的【概率模型 + 捕获概率证书】（研究 W2 / C1）。

二维水平脱靶  m ~ N(μ, σ_m² I₂)   （μ 为估计偏差，σ_m² = σ_e² + σ_track²）
捕获事件      E = { ‖m‖ ≤ r_eff }
释放策略      当 ‖δ̂‖ ≤ T 时释放，否则继续等（/中止）

三种阈值 T：
  · exact    —— 用**非中心卡方**（Rice 分布 / Marcum-Q）精确求 T：
                  P(‖m‖≤R | μ=T) = 1−ε ；无偏(μ=0)时退化为 Rayleigh 精确式
  · chernoff —— 保守界  T = R − √(2 ln(1/ε))·σ_m
  · heuristic—— 项目旧行为  T = R − 1.0·σ_m（k=1）

标定：SITL 的 rel_pos_sigma 经 `_relnav_a` 的 EMA(α=0.30) 滤波后
      等效 σ_e = sqrt(α/(2−α))·rel_pos_sigma；延迟对运动的偏置 b = v_A·d。

用法：
  python3 tools/coord_prob.py [--n N] [--eps E] [--seed S] [--alpha A]
"""
from __future__ import annotations

import argparse
import math
import numpy as np

try:
    from scipy.stats import ncx2
    from scipy.optimize import brentq
    _HAVE_SCIPY = True
except Exception:  # noqa: BLE001
    _HAVE_SCIPY = False


# ------------------------------------------------------------------ 证书
def k_chernoff(eps: float) -> float:
    """Chernoff 界系数：P(‖m‖>R) ≤ exp(−(R−‖μ‖)²/(2σ²)) ⇒ k=√(2 ln(1/ε))。"""
    return math.sqrt(2.0 * math.log(1.0 / eps))


def eps_of_k(k: float) -> float:
    return math.exp(-0.5 * k * k)


def capture_prob(R: float, mu: float, sigma: float) -> float:
    """P(‖m‖ ≤ R)，m~N(μ, σ²I₂)。用非中心卡方：X=‖m/σ‖² ~ ncx2(df=2, nc=(μ/σ)²)。"""
    if sigma <= 0.0:
        return 1.0 if mu <= R else 0.0
    if not _HAVE_SCIPY:
        # 退化近似：无偏 Rayleigh
        return 1.0 - math.exp(-(max(R - mu, 0.0) ** 2) / (2 * sigma * sigma))
    return float(ncx2.cdf((R / sigma) ** 2, df=2, nc=(mu / sigma) ** 2))


def T_exact(eps: float, R: float, sigma_m: float) -> float:
    """精确阈值 T：使 μ=T 时 P(‖m‖≤R)=1−ε；不可达则返回 0（无法认证）。"""
    if R <= 0.0 or sigma_m <= 0.0:
        return 0.0
    if not _HAVE_SCIPY:
        return max(0.0, R - k_chernoff(eps) * sigma_m)
    f = lambda T: capture_prob(R, T, sigma_m) - (1.0 - eps)   # noqa: E731, 关于 T 递减
    if f(0.0) <= 0.0:
        return 0.0
    return float(brentq(f, 0.0, R))


def threshold(method: str, eps: float, r_eff: float, margin: float,
              sigma_m: float) -> float:
    R = r_eff - margin
    if method == 'exact':
        return T_exact(eps, R, sigma_m)
    if method == 'chernoff':
        return max(0.0, R - k_chernoff(eps) * sigma_m)
    return max(0.0, R - 1.0 * sigma_m)           # heuristic(k=1)


# ------------------------------------------------------------------ 仿真
def sample(n, sigma0, sigma_e, bias, sigma_track, rng):
    d_true = rng.normal(0.0, sigma0, (n, 2))
    e = rng.normal(0.0, sigma_e, (n, 2))
    e[:, 0] += bias
    d_hat = d_true + e
    w = rng.normal(0.0, sigma_track, (n, 2))
    m = d_true + w
    return d_hat, m


def sim_policy(n, r_eff, margin, sigma0, sigma_e, bias, sigma_track,
               method, eps, rng):
    d_hat, m = sample(n, sigma0, sigma_e, bias, sigma_track, rng)
    sigma_m = math.sqrt(sigma_e ** 2 + sigma_track ** 2)
    T = threshold(method, eps, r_eff, margin, sigma_m)
    n_hat = np.linalg.norm(d_hat, axis=1)
    n_m = np.linalg.norm(m, axis=1)
    released = n_hat <= T
    captured = n_m <= r_eff
    success = released & captured
    n_rel = int(released.sum())
    return dict(method=method, r_eff=r_eff, sigma_m=sigma_m, T=T,
                release_rate=float(released.mean()),
                abort_rate=float(1.0 - released.mean()),
                success_rate=float(success.mean()),
                cond_capture=(float(success.sum()) / n_rel if n_rel else float('nan')))


def fmt(x, nd=3):
    return 'nan' if isinstance(x, float) and math.isnan(x) else f'{x:.{nd}f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=300000)
    ap.add_argument('--eps', type=float, default=0.05)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--alpha', type=float, default=0.30, help='SITL EMA 滤波 alpha')
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    N = args.n

    print('=' * 82)
    print(f'C1 概率证书（scipy={_HAVE_SCIPY}）  k_chernoff(ε)=√(2 ln 1/ε)')
    print('=' * 82)
    print(f"{'ε':>7} {'k_chernoff':>11} {'k=1→ε_eff':>10}")
    for e in (0.20, 0.10, 0.05, 0.01):
        print(f"{e:>7.3f} {k_chernoff(e):>11.3f} {eps_of_k(1.0):>10.3f}")
    print(f"  → release_sigma_k=1.0 仅对应 ε≈{eps_of_k(1.0):.2f}（无可陈述保证）")

    # ---- 精确 vs Chernoff vs 启发式（阈值 T 与条件捕获）----
    sigma0, sigma_track = 0.08, 0.02
    print('\n' + '=' * 82)
    print('阈值 T 对比（σ_e=0.03, σ_track=0.02, ε=0.05, margin=0.05）')
    print('=' * 82)
    sm = math.sqrt(0.03 ** 2 + 0.02 ** 2)
    print(f"{'漏斗 eff_r':>9} | {'T_exact':>8} {'T_chernoff':>11} {'T_heur(k=1)':>12}")
    for r_eff in (0.10, 0.14, 0.20, 0.25, 0.30):
        print(f"{r_eff:>9.2f} | {threshold('exact', args.eps, r_eff, 0.05, sm):>8.4f} "
              f"{threshold('chernoff', args.eps, r_eff, 0.05, sm):>11.4f} "
              f"{threshold('heuristic', args.eps, r_eff, 0.05, sm):>12.4f}")

    # ---- 策略成功率（大漏斗）----
    for r_eff in (0.25,):
        print('\n' + '=' * 82)
        print(f'策略对比（eff_r={r_eff}, ε={args.eps}, margin=0.05）')
        print('=' * 82)
        print(f"{'方法':<12} {'策略':<12} {'σ_e':>5} {'T':>7} {'释放率':>7} "
              f"{'中止率':>7} {'成功率':>7} {'条件捕获':>8}")
        for mname in ('exact', 'chernoff', 'heuristic'):
            for pname, sig_e in (('direct', 0.06), ('authority', 0.02)):
                r = sim_policy(N, r_eff, 0.05, sigma0, sig_e, 0.0, sigma_track,
                               mname, args.eps, rng)
                print(f"{mname:<12} {pname:<12} {sig_e:>5.2f} {r['T']:>7.4f} "
                      f"{r['release_rate']:>7.3f} {r['abort_rate']:>7.3f} "
                      f"{r['success_rate']:>7.3f} {r['cond_capture']:>8.3f}")

    # ---- 标定：SITL rel_pos_sigma → σ_e（EMA）→ 证书可行性 ----
    print('\n' + '=' * 82)
    print(f'标定：rel_pos_sigma 经 EMA(α={args.alpha}) → σ_e_eff；margin=0.05, ε={args.eps}')
    print('=' * 82)
    fac = math.sqrt(args.alpha / (2.0 - args.alpha))
    print(f"  EMA 噪声折算因子 = sqrt(α/(2−α)) = {fac:.3f}")
    print(f"{'rel_σ':>6} {'σ_e_eff':>8} {'σ_m':>7} | "
          f"{'T@0.25':>8} {'可行?':>6} | {'T@0.14':>8} {'可行?':>6}")
    for rel in (0.0, 0.03, 0.05, 0.10, 0.15):
        s_e = fac * rel
        s_m = math.sqrt(s_e ** 2 + sigma_track ** 2)
        T25 = threshold('exact', args.eps, 0.25, 0.05, s_m)
        T14 = threshold('exact', args.eps, 0.14, 0.05, s_m)
        print(f"{rel:>6.2f} {s_e:>8.4f} {s_m:>7.4f} | {T25:>8.4f} "
              f"{'✅' if T25 > 0 else '❌':>6} | {T14:>8.4f} {'✅' if T14 > 0 else '❌':>6}")

    # ---- 覆盖性（exact）----
    print('\n' + '=' * 82)
    print('覆盖性检查（exact, authority σ_e=0.03, eff_r=0.25）')
    print('=' * 82)
    print(f"{'ε':>7} {'T':>8} {'条件捕获':>10} {'≥1−ε':>7}")
    for e in (0.20, 0.10, 0.05, 0.01):
        r = sim_policy(N, 0.25, 0.05, sigma0, 0.03, 0.0, sigma_track,
                       'exact', e, rng)
        print(f"{e:>7.3f} {r['T']:>8.4f} {r['cond_capture']:>10.4f} "
              f"{'✅' if r['cond_capture'] >= 1 - e - 1e-3 else '❌':>7}")


if __name__ == '__main__':
    main()

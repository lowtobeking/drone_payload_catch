#!/usr/bin/env python3
"""stats.py —— 统计工具（成功率置信区间 + 配对比较），纯 Python、可自测。

- Wilson 区间：二项比例的小样本置信区间（比正态近似更稳，n 小/接近 1 时也可用）。
- McNemar 检验：两种方法在**同一批样本**上的配对差异显著性（适合消融对比）。

用法: python3 -m payload_catch.stats     # 自测
"""
from __future__ import annotations

import math


def wilson_ci(k: int, n: int, z: float = 1.96):
    """返回 (p, lo, hi)：成功率及其 Wilson 95% 置信区间。"""
    if n <= 0:
        return (float('nan'), float('nan'), float('nan'))
    p = k / n
    denom = 1.0 + z * z / n
    center = (p + z * z / (2.0 * n)) / denom
    half = (z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n))) / denom
    return p, max(0.0, center - half), min(1.0, center + half)


def fmt_ci(k: int, n: int, z: float = 1.96) -> str:
    p, lo, hi = wilson_ci(k, n, z)
    if math.isnan(p):
        return '—'
    return f'{k}/{n} ({100*p:.1f}% [{100*lo:.1f}, {100*hi:.1f}])'


def mcnemar(b: int, c: int, exact_thresh: int = 25) -> dict:
    """配对检验：b=甲对乙错，c=甲错乙对。返回统计量与近似 p 值。"""
    n = b + c
    if n == 0:
        return dict(n=0, chi2=0.0, p=1.0, note='无差异样本')
    if n < exact_thresh:
        # 精确二项（双侧）：p = 2 * P(X ≤ min(b,c)), X~Bin(n,0.5)
        m = min(b, c)
        cum = sum(math.comb(n, i) for i in range(m + 1)) / (2.0 ** n)
        return dict(n=n, chi2=float('nan'), p=min(1.0, 2.0 * cum), note='exact')
    chi2 = (abs(b - c) - 1.0) ** 2 / n          # 连续性校正
    p = math.erfc(math.sqrt(chi2 / 2.0))         # 卡方 df=1 的近似
    return dict(n=n, chi2=chi2, p=p, note='chi2-cc')


def _selftest():
    ok = True
    # 1) 全成功：下界 > 0.9（小样本）
    p, lo, hi = wilson_ci(30, 30)
    print(f'[1] 30/30 → p={p:.2f} CI=[{lo:.3f},{hi:.3f}]')
    ok &= (lo > 0.85 and hi <= 1.0)

    # 2) 已知值：5/10 → 接近 [0.24, 0.76]
    p, lo, hi = wilson_ci(5, 10)
    print(f'[2] 5/10  → CI=[{lo:.2f},{hi:.2f}] (期望约 [0.24,0.76])')
    ok &= (abs(lo - 0.24) < 0.05 and abs(hi - 0.76) < 0.05)

    # 3) 大样本收紧
    p, lo, hi = wilson_ci(950, 1000)
    print(f'[3] 950/1000 → CI=[{lo:.3f},{hi:.3f}] (窄)')
    ok &= (hi - lo < 0.05)

    # 4) McNemar：明显差异显著
    r = mcnemar(20, 2)
    print(f'[4] McNemar b=20 c=2 → p={r["p"]:.4f} ({r["note"]})')
    ok &= (r['p'] < 0.05)

    print('stats 自测', '通过 ✅' if ok else '失败 ❌')
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)

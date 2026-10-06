#!/usr/bin/env python3
"""coord_cert.py —— 捕获概率证书（C1）核心数学（无 ROS 依赖，可离线/节点共用）。

二维水平脱靶  m ~ N(μ, σ_m² I₂)，‖m‖ 服从 Rice 分布（等价非中心卡方 df=2）。
  P(‖m‖ ≤ R) = F_ncx2((R/σ)²; 2, (μ/σ)²)

释放证书：取阈值 T(ε) 使 μ=T 时 P(‖m‖≤R)=1−ε；则 ‖δ̂‖ ≤ T ⇒ P(capture) ≥ 1−ε。
  无偏时退化 Rayleigh 精确：T = σ√(2 ln(1/ε))。
  保守闭式（Chernoff）：T = R − √(2 ln(1/ε))·σ。
"""
from __future__ import annotations

import math

try:
    from scipy.stats import ncx2
    from scipy.optimize import brentq
    HAVE_SCIPY = True
except Exception:  # noqa: BLE001
    HAVE_SCIPY = False


def k_chernoff(eps: float) -> float:
    """Chernoff 界系数 k(ε)=√(2 ln(1/ε))。"""
    return math.sqrt(2.0 * math.log(1.0 / eps))


def eps_of_k(k: float) -> float:
    """给定系数 k 对应的保证水平 ε=exp(−k²/2)（揭示旧启发式的实际保证）。"""
    return math.exp(-0.5 * k * k)


def capture_prob(R: float, mu: float, sigma: float) -> float:
    """P(‖m‖ ≤ R)，m~N(μ, σ²I₂)。"""
    if sigma <= 0.0:
        return 1.0 if mu <= R else 0.0
    if not HAVE_SCIPY:
        return 1.0 - math.exp(-(max(R - mu, 0.0) ** 2) / (2.0 * sigma * sigma))
    return float(ncx2.cdf((R / sigma) ** 2, df=2, nc=(mu / sigma) ** 2))


def T_exact(eps: float, R: float, sigma_m: float) -> float:
    """精确阈值 T（μ=T 时 P(‖m‖≤R)=1−ε）；不可达返回 0（无法认证）。"""
    if R <= 0.0 or sigma_m <= 0.0:
        return 0.0
    if not HAVE_SCIPY:
        return max(0.0, R - k_chernoff(eps) * sigma_m)
    f = lambda T: capture_prob(R, T, sigma_m) - (1.0 - eps)   # noqa: E731（关于 T 递减）
    if f(0.0) <= 0.0:
        return 0.0
    return float(brentq(f, 0.0, R))


def cert_threshold(mode: str, eps: float, r_eff: float, margin: float,
                   sigma_m: float) -> float:
    """按模式返回释放阈值 T（释放判据：估计范数 ‖δ̂‖ ≤ T）。"""
    R = r_eff - margin
    if mode == 'exact':
        return T_exact(eps, R, sigma_m)
    if mode == 'chernoff':
        return max(0.0, R - k_chernoff(eps) * sigma_m)
    # 'heuristic'：旧行为 k=1
    return max(0.0, R - 1.0 * sigma_m)

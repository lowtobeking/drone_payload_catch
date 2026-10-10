#!/usr/bin/env python3
"""C1 捕获概率证书（coord_cert.py）离线自检——纯数学，无需 ROS/PX4/Gazebo。

    python3 tools/test_coord_cert.py

证书是论文核心贡献 C1 的地基，且**在仿真里看不出来对不对**（阈值偏大/偏小
都只是"成功率略有变化"），所以在节点里用它之前先把数学本身钉死。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch import coord_cert as cc   # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


print('=== k(ε) 与 ε(k) 互逆 ===')
for eps in (1e-1, 1e-2, 1e-3, 1e-6):
    k = cc.k_chernoff(eps)
    check(f'eps_of_k(k_chernoff({eps:g})) 回原值',
          abs(cc.eps_of_k(k) - eps) < 1e-12 * max(eps, 1e-12),
          f'k={k:.4f}')


print('\n=== capture_prob 边界 ===')
check('σ=0 且 μ≤R → 1.0', cc.capture_prob(0.2, 0.1, 0.0) == 1.0)
check('σ=0 且 μ>R → 0.0', cc.capture_prob(0.2, 0.3, 0.0) == 0.0)
check('R=0 且 μ=0 → 0.0', cc.capture_prob(0.0, 0.0, 0.1) == 0.0)
check('R≫σ,μ → ≈1.0', cc.capture_prob(10.0, 0.0, 0.1) > 1 - 1e-9)
check('单调：R 增大不减', cc.capture_prob(0.2, 0.1, 0.1) <= cc.capture_prob(0.3, 0.1, 0.1))


print('\n=== capture_prob vs 蒙特卡洛（Rice 分布正确性）===')
rng = np.random.default_rng(0)
N = 400000
for (R, mu, sig) in [(0.25, 0.05, 0.08), (0.20, 0.10, 0.10), (0.35, 0.0, 0.15)]:
    m = rng.normal(loc=[mu, 0.0], scale=sig, size=(N, 2))
    # 各向同性：把均值放到 x 轴（‖m‖ 分布只依赖 μ 的模）
    frac = float(np.mean(np.linalg.norm(m, axis=1) <= R))
    p = cc.capture_prob(R, mu, sig)
    check(f'R={R} μ={mu} σ={sig}: 解析 {p:.4f} ≈ MC {frac:.4f}',
          abs(p - frac) < 0.004, f'差 {abs(p - frac):.4f}')


print('\n=== T_exact：无偏时的可认证边界 ===')
for eps in (0.1, 0.05, 0.01, 0.001):
    sig = 0.1
    R_min = cc.k_chernoff(eps) * sig     # μ=0 时恰好 P=1−ε 所需的最小 R
    check(f'ε={eps}: P(R_min=kσ; μ=0)=1−ε',
          abs(cc.capture_prob(R_min, 0.0, sig) - (1 - eps)) < 1e-9)
    check(f'ε={eps}: R=R_min 恰在边界 → T=0', cc.T_exact(eps, R_min, sig) == 0.0)
    check(f'ε={eps}: R<R_min 不可认证 → T=0', cc.T_exact(eps, 0.98 * R_min, sig) == 0.0)
    check(f'ε={eps}: R>R_min → T>0', cc.T_exact(eps, 1.02 * R_min, sig) > 0.0)


print('\n=== T_exact 满足定义 P(R; T, σ)=1−ε ===')
for eps in (0.2, 0.05, 0.01):
    R, sig = 0.30, 0.06
    T = cc.T_exact(eps, R, sig)
    got = cc.capture_prob(R, T, sig)
    check(f'ε={eps}: P(R;T={T:.4f},σ)≈{1-eps:.4f}', abs(got - (1 - eps)) < 1e-6,
          f'得到 {got:.6f}')


print('\n=== T_exact 单调性与不可达 ===')
R, sig = 0.30, 0.06
check('ε 越小 → T 越小（保证越强）',
      cc.T_exact(0.001, R, sig) < cc.T_exact(0.1, R, sig))
check('R 越大 → T 越大', cc.T_exact(0.05, 0.4, sig) > cc.T_exact(0.05, 0.2, sig))
check('σ 越大 → T 越小', cc.T_exact(0.05, R, 0.15) < cc.T_exact(0.05, R, 0.03))
check('R 太小不可认证 → 0', cc.T_exact(0.01, 0.05, 0.5) == 0.0)


print('\n=== cert_threshold：模式关系 ===')
eps, r_eff, margin, sig = 0.05, 0.25, 0.01, 0.05
R = r_eff - margin
T_ex = cc.cert_threshold('exact', eps, r_eff, margin, sig)
T_ch = cc.cert_threshold('chernoff', eps, r_eff, margin, sig)
T_he = cc.cert_threshold('heuristic', eps, r_eff, margin, sig)
check('exact == T_exact', abs(T_ex - cc.T_exact(eps, R, sig)) < 1e-12)
check('exact ≥ chernoff（Chernoff 保守）', T_ex >= T_ch - 1e-12,
      f'exact={T_ex:.4f} chernoff={T_ch:.4f}')
check('heuristic(k=1) ≥ chernoff（启发式保证更弱）', T_he >= T_ch - 1e-12,
      f'heuristic={T_he:.4f} chernoff={T_ch:.4f}')
check('chernoff 公式 = R − kσ', abs(T_ch - max(0.0, R - cc.k_chernoff(eps) * sig)) < 1e-12)
check('exact 阈值给出 ≥1−ε 的保证', cc.capture_prob(R, T_ex, sig) >= 1 - eps - 1e-9,
      f'P={cc.capture_prob(R, T_ex, sig):.5f}')

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ test_coord_cert 全部通过')

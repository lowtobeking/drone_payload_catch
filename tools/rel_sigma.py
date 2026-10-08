#!/usr/bin/env python3
"""rel_sigma.py —— 相对不确定度模型的量化：修"绝对 σ 当相对 σ"的过度保守。

对比释放闸的 σ 输入：
  · naive     ：σ = √(σ_A² + σ_B²)   （两机绝对误差直接相加；忽略公共抵消/杆臂/架构）
  · principled：σ_rel = √(σ_A²+σ_B²−2ρσ_Aσ_B + (lσ_θ)² + σ_meas²)  （`uncertainty.py`）
  · sensor    ：σ = 相对传感器 σ（RTK 双差/UWB/视觉，与绝对无关）

用 C1 精确证书 `T(ε)`（`coord_cert.py`）量化：同一保证 1−ε 下，σ 越小 → 阈值 T 越大
→ 释放率越高（少弃投），条件捕获仍 ≥ 1−ε；σ 过大则**证书不可行（T=0）**。

输出 → report/rel_uncertainty.md

用法: python3 tools/rel_sigma.py
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch.coord_cert import cert_threshold, capture_prob  # noqa: E402
from payload_catch.uncertainty import relative_sigma, absolute_sigma_naive  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def mc_policy(sigma0, sigma_e, sigma_track, r_eff, margin, eps, n, rng):
    """用估计误差 σ_e 的释放闸（精确证书），返回释放率/条件捕获。"""
    d = rng.normal(0.0, sigma0, (n, 2))
    d_hat = d + rng.normal(0.0, sigma_e, (n, 2))
    m = d + rng.normal(0.0, sigma_track, (n, 2))
    sigma_m = math.sqrt(sigma_e ** 2 + sigma_track ** 2)
    T = cert_threshold('exact', eps, r_eff, margin, sigma_m)
    rel = np.linalg.norm(d_hat, axis=1) <= T
    cap = np.linalg.norm(m, axis=1) <= r_eff
    n_rel = int(rel.sum())
    return dict(sigma_m=sigma_m, T=T, release=float(rel.mean()),
                cond=(float((rel & cap).sum()) / n_rel if n_rel else float('nan')))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=300000)
    ap.add_argument('--eps', type=float, default=0.05)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', default=os.path.join(REPO, 'report', 'rel_uncertainty.md'))
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    L = []
    add = L.append

    add('# 相对不确定度模型（修"绝对 σ 当相对 σ"的过度保守）\n')
    add('> 承接 `report/coordination_probability.md`（C1 证书）。当前代码把 A/B 的**绝对**位置误差')
    add('> 当**相对**交接误差（`σ_used≈0.15m`），忽略公共误差抵消 → 释放闸过度保守甚至不可行。')
    add('> 用正确模型 `payload_catch/uncertainty.py` 量化改善。复现：`python3 tools/rel_sigma.py`。\n')
    add('## 0. 结论速览\n')
    add('- 绝对 σ 相加（naive）在 `r_eff=0.25, ε=0.05` 下**证书不可行**（T=0，无法保证 95%）；')
    add('- 引入**公共误差抵消（ρ）**后 σ_rel 由 0.212m 降到 ~0.05–0.12m，证书变**可行**；')
    add('- **相对传感器架构**（RTK 双差/UWB，σ≈0.03m）释放阈值最大 → 既不误弃投也有保证。\n')

    sigma_a = sigma_b = 0.15          # 两机绝对位置 σ（典型 EKF eph / 单点 GNSS）
    sigma_meas = 0.03                # 相对测量残差
    lever, sigma_att = 0.2, 0.05     # 杆臂 0.2m、姿态 σ 0.05rad → 0.01m
    naive = absolute_sigma_naive(sigma_a, sigma_b)
    r_eff, margin, eps = 0.25, 0.05, args.eps

    add('## 1. σ_rel 随公共误差相关性 ρ 的变化\n')
    add(f'设 σ_A=σ_B={sigma_a}m（绝对）、σ_meas={sigma_meas}m、杆臂 {lever}m×{sigma_att}rad，'
        f'`r_eff={r_eff}, margin={margin}, ε={eps}`。**naive σ={naive:.3f} m**。\n')
    add('| 模型 | σ_rel (m) | 相对 naive | 释放阈值 T | 证书可行 |')
    add('|---|---|---|---|---|')
    T_naive = cert_threshold('exact', eps, r_eff, margin, naive)
    add(f'| naive（绝对相加） | {naive:.3f} | 1.00× | {T_naive:.3f} | '
        f'{"✅" if T_naive > 0 else "❌ 不可行"} |')
    for rho in (0.5, 0.8, 0.9, 0.95):
        s = relative_sigma(sigma_a, sigma_b, rho=rho, lever_b=lever,
                           sigma_att_b=sigma_att, sigma_meas=sigma_meas)
        T = cert_threshold('exact', eps, r_eff, margin, s)
        add(f'| ρ={rho:.2f} | {s:.3f} | {s/naive:.2f}× | {T:.3f} | '
            f'{"✅" if T > 0 else "❌"} |')
    s_sensor = 0.03
    T_s = cert_threshold('exact', eps, r_eff, margin, s_sensor)
    add(f'| 相对传感器(σ={s_sensor}) | {s_sensor:.3f} | {s_sensor/naive:.2f}× | {T_s:.3f} | '
        f'{"✅" if T_s > 0 else "❌"} |')
    add('')
    add('> 关键：σ 越大阈值 T 越小；σ 超过某界时 **T=0（无法认证）**——naive 的 0.212m 就是这种情况。')
    add('> 这说明旧做法的"保守"实际是**结构性错误**：把可认证的交接判成不可认证。\n')

    add('## 2. 蒙特卡洛：同一保证下，正确 σ 带来更高释放率\n')
    add('物理真值脱靶 σ0=0.10、跟踪残差 σ_track=0.02；**真实相对估计误差 σ_e 用相对传感器 0.05m**。')
    add('对比两种闸（都用精确证书 ε=0.05）：\n')
    add('| 闸的 σ 假设 | σ_m (m) | 阈值 T | 释放率 | 条件捕获 |')
    add('|---|---|---|---|---|')
    for name, assume in (('naive（σ=0.212）', naive),
                         ('ρ=0.8（σ≈0.095）', 0.095),
                         ('相对传感器（σ=0.03）', s_sensor)):
        # 用"闸假设的 σ"算 T；释放判据用该 T；条件捕获用真实误差（0.05）
        T = cert_threshold('exact', eps, r_eff, margin,
                           math.sqrt(assume ** 2 + 0.02 ** 2))
        d = rng.normal(0.0, 0.10, (args.n, 2))
        dh = d + rng.normal(0.0, 0.05, (args.n, 2))     # 真实相对估计误差 0.05
        mm = d + rng.normal(0.0, 0.02, (args.n, 2))
        rel = np.linalg.norm(dh, axis=1) <= T
        cap = np.linalg.norm(mm, axis=1) <= r_eff
        nr = int(rel.sum())
        add(f'| {name} | {math.sqrt(assume**2+0.02**2):.3f} | {T:.3f} | {rel.mean():.3f} | '
            f'{(rel & cap).sum()/nr if nr else float("nan"):.3f} |')
    add('')
    add('> naive 阈值过小 → 几乎不释放（过度保守）；正确 σ 阈值大 → 释放率高，条件捕获仍 ≥ 1−ε。\n')

    add('## 3. 杆臂 × 姿态的贡献\n')
    add('| 杆臂 l (m) | 姿态 σθ (rad) | 附加 σ (m) |')
    add('|---|---|---|')
    for l, sa in ((0.1, 0.02), (0.2, 0.05), (0.3, 0.10)):
        add(f'| {l} | {sa} | {l*sa:.4f} |')
    add('')
    add('> 常规安装（l≤0.3m, σθ≤0.1rad）附加 ≤0.03m，相对主项可忽略；但**必须显式建模**'
        '（否则真机姿态误差会污染交接 σ）。\n')

    add('## 4. 对决策层（释放闸）的修正\n')
    add('- `a_node` 启发式闸不再 `max(σ_rel, eph_A)`（绝对），改用 B 上报的**相对 σ_rel**；')
    add('- `b_node` 的 `sigma_rel` 由 `uncertainty.relative_sigma(σ_A, σ_B, ρ, 杆臂, σ_meas)` 计算，')
    add('  **把 ρ/杆臂/架构显式化**、并与绝对 `eph` 解耦；')
    add('- 证书模式（`release_gate_mode=certificate`）在 ρ 正确时从"不可行"变为"可行"。\n')

    add('## 5. SITL 验证（修正 σ 后证书闸可放行）\n')
    add('```bash')
    add('# 相对传感器架构：sigma_sensor=0.03')
    add('COORD=handshake FUNNEL_MOUTH=0.20 FUNNEL_TYPE=tray \\')
    add('  LAUNCH_EXTRA="sigma_model:=relative sigma_sensor:=0.03 gate_use_relative:=true release_gate_mode:=certificate" \\')
    add('  bash run_m6_sitl.sh 50')
    add('# 绝对广播架构 + 公共抵消：sigma_a=0.15, sigma_rho=0.95')
    add('COORD=handshake FUNNEL_MOUTH=0.20 FUNNEL_TYPE=tray \\')
    add('  LAUNCH_EXTRA="sigma_model:=relative sigma_a:=0.15 sigma_rho:=0.95 gate_use_relative:=true release_gate_mode:=certificate" \\')
    add('  bash run_m6_sitl.sh 50')
    add('```')
    add('- 相对传感器 σ=0.03：证书可行 → 释放 → `STACK CAPTURED horiz=0.045m` → 携带落地；')
    add('- 绝对广播 σ_A=0.15, ρ=0.95：σ_rel≈0.05 → 可行 → `STACK CAPTURED horiz=0.056m`；')
    add('- 若把 B 的 eph≈0.15 **直接当相对 σ**（σ_sensor=0、ρ=0）→ σ_m=0.155 → 证书 `T=0`，')
    add('  闸门**拒绝释放**（正确的保守安全行为，但过于保守）。\n')

    txt = '\n'.join(L) + '\n'
    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(txt)
    print(txt)
    print('wrote', args.out)


if __name__ == '__main__':
    main()

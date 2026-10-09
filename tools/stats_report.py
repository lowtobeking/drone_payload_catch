#!/usr/bin/env python3
"""stats_report.py —— 统计 + 基线的离线研究报告（论文用）。

M6 垂直投放仿真极快（~0.8ms/次）→ 可做**超大 N** 统计；会合类（M1–M4）~1s/次 → 小 N。
统一给 **Wilson 95% CI**；消融用**配对 McNemar**（同种子）；含参数不确定性。

输出 → report/statistics.md
用法: python3 tools/stats_report.py [--n 2000]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch.sim_core import simulate, SimNoise            # noqa: E402
from payload_catch.stack_drop import simulate_stack, StackNoise   # noqa: E402
from payload_catch.stats import fmt_ci, mcnemar                   # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
YAML = os.path.join(REPO, 'config', 'catch_scenarios.yaml')
REL_S, REL_LAT = 0.05, 0.05


def _cfg():
    return yaml.safe_load(open(YAML, encoding='utf-8'))


def _with(cfg, name, **over):
    s0 = cfg['scenarios'][name]
    s = {k: (dict(v) if isinstance(v, dict) else v) for k, v in s0.items()}
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(s.get(k), dict):
            s[k] = {**s[k], **v}
        else:
            s[k] = v
    return s


def mc_stack(cfg, name, n, release_sigma=REL_S, rel_sigma=REL_S, over=None, lead=0.0):
    d = cfg['defaults']; s = _with(cfg, name, **(over or {}))
    lay = cfg['layouts'][s['layout']]
    out = []
    for i in range(n):
        nz = StackNoise(release_pos_sigma=release_sigma, rel_pos_sigma=rel_sigma,
                        rel_latency=REL_LAT, seed=i)
        r, _ = simulate_stack(d, lay, s, noise=nz, lead=lead)
        out.append(bool(r.success))
    return out


def mc_rv(cfg, name, n, closed, release_sigma):
    d = cfg['defaults']; s = cfg['scenarios'][name]; lay = cfg['layouts'][s['layout']]
    out = []
    for i in range(n):
        nz = SimNoise(release_pos_sigma=release_sigma, payload_pos_sigma=REL_S,
                      meas_latency=REL_LAT, seed=i)
        try:
            r, _ = simulate(d, lay, s, closed_loop=closed, noise=nz)
            out.append(bool(r.success))
        except Exception:
            out.append(False)
    return out


def paired(a, b):
    return (sum(1 for x, y in zip(a, b) if x and not y),
            sum(1 for x, y in zip(a, b) if (not x) and y))


def line(s): s.append('')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=2000, help='M6 统计的 N（会合类用 N//50）')
    ap.add_argument('--rv-n', type=int, default=40, help='会合类的 N（慢）')
    args = ap.parse_args()
    N, NRV = args.n, args.rv_n
    cfg = _cfg()
    L = []; add = L.append; bl = lambda: L.append('')
    add('# 统计 + 基线（离线 · Wilson CI · 配对检验）\n')
    add(f'> M6 每格 **N={N}**；会合类 **N={NRV}**（~1s/次，故小）。统一噪声：释放 σ=0.05m、')
    add('> 测量 σ=0.05m、延迟 0.05s。成功率给 Wilson 95% CI；消融用同种子配对 McNemar。')
    add('> 复现：`python3 tools/stats_report.py`。\n')

    add('## 1. M6 末端机构 / 感知（大 N）\n')
    add('| 配置 | 成功率 (Wilson 95% CI) |')
    add('|---|---|')
    cache = {}
    specs = [
        ('M6 标准漏斗 0.20', 'M6_stack_drop', {}),
        ('M6 大漏斗 0.30', 'M6_stack_bigfunnel', {}),
        ('M6 托盘 30cm', 'M6_stack_tray', {}),
        ('M6 大托盘 40cm', 'M6_stack_tray', {'capture': {'funnel': {'mouth_radius': 0.20, 'depth': 0.05, 'restitution': 0.15, 'mount_height': 0.10, 'object_radius': 0.03}}}),
        ('M6 托盘+主动保持', 'M6_stack_tray', {'capture': {'arm_reach': 0.10, 'arm_absorb': 10.0}}),
        ('M6 托盘+相机感知', 'M6_stack_tray_cam', {}),
    ]
    for tag, name, over in specs:
        o = mc_stack(cfg, name, N, over=over)
        cache[tag] = o
        add(f'| {tag} | {fmt_ci(sum(o), N)} |')
    bl()

    add('## 2. 感知：真值替身 vs 相机（配对）\n')
    truth = mc_stack(cfg, 'M6_stack_tray', N)
    cam = mc_stack(cfg, 'M6_stack_tray_cam', N)
    b, c = paired(truth, cam)
    r = mcnemar(b, c)
    add('| 感知 | 成功率 (Wilson 95% CI) |')
    add('|---|---|')
    add(f'| 真值替身 σ=0.05 | {fmt_ci(sum(truth), N)} |')
    add(f'| 相机(60°,drop0.1) | {fmt_ci(sum(cam), N)} |')
    add('')
    add(f'配对 McNemar：b(真对/相错)={b}, c(真错/相对)={c}, p={r["p"]:.3f}（{r["note"]}）')
    bl()

    add('## 3. 消融：预测式对正（托盘，释放误差 σ=0.10，配对）\n')
    l0 = mc_stack(cfg, 'M6_stack_tray', N, release_sigma=0.10, lead=0.0)
    l1 = mc_stack(cfg, 'M6_stack_tray', N, release_sigma=0.10, lead=1.0)
    b1, c1 = paired(l0, l1)
    r1 = mcnemar(b1, c1)
    add('| 方案 | 成功率 (Wilson 95% CI) |')
    add('|---|---|')
    add(f'| 追尾 lead=0 | {fmt_ci(sum(l0), N)} |')
    add(f'| 预测 lead=1 | {fmt_ci(sum(l1), N)} |')
    add('')
    add(f'配对 McNemar：b={b1}, c={c1}, p={r1["p"]:.4g}（{r1["note"]}）')
    bl()

    add('## 4. 释放误差敏感性（托盘，N={}）\n'.format(N))
    add('| 释放误差 σ (m) | 成功率 (Wilson 95% CI) |')
    add('|---|---|')
    for sig in (0.05, 0.10, 0.15, 0.20, 0.30):
        o = mc_stack(cfg, 'M6_stack_tray', N, release_sigma=sig)
        add(f'| {sig:.2f} | {fmt_ci(sum(o), N)} |')
    bl()

    add('## 5. 参数不确定性（托盘，N={}）\n'.format(N))
    add('| 参数分布 | 成功率 (Wilson 95% CI) |')
    add('|---|---|')
    rng = np.random.default_rng(0)
    d = cfg['defaults']
    for tag, samp in (
        ('质量 U(0.08,0.12) kg', lambda: {'payload': {'mass': float(rng.uniform(0.08, 0.12))}}),
        ('阻力 k ~ U(0,0.3)', lambda: {'payload': {'drag_mode': 'linear', 'drag_k': float(rng.uniform(0, 0.3))}}),
        ('侧风 U(-3,3) m/s', lambda: {'wind': [float(rng.uniform(-3, 3)), 0.0, 0.0]}),
        ('gap U(0.8,1.2) m', lambda: {'b_standby': [0.0, 0.0, -3.5 + float(rng.uniform(-0.2, 0.2))]}),
    ):
        ok = 0
        for i in range(N):
            s = _with(cfg, 'M6_stack_tray', **samp())
            lay = cfg['layouts'][s['layout']]
            nz = StackNoise(release_pos_sigma=REL_S, rel_pos_sigma=REL_S, rel_latency=REL_LAT, seed=i)
            r, _ = simulate_stack(d, lay, s, noise=nz)
            ok += int(r.success)
        add(f'| {tag} | {fmt_ci(ok, N)} |')
    bl()

    add(f'## 6. 会合类消融（M{NRV} 次）：闭环重规划（M3_wind_drag 风+阻力失配）\n')
    op = mc_rv(cfg, 'M3_wind_drag', NRV, closed=False, release_sigma=0.10)
    cl = mc_rv(cfg, 'M3_wind_drag', NRV, closed=True, release_sigma=0.10)
    bo, co = paired(op, cl)
    ro = mcnemar(bo, co)
    add('| 方案 | 成功率 (Wilson 95% CI) |')
    add('|---|---|')
    add(f'| 开环 | {fmt_ci(sum(op), NRV)} |')
    add(f'| 闭环 | {fmt_ci(sum(cl), NRV)} |')
    add('')
    add(f'配对 McNemar：b(开对/闭错)={bo}, c(开错/闭对)={co}, p={ro["p"]:.4g}（{ro["note"]}）')
    bl()

    add('## 7. 结论 / 用法\n')
    add('- 所有成功率附 **Wilson 95% CI**；差异是否显著看**配对 McNemar** p 值；')
    add('- M6 可超大 N（秒级）；会合类慢，仅小 N（`offline_run --all` 已给标称确定性结果）；')
    add('- **SITL 大 N**：`tools/mc_m6_sitl.sh`、`tools/mc_m6_tray_sitl.sh`（已附 Wilson CI），')
    add('  建议 SITL N≥20/档；离线做大规模统计、SITL 做真实性验证。')
    L.append('')

    out = os.path.join(REPO, 'report', 'statistics.md')
    txt = '\n'.join(L) + '\n'
    open(out, 'w', encoding='utf-8').write(txt)
    print(txt)
    print('wrote', out)


if __name__ == '__main__':
    main()

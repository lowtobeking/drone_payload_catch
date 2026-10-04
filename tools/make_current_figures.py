#!/usr/bin/env python3
"""make_current_figures.py — 生成"当前工作"关键结果图（离线，纯 Python）。

输出 (PNG -> report/figures/):
  fig_cur_summary.png  四联图：
    (a) 侧风鲁棒性：基线 / +速度前馈 / +预测对正 / +大漏斗 / +ZEM
    (b) ZEM 增益敏感性
    (c) 漏斗物理：v_rel vs gap、v_retain vs depth/e
    (d) 末端机构：平顶盘 vs 空心锥 vs 主动保持 的能力对比
用法: python3 tools/make_current_figures.py
"""
from __future__ import annotations

import os
import statistics as st
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
from payload_catch.stack_drop import simulate_stack, StackNoise, retain_speed, contact_rel_speed  # noqa

FIG = os.path.join(REPO, 'report', 'figures')
os.makedirs(FIG, exist_ok=True)
plt.rcParams.update({'figure.dpi': 130, 'font.size': 9, 'axes.grid': True,
                     'grid.alpha': 0.3, 'axes.unicode_minus': False})
with open(os.path.join(REPO, 'config', 'catch_scenarios.yaml'), encoding='utf-8') as f:
    CFG = yaml.safe_load(f)
DEF = CFG['defaults']
NOISE = dict(release_pos_sigma=0.05, rel_pos_sigma=0.05, rel_latency=0.05)


def mc(wind, lead, vel_ff, est, funnel, zem, n=20, comp=False):
    ok = 0
    for i in range(n):
        layout = {'a_init': [0, 0, -4.2], 'b_standby': [0, 0, -3.5]}
        scen = {'stack': {'a_dive': 3.0, 'a_brake': 6.0}, 'vert_mode': 'minimal',
                'capture': {'funnel': dict(funnel)},
                'payload': {'drag_mode': 'linear', 'drag_k': 1.0}, 'wind': [wind, 0, 0]}
        if comp:
            scen['a_wind_comp'] = True
        kw = dict(est_mode=est, lead=lead, vel_ff=vel_ff, kf_q=0.5, zem_gain=zem)
        res, _ = simulate_stack(DEF, layout, scen, noise=StackNoise(seed=i, **NOISE), **kw)
        ok += int(res.success)
    return ok / n


def main():
    f_small = dict(DEF['capture']['funnel'])
    f_big = dict(DEF['capture']['funnel']); f_big['mouth_radius'] = 0.30
    winds = [1, 2, 3, 4, 5]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.6))

    # (a) 侧风鲁棒性
    ax = axes[0, 0]
    curves = [
        ('baseline (raw, chase)', dict(lead=0, vel_ff=False, est='raw', funnel=f_small, zem=0)),
        ('+velocity FF', dict(lead=0, vel_ff=True, est='raw', funnel=f_small, zem=0)),
        ('+lead pursuit (1.0)', dict(lead=1, vel_ff=True, est='raw', funnel=f_small, zem=0)),
        ('+big funnel + windkf', dict(lead=1, vel_ff=True, est='windkf', funnel=f_big, zem=0)),
        ('+ZEM (1.0)', dict(lead=1, vel_ff=True, est='windkf', funnel=f_big, zem=1.0)),
    ]
    for name, kw in curves:
        ys = [mc(w, **kw) for w in winds]
        ax.plot(winds, np.array(ys) * 100, 'o-', label=name)
    ax.set_xlabel('wind speed (m/s)'); ax.set_ylabel('success rate (%)')
    ax.set_title('(a) Wind robustness: mechanisms stacked')
    ax.set_ylim(-2, 103); ax.legend(fontsize=7)

    # (b) ZEM 增益
    ax = axes[0, 1]
    zs = [0.0, 0.5, 1.0, 1.5, 2.0]
    for w in [3, 4, 5]:
        ys = [mc(w, lead=1, vel_ff=True, est='windkf', funnel=f_big, zem=z) for z in zs]
        ax.plot(zs, np.array(ys) * 100, 'o-', label=f'wind {w} m/s')
    ax.set_xlabel('ZEM gain'); ax.set_ylabel('success rate (%)')
    ax.set_title('(b) ZEM terminal-guidance gain sensitivity')
    ax.legend(fontsize=8)

    # (c) 漏斗物理
    ax = axes[1, 0]
    gaps = np.linspace(0.2, 2.0, 60)
    for a in [0.0, 2.0, 4.0, 6.0]:
        ax.plot(gaps, [contact_rel_speed(gp, a) for gp in gaps], label=f'a_dive={a:.0f}')
    for d, e, c in [(0.30, 0.60, 'k'), (0.50, 0.60, 'gray'), (0.50, 0.40, 'brown')]:
        ax.axhline(retain_speed(d, e), ls='--', color=c, label=f'v_retain d={d} e={e}')
    ax.set_xlabel('gap (m)'); ax.set_ylabel('contact rel. speed (m/s)')
    ax.set_title('(c) v_rel=sqrt(2(g-a)*gap) vs funnel v_retain'); ax.legend(fontsize=7)

    # (d) 末端机构能力
    ax = axes[1, 1]
    models = ['flat disk', 'hollow cone(30)', 'active latch']
    vt = [4.04, 4.04, 8.0]
    vh = [2.21, 3.13, 8.0]
    tilt = [45, 60, 180]
    x = np.arange(len(models))
    ax.bar(x - 0.26, vt, 0.26, label='v_retain (m/s)')
    ax.bar(x, vh, 0.26, label='v_h_tol (m/s)')
    ax.bar(x + 0.26, np.array(tilt) / 20.0, 0.26, label='tilt_max/20 (°)')
    ax.set_xticks(x); ax.set_xticklabels(models)
    ax.set_title('(d) end-effector capability (eff_r=0.25m)'); ax.legend(fontsize=8)

    fig.suptitle('Aerial drop-catch: current-work key results (offline)', fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = os.path.join(FIG, 'fig_cur_summary.png')
    fig.savefig(out)
    print('wrote', out)


if __name__ == '__main__':
    main()

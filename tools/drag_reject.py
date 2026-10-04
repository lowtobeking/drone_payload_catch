#!/usr/bin/env python3
"""drag_reject.py —— 抗（空气）阻力/风扰的离线研究（纯 Python，无 ROS）。

在 M6 垂直堆叠投送上，比较不同【阻力/风扰】下的估计器与控制：
  · 常值风（linear drag）—— 基线；
  · 二次阻力（quadratic drag）—— 更真实的空气动力学；
  · 正弦阵风（sinusoidal gust）—— 时变风；
  · 阶跃阵风（step gust）—— 突变风。
估计器：raw（有限差分）/ kf（纯弹道）/ windkf（估计常值风）。
并测试 windkf 的【风过程噪声 kf_qw】对跟踪阵风的作用。

用法: python3 tools/drag_reject.py
"""
from __future__ import annotations

import os
import statistics as st
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)

from payload_catch.stack_drop import simulate_stack, StackNoise  # noqa: E402

DEFAULT_YAML = os.path.join(REPO, 'config', 'catch_scenarios.yaml')


def load_cfg(p):
    with open(p, encoding='utf-8') as f:
        return yaml.safe_load(f)


def mc(defaults, funnel, disturb, n, **kw):
    """跑 n 次 MC，返回成功率/余量统计。"""
    ok = 0
    margins = []
    for i in range(n):
        layout = {'a_init': [0.0, 0.0, -(3.5 + disturb['gap'])],
                  'b_standby': [0.0, 0.0, -3.5]}
        scen = {'stack': {'a_dive': 3.0, 'a_brake': 6.0},
                'vert_mode': 'minimal',
                'capture': {'funnel': dict(funnel)},
                'payload': {'drag_mode': disturb['drag'],
                            'drag_k': disturb['k']},
                'wind': list(disturb['wind'])}
        if disturb.get('gust'):
            scen['gust'] = disturb['gust']
        noise = StackNoise(release_pos_sigma=0.05, rel_pos_sigma=0.05,
                           rel_latency=0.05, seed=i)
        res, _ = simulate_stack(defaults, layout, scen, noise=noise, **kw)
        ok += int(res.success)
        if res.success:
            margins.append(res.capture_margin)
    return ok, (st.mean(margins) if margins else float('nan'))


def main():
    cfg = load_cfg(DEFAULT_YAML)
    defaults = cfg['defaults']
    funnel = {**defaults['capture']['funnel']}
    funnel['mouth_radius'] = 0.30
    n = 30
    base_geom = dict(gap=0.7, drag='linear', k=1.0, wind=(2.0, 0.0, 0.0))

    disturbs = {
        '常值风(linear)': dict(base_geom),
        '二次阻力(quad)': dict(base_geom, drag='quadratic', k=0.10),
        '正弦阵风(0.3s)': dict(base_geom, gust=dict(type='sin', amp=1.5,
                                                 period=0.3, t_start=0.0)),
        '阶跃阵风': dict(base_geom, wind=(1.0, 0.0, 0.0),
                     gust=dict(type='step', amp=2.0, t_start=0.1)),
    }
    ests = [
        ('raw+lead1', dict(est_mode='raw', lead=1.0)),
        ('kf+lead1', dict(est_mode='kf', lead=1.0, kf_q=10.0)),
        ('windkf+lead1', dict(est_mode='windkf', lead=1.0, kf_q=0.5, kf_qw=0.05)),
        ('windkf+qw0.5', dict(est_mode='windkf', lead=1.0, kf_q=0.5, kf_qw=0.5)),
        ('windkf+qw2', dict(est_mode='windkf', lead=1.0, kf_q=0.5, kf_qw=2.0)),
    ]
    print(f'== M6 抗阻力/风扰（大漏斗 eff_r=0.25, gap=0.7 免下潜, n={n}/格）==')
    header = f"{'扰动':>16} | " + ' | '.join(f'{nm:>13}' for nm, _ in ests)
    print(header)
    for dname, d in disturbs.items():
        cells = []
        for _, kw in ests:
            ok, _mg = mc(defaults, funnel, d, n, **kw)
            cells.append(f'{ok:>2}/{n}')
        print(f'{dname:>16} | ' + ' | '.join(f'{c:>13}' for c in cells))
    print('\n（每格=成功率；越靠右=kf 越信任测量/风变化）')


if __name__ == '__main__':
    main()

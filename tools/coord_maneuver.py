#!/usr/bin/env python3
"""coord_maneuver.py —— A 速度/高度 × B 机动的【联合优化】（研究 W3 / C4）。

协调点：A 的**速度 v_A** 与**高度**是可调量，直接影响 B 的捕获余量（v_A·延迟、落差、相对速度）。
本工具在**固定干扰**（侧风 + 释放/定位噪声 + 延迟）下，联合搜索：

    A 速度 v_A  ×  A 高度(= h_B+gap)  ×  B 下潜 a_dive

目标：最大化【鲁棒捕获余量(p10)】，给出：
  · 速度—余量**权衡曲线**（A 的"协调代价"）；
  · 联合最优 vs 固定默认几何（去耦）的改进；
  · 移动交付（v_A>0）下的推荐几何。

用法：
  python3 tools/coord_maneuver.py [--wind 2.0] [--n 12]
"""
from __future__ import annotations

import argparse
import itertools
import math
import os
import statistics as st
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)

from payload_catch.stack_drop import (  # noqa: E402
    simulate_stack, plan_stack_drop, StackNoise, retain_speed)

DEFAULT_YAML = os.path.join(REPO, 'config', 'catch_scenarios.yaml')
G = 9.81


def load_cfg(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _pct(xs, q):
    if not xs:
        return float('nan')
    xs = sorted(xs)
    i = max(0, min(len(xs) - 1, int(round(q * (len(xs) - 1)))))
    return xs[i]


def evaluate(defaults, gap, h_b, a_dive, v_A, funnel, disturb, n):
    """联合几何 + A 速度下跑 n 次 MC（B 与 A 同速起飞，载荷继承 A 速度）。"""
    h_a = h_b + gap
    layout = {'a_init': [0.0, 0.0, -h_a], 'b_standby': [0.0, 0.0, -h_b]}
    scen = {
        'stack': {'a_dive': a_dive, 'a_brake': 6.0},
        'capture': {'funnel': dict(funnel)},
        'payload': {'drag_mode': 'linear', 'drag_k': disturb['k']},
        'wind': [disturb['wind'], 0.0, 0.0],
        'planner': {'ground_margin': 0.30},
        'v_release': [v_A, 0.0, 0.0],       # 载荷继承 A 的平飞速度
        'b_v0': [v_A, 0.0, 0.0],            # B 以同速跟飞（编队）
    }
    ok = 0
    margins = []
    for i in range(n):
        noise = StackNoise(release_pos_sigma=disturb['rel'],
                           rel_pos_sigma=disturb['rel'],
                           rel_latency=disturb['lat'], seed=i)
        res, _ = simulate_stack(defaults, layout, scen, noise=noise,
                                est_mode='windkf', lead=1.0, kf_q=0.5)
        if res.success:
            ok += 1
            margins.append(res.capture_margin)
    return dict(rate=ok / n,
                p10=_pct(margins, 0.10),
                mn=min(margins) if margins else float('nan'),
                mean=st.mean(margins) if margins else float('nan'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', default=DEFAULT_YAML)
    ap.add_argument('--wind', type=float, default=2.0)
    ap.add_argument('--n', type=int, default=12)
    ap.add_argument('--mouth', type=float, default=0.30)
    args = ap.parse_args()
    cfg = load_cfg(args.yaml)
    defaults = cfg['defaults']
    funnel = {**defaults['capture']['funnel']}
    funnel['mouth_radius'] = float(args.mouth)
    disturb = {'wind': args.wind, 'k': 1.0, 'rel': 0.05, 'lat': 0.05}

    v_As = [0.0, 0.5, 1.0, 1.5, 2.0]
    gaps = [0.7, 1.0, 1.3]
    h_bs = [3.0, 3.5, 4.0]
    a_dives = [1.0, 2.0, 3.0]

    print(f'== C4 联合机动优化（侧风 {args.wind} m/s, k=1; 释放/定位 σ=0.05, 延迟 0.05; '
          f'windkf+lead1; n={args.n}/点）==')
    print(f"\n{'v_A':>5} | {'最优几何 gap/h_B/a_dive':>22} | {'成功率':>6} {'p10余量':>8} "
          f"{'min余量':>8} | {'默认(1.0/3.5/3.0)成功率':>18} {'p10':>7}")
    best_by_v = {}
    for v_A in v_As:
        rows = []
        for gap, h_b, a_dive in itertools.product(gaps, h_bs, a_dives):
            m = evaluate(defaults, gap, h_b, a_dive, v_A, funnel, disturb, args.n)
            rows.append((m['rate'], m['p10'], m['mn'], m['mean'], gap, h_b, a_dive))
        rows.sort(key=lambda r: (r[0], r[1]), reverse=True)
        best = rows[0]
        best_by_v[v_A] = best
        # 默认几何对照
        d = evaluate(defaults, 1.0, 3.5, 3.0, v_A, funnel, disturb, args.n)
        print(f"{v_A:>5.1f} | {best[4]:>6.2f}/{best[5]:>4.1f}/{best[6]:>4.1f}      "
              f"| {best[0]*100:5.0f}% {best[1]:8.3f} {best[2]:8.3f} "
              f"| {d['rate']*100:17.0f}% {d['p10']:7.3f}")

    # 联合 vs 去耦（v_A=1.0）：只优化 B（固定 A 高度）vs 联合（含 A 高度）
    print('\n' + '=' * 78)
    print('联合 vs 去耦（v_A=1.0, 侧风 %.1f）：A 高度也是决策变量' % args.wind)
    print('=' * 78)
    print(f"{'方案':<28} {'gap':>5} {'h_B':>5} {'a_dive':>7} {'成功率':>6} {'p10余量':>8}")
    combos = [
        ('B-only：a_dive 最优(固定 gap=1.0)', [1.0], [3.5], a_dives),
        ('B-only：gap/a_dive 最优(固定 h_B=3.5)', gaps, [3.5], a_dives),
        ('联合：gap×h_B×a_dive', gaps, h_bs, a_dives),
    ]
    for name, gg, hh, aa in combos:
        rows = []
        for gap, h_b, a_dive in itertools.product(gg, hh, aa):
            m = evaluate(defaults, gap, h_b, a_dive, 1.0, funnel, disturb, args.n)
            rows.append((m['rate'], m['p10'], m['mn'], m['mean'], gap, h_b, a_dive))
        rows.sort(key=lambda r: (r[0], r[1]), reverse=True)
        b = rows[0]
        print(f"{name:<28} {b[4]:>5.2f} {b[5]:>5.1f} {b[6]:>7.1f} "
              f"{b[0]*100:5.0f}% {b[1]:>8.3f}")

    print('\n>>> 耦合本质：载荷继承 A 速度 v_A；B 需匹配并在延迟 d 内消除 v_A·d 漂移。')
    print('>>> 速度—余量曲线即"A 的协调代价"：v_A↑ ⇒ B 余量↓（需 A 配合限速/选择高度）。')


if __name__ == '__main__':
    main()

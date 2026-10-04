#!/usr/bin/env python3
"""geom_opt.py —— M6 垂直堆叠投送的【鲁棒几何/计划优化】（纯 Python，无 ROS）。

背景：现有规划器在 (t_r, τ_c) 上取**标称最优**；但真正决定成败的是**干扰下的最坏情况余量**。
本脚本把几何/计划当成设计变量（gap、B 待命高度 h_B、下潜加速度 a_dive），
在**固定干扰分布**（侧风 + 释放误差 + 相对定位噪声/延迟）下跑蒙特卡洛，
以【成功率】为主、【捕获余量(10 分位)】为辅，搜索鲁棒最优几何。

用法：
  python3 tools/geom_opt.py                 # 默认网格 + 侧风 2 m/s
  python3 tools/geom_opt.py --wind 3.0      # 更恶劣的侧风
  python3 tools/geom_opt.py --n 15          # 每点 MC 次数
  python3 tools/geom_opt.py --refine        # 在粗搜最优附近细化
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


def evaluate(defaults, base, gap, h_b, a_dive, g, funnel, disturb, n,
             est_kw=None):
    """给定几何跑 n 次 MC，返回指标 dict。"""
    h_a = h_b + gap
    layout = {'a_init': [0.0, 0.0, -h_a], 'b_standby': [0.0, 0.0, -h_b]}
    scen = {
        'stack': {'a_dive': a_dive, 'a_brake': 6.0},
        'capture': {'funnel': dict(funnel)},
        'payload': {'drag_mode': 'linear', 'drag_k': disturb['k']},
        'wind': [disturb['wind'], 0.0, 0.0],
        'planner': {'ground_margin': 0.30},
    }
    ekw = {'est_mode': 'windkf', 'lead': 1.0, 'kf_q': 0.5}
    if est_kw:
        ekw.update(est_kw)
    ok = 0
    margins = []
    for i in range(n):
        noise = StackNoise(release_pos_sigma=disturb['rel'],
                           rel_pos_sigma=disturb['rel'],
                           rel_latency=disturb['lat'], seed=i)
        res, plan = simulate_stack(defaults, layout, scen, noise=noise, **ekw)
        if res.success:
            ok += 1
            margins.append(res.capture_margin)
        elif not plan.feasible:
            pass
    return {
        'rate': ok / n,
        'mean_margin': st.mean(margins) if margins else float('nan'),
        'p10_margin': _pct(margins, 0.10),
        'min_margin': min(margins) if margins else float('nan'),
    }


def analytic(gap, h_b, a_dive, funnel, a_brake=6.0, ground=0.30):
    """闭式：v_rel、下潜/刹车、刹停后高度、v_retain。"""
    v_ret = retain_speed(funnel['depth'], funnel['restitution'])
    if a_dive >= G:
        return None
    t_c = math.sqrt(2 * gap / (G - a_dive))
    v_rel = (G - a_dive) * t_c
    dive = 0.5 * a_dive * t_c * t_c
    v_b = a_dive * t_c
    brake = v_b * v_b / (2 * a_brake)
    post_alt = h_b - dive - brake
    feasible = (post_alt >= ground) and (v_rel <= v_ret + 1e-9)
    return dict(v_rel=v_rel, post_alt=post_alt, v_ret=v_ret, feasible=feasible)


def run_grid(cfg, gaps, h_bs, a_dives, wind, n, mouth=0.30, min_gap=0.6, top=20):
    defaults = cfg['defaults']
    funnel = {**defaults['capture']['funnel']}
    funnel['mouth_radius'] = float(mouth)
    eff_r = funnel['mouth_radius'] - funnel['object_radius']
    disturb = {'wind': wind, 'k': 1.0, 'rel': 0.05, 'lat': 0.05}
    print(f'== 鲁棒几何搜索（侧风 {wind} m/s, k=1；漏斗 mouth={mouth} eff_r={eff_r:.2f}；'
          f'gap≥{min_gap}；释放/定位 σ=0.05, 延迟 0.05, n={n}/点, windkf+lead1）==')
    rows = []
    for gap, h_b, a_dive in itertools.product(gaps, h_bs, a_dives):
        if gap < min_gap - 1e-9:
            continue
        an = analytic(gap, h_b, a_dive, funnel)
        if an is None or not an['feasible']:
            continue
        m = evaluate(defaults, None, gap, h_b, a_dive, G, funnel, disturb, n)
        rows.append((m['rate'], m['p10_margin'], m['min_margin'], m['mean_margin'],
                     gap, h_b, a_dive, an['v_rel'], an['post_alt']))
    # 先按成功率、再按 p10 余量排序
    rows.sort(key=lambda r: (r[0], r[1]), reverse=True)
    print(f"\n{'成功率':>7} {'p10余量':>8} {'min余量':>8} {'均余量':>8} "
          f"{'gap':>5} {'h_B':>5} {'a_dive':>7} {'v_rel':>7} {'刹停后':>7}")
    for r in rows[:top]:
        print(f"{r[0]*100:6.0f}% {r[1]:8.3f} {r[2]:8.3f} {r[3]:8.3f} "
              f"{r[4]:5.2f} {r[5]:5.2f} {r[6]:7.2f} {r[7]:7.2f} {r[8]:7.2f}")
    print(f"\n共评估 {len(rows)} 个可行几何。")
    if rows:
        best = rows[0]
        # 另选“成功率≥90% 中 p10 余量最大”者作为稳健推荐
        rob = max((r for r in rows if r[0] >= 0.9), key=lambda r: r[1], default=rows[0])
        print(f"\n>>> 成功率最优：gap={best[4]:.2f} h_B={best[5]:.2f} a_dive={best[6]:.2f} "
              f"→ {best[0]*100:.0f}% / p10余量 {best[1]:.3f}")
        print(f">>> 稳健推荐（≥90% 中 p10 余量最大）：gap={rob[4]:.2f} h_B={rob[5]:.2f} "
              f"a_dive={rob[6]:.2f} → {rob[0]*100:.0f}% / p10余量 {rob[1]:.3f} / v_rel={rob[7]:.2f}")
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', default=DEFAULT_YAML)
    ap.add_argument('--wind', type=float, default=2.0)
    ap.add_argument('--n', type=int, default=15)
    ap.add_argument('--mouth', type=float, default=0.30)
    ap.add_argument('--min-gap', type=float, default=0.6)
    ap.add_argument('--refine', action='store_true')
    args = ap.parse_args()
    cfg = load_cfg(args.yaml)
    if args.refine:
        gaps = [0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2]
        h_bs = [2.4, 2.8, 3.0, 3.2, 3.6, 3.8]
        a_dives = [2.0, 2.5, 3.0, 3.5, 4.0]
    else:
        gaps = [0.6, 0.8, 1.0, 1.2, 1.5, 1.8]
        h_bs = [2.0, 2.5, 3.0, 3.5, 4.0]
        a_dives = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0]
    run_grid(cfg, gaps, h_bs, a_dives, args.wind, args.n,
             mouth=args.mouth, min_gap=args.min_gap)


if __name__ == '__main__':
    main()

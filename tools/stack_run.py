#!/usr/bin/env python3
"""stack_run.py —— M6 垂直堆叠投放的离线体检 CLI（纯 Python，不需要 ROS/SITL）。

用法:
  python3 tools/stack_run.py                       # 跑 M6_stack_drop
  python3 tools/stack_run.py --scenario M6_stack_drop
  python3 tools/stack_run.py --sweep-dive           # 扫 a_dive：冲击 vs 刹车余量
  python3 tools/stack_run.py --sweep-gap            # 扫 gap：接触速度 vs v_retain
  python3 tools/stack_run.py --mc 50                # 蒙特卡洛（相对定位噪声/延迟）
"""
from __future__ import annotations

import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)

from payload_catch.stack_drop import (  # noqa: E402
    simulate_stack, plan_stack_drop, StackNoise, retain_speed)

DEFAULT_YAML = os.path.join(REPO, 'config', 'catch_scenarios.yaml')


def load_cfg(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def print_plan(plan):
    print(f'  [规划] feasible={plan.feasible} reason={plan.reason}')
    if not plan.feasible:
        return
    print(f'         gap={plan.gap:.2f}m  a_dive={plan.a_dive:.2f} m/s²  t_c={plan.t_c:.3f}s')
    print(f'         接触相对速度={plan.v_rel:.3f} m/s  载荷速度={plan.v_pay:.2f}  B速度={plan.v_b:.2f}')
    print(f'         接触高度={plan.z_c:.3f}m  B下潜={plan.dive:.3f}m  刹停后高度={plan.post_brake_alt:.3f}m')
    print(f'         漏斗保持速度 v_retain={plan.v_retain:.3f} m/s  (余量 {plan.v_retain-plan.v_rel:+.3f})')


def print_result(res):
    if res.success:
        print(f'  [仿真] PASS  捕获时刻={res.t_capture:.3f}s  最近距离={res.miss_dist:.4f}m  '
              f'捕获相对速度={res.rel_speed_at_capture:.3f} m/s')
        print(f'         捕获时水平偏差={res.horiz_miss_at_capture:.4f}m  '
              f'B 最低离地={res.b_min_alt:.3f}m  峰值|a|={res.peak_accel:.2f}  |v|={res.peak_speed:.2f}')
    else:
        print(f'  [仿真] FAIL  最近距离={res.miss_dist:.4f}m  B 最低离地={res.b_min_alt:.3f}m')


def run_one(cfg, name):
    defaults = cfg['defaults']
    scen = cfg['scenarios'][name]
    layout = cfg['layouts'][scen['layout']]
    res, plan = simulate_stack(defaults, layout, scen)
    print(f'== 工况 {name} ==')
    print_plan(plan)
    print_result(res)
    return res, plan


def sweep_dive(cfg, name):
    defaults = cfg['defaults']
    scen = cfg['scenarios'][name]
    layout = cfg['layouts'][scen['layout']]
    a_h, b_h = -layout['a_init'][2], -layout['b_standby'][2]
    funnel = {**defaults['capture'].get('funnel', {}), **scen.get('capture', {}).get('funnel', {})}
    print(f'== 扫 a_dive (gap={a_h-b_h:.2f}m, 漏斗 depth={funnel["depth"]}m e={funnel["restitution"]}) ==')
    print(f"{'a_dive':>7} {'t_c(s)':>7} {'v_rel':>7} {'接触高度':>9} {'刹停后':>8} {'可行':>5}")
    for a_dive in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0):
        s = dict(scen)
        s['stack'] = {**scen.get('stack', {}), 'a_dive': a_dive}
        res, plan = simulate_stack(defaults, layout, s)
        if plan.t_c == plan.t_c:  # not nan
            print(f"{a_dive:7.1f} {plan.t_c:7.3f} {plan.v_rel:7.3f} {plan.z_c:9.3f} "
                  f"{plan.post_brake_alt:8.3f} {str(plan.feasible):>5}")
        else:
            print(f"{a_dive:7.1f} {'--':>7} {'--':>7} {'--':>9} {'--':>8} {str(plan.feasible):>5}  ({plan.reason})")


def sweep_gap(cfg, name):
    defaults = cfg['defaults']
    scen = cfg['scenarios'][name]
    layout = cfg['layouts'][scen['layout']]
    b_h = -layout['b_standby'][2]
    a_dive = float({**defaults.get('stack', {}), **scen.get('stack', {})}.get('a_dive', 3.0))
    funnel = {**defaults['capture'].get('funnel', {}), **scen.get('capture', {}).get('funnel', {})}
    print(f'== 扫 gap (a_dive={a_dive}, 漏斗 depth={funnel["depth"]}m e={funnel["restitution"]}) ==')
    print(f"{'gap':>5} {'A高度':>6} {'t_c(s)':>7} {'v_rel':>7} {'接触高度':>9} {'可行':>5}")
    for gap in (0.3, 0.5, 0.8, 1.0, 1.2, 1.5):
        s = dict(scen)
        s['a_init'] = [0.0, 0.0, -(b_h + gap)]
        res, plan = simulate_stack(defaults, layout, s)
        if plan.t_c == plan.t_c:
            print(f"{gap:5.2f} {b_h+gap:6.2f} {plan.t_c:7.3f} {plan.v_rel:7.3f} {plan.z_c:9.3f} {str(plan.feasible):>5}")
        else:
            print(f"{gap:5.2f} {b_h+gap:6.2f} {'--':>7} {'--':>7} {'--':>9} {str(plan.feasible):>5}  ({plan.reason})")


def run_mc(cfg, name, n):
    defaults = cfg['defaults']
    scen = cfg['scenarios'][name]
    layout = cfg['layouts'][scen['layout']]
    funnel = {**defaults['capture'].get('funnel', {}), **scen.get('capture', {}).get('funnel', {})}
    eff_r = float(funnel.get('mouth_radius', 0.20)) - float(funnel.get('object_radius', 0.05))
    ok = 0
    miss, relv, hmiss = [], [], []
    for i in range(n):
        noise = StackNoise(release_pos_sigma=0.05, rel_pos_sigma=0.05, rel_latency=0.05, seed=i)
        res, _ = simulate_stack(defaults, layout, scen, noise=noise)
        ok += int(res.success)
        miss.append(res.miss_dist)
        if res.success:
            relv.append(res.rel_speed_at_capture)
            hmiss.append(res.horiz_miss_at_capture)
    import statistics as st
    print(f'== 蒙特卡洛 {name} n={n} ==')
    print(f'  成功率 = {ok}/{n} ({100.0*ok/n:.0f}%)')
    print(f'  最近距离: mean={st.mean(miss):.4f}m  max={max(miss):.4f}m')
    if relv:
        print(f'  捕获相对速度: mean={st.mean(relv):.3f} m/s  max={max(relv):.3f}')
        print(f'  捕获时水平偏差: mean={st.mean(hmiss):.4f}m  max={max(hmiss):.4f}m  '
              f'(口内有效半径={eff_r:.2f}m)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', default=DEFAULT_YAML)
    ap.add_argument('--scenario', default='M6_stack_drop')
    ap.add_argument('--sweep-dive', action='store_true')
    ap.add_argument('--sweep-gap', action='store_true')
    ap.add_argument('--mc', type=int, default=0)
    args = ap.parse_args()
    cfg = load_cfg(args.yaml)
    if args.sweep_dive:
        sweep_dive(cfg, args.scenario); return
    if args.sweep_gap:
        sweep_gap(cfg, args.scenario); return
    if args.mc > 0:
        run_mc(cfg, args.scenario, args.mc); return
    run_one(cfg, args.scenario)


if __name__ == '__main__':
    main()

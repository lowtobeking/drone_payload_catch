#!/usr/bin/env python3
"""offline_run.py —— 离线跑通"空投—空中捕获"任务并出体检报告（纯 Python，不需要 ROS/SITL）。

用法:
  python3 tools/offline_run.py                       # 跑 M1_basic
  python3 tools/offline_run.py --scenario M1_wind    # 指定工况
  python3 tools/offline_run.py --all                 # 跑全部工况
  python3 tools/offline_run.py --plot                # 另存轨迹图(需 matplotlib)
  python3 tools/offline_run.py --sweep-noise         # 释放/B 初值噪声鲁棒性扫描

判据来自 config/catch_scenarios.yaml 的 thresholds。
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)

from payload_catch.sim_core import simulate, SimNoise, SimResult   # noqa: E402
from payload_catch.rendezvous import HoverReleasePlanner, PlanResult  # noqa: E402

DEFAULT_YAML = os.path.join(REPO, 'config', 'catch_scenarios.yaml')


def load_cfg(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def print_plan(plan: PlanResult):
    print(f'  [规划] feasible={plan.feasible} reason={plan.reason}')
    if not plan.feasible:
        return
    print(f'         释放 t_r={plan.t_r:.3f}s  捕获 t_c={plan.t_c:.3f}s  '
          f'下落 tau_c={plan.tau_c:.3f}s  捕获高度 h_c={plan.h_c:.3f}m')
    print(f'         捕获点 p_c=({plan.p_c[0]:+.3f},{plan.p_c[1]:+.3f},{plan.p_c[2]:+.3f}) NED  '
          f'载荷速度 v_p=({plan.v_p[0]:+.2f},{plan.v_p[1]:+.2f},{plan.v_p[2]:+.2f})')
    print(f'         B 峰值 |a|={plan.peak_accel:.2f} m/s²  |v|={plan.peak_speed:.2f} m/s  '
          f'最低离地={plan.min_alt:.2f}m  速度失配={plan.delta_v:.4f} m/s  代价={plan.cost:.4f}')


def print_result(res: SimResult, th: dict):
    tag = 'PASS' if res.success else 'FAIL'
    miss_ok = res.miss_dist <= th.get('miss_dist', 0.3)
    print(f'  [仿真] {tag}  捕获时刻={res.t_capture if res.success else float("nan"):.3f}s  '
          f'最近距离={res.miss_dist:.4f}m ({"OK" if miss_ok else "超阈值"})')
    if res.success:
        rv_ok = res.rel_speed_at_capture <= th.get('rel_speed', 1.5)
        print(f'         捕获时相对速度={res.rel_speed_at_capture:.4f} m/s '
              f'({"OK" if rv_ok else "超阈值"})')
    print(f'         B 峰值 |a|={res.peak_accel:.2f} m/s²  |v|={res.peak_speed:.2f} m/s')
    if res.note:
        print(f'         备注: {res.note}')


def run_one(cfg, name, plot=False):
    defaults = cfg['defaults']
    scen = cfg['scenarios'][name]
    layout = cfg['layouts'][scen['layout']]
    th = {**cfg.get('thresholds', {}), **scen.get('thresholds', {})}
    print(f'== 工况 {name} ==')
    res, plan = simulate(defaults, layout, scen)
    print_plan(plan)
    print_result(res, th)
    if plot:
        _plot(res, plan, name)
    return res


def _plot(res: SimResult, plan: PlanResult, name: str):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as e:  # noqa: BLE001
        print(f'  (跳过绘图: {e})')
        return
    t = np.asarray(res.t)
    bp = np.asarray(res.b_pos)
    pp = np.asarray(res.p_pos)
    outdir = os.path.join(REPO, 'report', 'figures')
    os.makedirs(outdir, exist_ok=True)
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    # 俯视 (N-E)
    ax[0].plot(bp[:, 0], bp[:, 1], label='B')
    ax[0].plot(pp[:, 0], pp[:, 1], '--', label='payload')
    ax[0].plot([plan.p_c[0]], [plan.p_c[1]], 'r*', ms=12, label='planned catch')
    ax[0].set_xlabel('North [m]'); ax[0].set_ylabel('East [m]')
    ax[0].set_title('top view (N-E)'); ax[0].axis('equal'); ax[0].legend()
    # 侧视 (N-D)，方便看下降
    ax[1].plot(bp[:, 0], -bp[:, 2], label='B alt')
    ax[1].plot(pp[:, 0], -pp[:, 2], '--', label='payload alt')
    ax[1].set_xlabel('North [m]'); ax[1].set_ylabel('altitude [m]')
    ax[1].set_title('side view (N-alt)'); ax[1].legend()
    # 相对距离
    ax[2].plot(t, res.rel_dist)
    ax[2].axhline(plan.h_c * 0 + 0.0, color='k', lw=0.5)
    ax[2].set_xlabel('t [s]'); ax[2].set_ylabel('|B−payload| [m]')
    ax[2].set_title(f'relative distance (min={res.miss_dist:.3f} m)')
    fig.tight_layout()
    out = os.path.join(outdir, f'offline_{name}.png')
    fig.savefig(out, dpi=130)
    print(f'  [图] {out}')


def run_sweep(cfg, name):
    """释放/B 初值/载荷状态噪声扫描：看捕获成功率。"""
    defaults = cfg['defaults']
    scen = cfg['scenarios'][name]
    layout = cfg['layouts'][scen['layout']]
    print(f'== 鲁棒性扫描 {name} ==')
    # 固定规划（标称），噪声下重跑，看"标称规划 + 扰动"的鲁棒性
    res0, plan = simulate(defaults, layout, scen)
    print_plan(plan)
    levels = [0.0, 0.05, 0.10, 0.20]
    for sigma in levels:
        n = 40
        n_ok = 0
        miss = []
        relv = []
        for k in range(n):
            noise = SimNoise(release_pos_sigma=sigma, release_vel_sigma=sigma,
                             b_pos_sigma=sigma, payload_meas_sigma=0.0, seed=1000 + k)
            r, _ = simulate(defaults, layout, scen, plan=plan, noise=noise)
            n_ok += int(r.success)
            miss.append(r.miss_dist)
            if r.success:
                relv.append(r.rel_speed_at_capture)
        rv = f'{np.mean(relv):.3f}' if relv else 'n/a'
        print(f'  sigma={sigma:.2f} m: 成功 {n_ok}/{n}  '
              f'平均最近 {np.mean(miss):.3f}m  成功时平均相对速度 {rv} m/s')
    return res0


def main():
    ap = argparse.ArgumentParser(description='离线空投—捕获任务体检')
    ap.add_argument('--config', default=DEFAULT_YAML)
    ap.add_argument('--scenario', default='M1_basic')
    ap.add_argument('--all', action='store_true', help='跑全部工况')
    ap.add_argument('--plot', action='store_true')
    ap.add_argument('--sweep-noise', action='store_true')
    args = ap.parse_args()

    cfg = load_cfg(args.config)
    names = list(cfg['scenarios']) if args.all else [args.scenario]
    for n in names:
        if n not in cfg['scenarios']:
            sys.exit(f'未知工况 "{n}"，可选: {list(cfg["scenarios"])}')
        if args.sweep_noise:
            run_sweep(cfg, n)
        else:
            run_one(cfg, n, plot=args.plot)
        print()


if __name__ == '__main__':
    main()

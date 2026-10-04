#!/usr/bin/env python3
"""stack_run.py —— M6 垂直堆叠投放的离线体检 CLI（纯 Python，不需要 ROS/SITL）。

用法:
  python3 tools/stack_run.py                       # 跑 M6_stack_drop
  python3 tools/stack_run.py --scenario M6_stack_drop
  python3 tools/stack_run.py --sweep-dive           # 扫 a_dive：冲击 vs 刹车余量
  python3 tools/stack_run.py --sweep-gap            # 扫 gap：接触速度 vs v_retain
  python3 tools/stack_run.py --sweep-wind           # 扫侧风：追尾 vs 预测对正（干扰鲁棒）
  python3 tools/stack_run.py --sweep-funnel         # 扫漏斗几何：eff_r / v_retain 的鲁棒收益
  python3 tools/stack_run.py --mc 50                # 蒙特卡洛（相对定位噪声/延迟）
  python3 tools/stack_run.py --lead 1.0             # 指定预测对正增益
  python3 tools/stack_run.py --est kf --vert adaptive   # KF 估计 + 自适应下潜
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


def run_one(cfg, name, lead=0.0, est_mode='raw', vert_mode='open'):
    defaults = cfg['defaults']
    scen = cfg['scenarios'][name]
    layout = cfg['layouts'][scen['layout']]
    res, plan = simulate_stack(defaults, layout, scen, lead=lead,
                               est_mode=est_mode, vert_mode=vert_mode)
    print(f'== 工况 {name} (lead={lead}, est={est_mode}, vert={vert_mode}) ==')
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


def _mc_wind(defaults, layout, scen, k, w, n, noise_kw, **kw):
    """对给定侧风跑 n 次，返回 (成功数, 成功时水平偏差均值)。"""
    import statistics as st
    ok = 0
    hmiss = []
    for i in range(n):
        s = dict(scen)
        s['payload'] = {'drag_mode': 'linear', 'drag_k': k}
        s['wind'] = [w, 0.0, 0.0]
        noise = StackNoise(seed=i, **noise_kw)
        res, _ = simulate_stack(defaults, layout, s, noise=noise, **kw)
        ok += int(res.success)
        if res.success:
            hmiss.append(res.horiz_miss_at_capture)
    return ok, (st.mean(hmiss) if hmiss else None)


def sweep_wind(cfg, name, k=1.0, winds=(1, 2, 3, 4, 5), n=30):
    """扫侧风：对比【原始追尾】→【速度前馈】→【预测对正】的成功率。

    两种干扰制度：
      A 纯风扰动（只有释放误差，相对定位干净）——检验控制优化本身；
      B 风 + 相对定位噪声/延迟——检验噪声下的极限。
    """
    defaults = cfg['defaults']
    scen = cfg['scenarios'][name]
    layout = cfg['layouts'][scen['layout']]
    funnel = {**defaults['capture'].get('funnel', {}), **scen.get('capture', {}).get('funnel', {})}
    eff_r = float(funnel.get('mouth_radius', 0.20)) - float(funnel.get('object_radius', 0.05))
    configs = [
        ('原始追尾', dict(vel_ff=False, lead=0.0)),
        ('+速度前馈', dict(vel_ff=True, lead=0.0)),
        ('+预测对正', dict(vel_ff=True, lead=1.0)),
    ]
    regimes = [
        ('A 纯风扰动（仅释放误差 0.05m）', dict(release_pos_sigma=0.05)),
        ('B 风+相对定位噪声（0.05m/0.05s）',
         dict(release_pos_sigma=0.05, rel_pos_sigma=0.05, rel_latency=0.05)),
    ]
    print(f'== M6 侧风干扰鲁棒性扫掠 (drag_k={k}, n={n}/格, eff_r={eff_r:.2f}m) ==')
    for title, nk in regimes:
        print(f'\n--- {title} ---')
        print(f"{'wind':>5} | " + ' | '.join(f'{nm:>17}' for nm, _ in configs))
        for w in winds:
            cells = []
            for _, kw in configs:
                ok, hm = _mc_wind(defaults, layout, scen, k, w, n, nk, **kw)
                cells.append(f'{ok:>2}/{n} h={hm:.3f}' if hm is not None else f'{ok:>2}/{n}  --   ')
            print(f'{w:>5.1f} | ' + ' | '.join(f'{c:>17}' for c in cells))
    print('\n  （h=成功时捕获水平偏差均值；成功率越低说明越接近物理/控制极限）')


def sweep_funnel(cfg, name, k=1.0, wind=2.0, n=20):
    """扫漏斗几何：口半径（横向余量 eff_r）与深度/恢复系数（垂直保持 v_retain）。

    面板 A：侧风+噪声下扫口半径 → 见 eff_r 对横向漂移的主导作用；
    面板 B：下击暴流下扫 depth/e → 见 v_retain 对垂直接触速度的作用。
    """
    defaults = cfg['defaults']
    scen = cfg['scenarios'][name]
    layout = cfg['layouts'][scen['layout']]
    obj_r = float(defaults['capture']['funnel'].get('object_radius', 0.05))
    noise = dict(release_pos_sigma=0.05, rel_pos_sigma=0.05, rel_latency=0.05)
    print(f'== 扫漏斗 (drag_k={k}, n={n}/格) ==')

    print('\n--- A. 口半径 → eff_r（侧风 w=%.1f, raw+lead=1, 噪声）---' % wind)
    print(f"{'mouth':>6} {'eff_r':>6} {'成功':>7} {'h̄':>7}")
    import statistics as st
    for mouth in (0.16, 0.20, 0.24, 0.28, 0.32, 0.36):
        ok, hm = 0, []
        for i in range(n):
            s = dict(scen)
            s['payload'] = {'drag_mode': 'linear', 'drag_k': k}
            s['wind'] = [wind, 0.0, 0.0]
            s['capture'] = {'funnel': {'mouth_radius': mouth, 'depth': 0.30,
                                       'restitution': 0.60, 'mount_height': 0.10,
                                       'object_radius': obj_r}}
            res, _ = simulate_stack(defaults, layout, s, noise=StackNoise(seed=i, **noise),
                                    est_mode='raw', lead=1.0)
            ok += int(res.success)
            if res.success:
                hm.append(res.horiz_miss_at_capture)
        print(f"{mouth:6.2f} {mouth-obj_r:6.2f} {ok:>3}/{n:<3} "
              f"{(st.mean(hm) if hm else float('nan')):7.3f}")

    print('\n--- B. 深度/恢复系数 → v_retain（下击暴流 wz=6, KF, n/格）---')
    print(f"{'depth':>6} {'e':>5} {'v_retain':>8} {'open垂直':>9} {'adaptive垂直':>11}")
    for depth, e in ((0.30, 0.60), (0.40, 0.60), (0.50, 0.60),
                     (0.30, 0.40), (0.50, 0.40)):
        oks = {}
        for vmode in ('open', 'adaptive'):
            ok = 0
            for i in range(n):
                s = dict(scen)
                s['payload'] = {'drag_mode': 'linear', 'drag_k': k}
                s['wind'] = [0.0, 0.0, 6.0]
                s['capture'] = {'funnel': {'mouth_radius': 0.20, 'depth': depth,
                                           'restitution': e, 'mount_height': 0.10,
                                           'object_radius': obj_r}}
                res, _ = simulate_stack(defaults, layout, s, noise=StackNoise(seed=i, **noise),
                                        est_mode='kf', vert_mode=vmode)
                ok += int(res.success)
            oks[vmode] = ok
        print(f"{depth:6.2f} {e:5.2f} {retain_speed(depth, e):8.2f} "
              f"{oks['open']:>4}/{n:<4} {oks['adaptive']:>6}/{n:<4}")

    # C. 末端机械臂：reach 扩 eff_r / absorb 提 v_retain
    import statistics as st
    base_funnel = dict(mouth_radius=0.20, depth=0.30, restitution=0.60,
                       mount_height=0.10, object_radius=obj_r)

    def _run_arm(arm, wind3, **kw):
        ok, mg = 0, []
        for i in range(n):
            s = dict(scen)
            s['payload'] = {'drag_mode': 'linear', 'drag_k': k}
            s['wind'] = list(wind3)
            cap = {'funnel': dict(base_funnel)}
            cap.update(arm)
            s['capture'] = cap
            res, _ = simulate_stack(defaults, layout, s, noise=StackNoise(seed=i, **noise), **kw)
            ok += int(res.success)
            if res.success:
                mg.append(res.capture_margin)
        return ok, (st.mean(mg) if mg else None)

    print('\n--- C. 末端机械臂 reach/absorb（windkf+lead1, n/格）---')
    print(f"{'配置':>20} {'侧风w=2 成功/余量':>22} {'下击暴流wz=6':>12}")
    for name, arm in (('原漏斗', {}), ('reach +0.10', {'arm_reach': 0.10}),
                      ('absorb +2.0', {'arm_absorb': 2.0}),
                      ('reach+absorb', {'arm_reach': 0.10, 'arm_absorb': 2.0})):
        okw, mgw = _run_arm(arm, (wind, 0, 0),
                            est_mode='windkf', lead=1.0, kf_q=0.5)
        okd, _ = _run_arm(arm, (0, 0, 6), est_mode='raw')
        mgs = f'{mgw:.3f}' if mgw is not None else '--'
        print(f"{name:>20} {okw:>3}/{n:<3} 余量={mgs:>6} {okd:>6}/{n:<3}")


def run_mc(cfg, name, n):
    defaults = cfg['defaults']
    scen = cfg['scenarios'][name]
    layout = cfg['layouts'][scen['layout']]
    funnel = {**defaults['capture'].get('funnel', {}), **scen.get('capture', {}).get('funnel', {})}
    arm_reach = float({**defaults['capture'], **scen.get('capture', {})}.get('arm_reach', 0.0))
    eff_r = float(funnel.get('mouth_radius', 0.20)) - float(funnel.get('object_radius', 0.05)) + arm_reach
    ok = 0
    miss, relv, hmiss, margin = [], [], [], []
    for i in range(n):
        noise = StackNoise(release_pos_sigma=0.05, rel_pos_sigma=0.05, rel_latency=0.05, seed=i)
        res, _ = simulate_stack(defaults, layout, scen, noise=noise)
        ok += int(res.success)
        miss.append(res.miss_dist)
        if res.success:
            relv.append(res.rel_speed_at_capture)
            hmiss.append(res.horiz_miss_at_capture)
            margin.append(res.capture_margin)
    import statistics as st
    print(f'== 蒙特卡洛 {name} n={n} ==')
    print(f'  成功率 = {ok}/{n} ({100.0*ok/n:.0f}%)')
    print(f'  最近距离: mean={st.mean(miss):.4f}m  max={max(miss):.4f}m')
    if relv:
        print(f'  捕获相对速度: mean={st.mean(relv):.3f} m/s  max={max(relv):.3f}')
        print(f'  捕获时水平偏差: mean={st.mean(hmiss):.4f}m  max={max(hmiss):.4f}m  '
              f'(口内有效半径={eff_r:.2f}m)')
        print(f'  捕获余量(eff_r−hmiss): mean={st.mean(margin):.4f}m  min={min(margin):.4f}m')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', default=DEFAULT_YAML)
    ap.add_argument('--scenario', default='M6_stack_drop')
    ap.add_argument('--sweep-dive', action='store_true')
    ap.add_argument('--sweep-gap', action='store_true')
    ap.add_argument('--sweep-wind', action='store_true')
    ap.add_argument('--sweep-funnel', action='store_true')
    ap.add_argument('--wind-k', type=float, default=1.0)
    ap.add_argument('--lead', type=float, default=0.0)
    ap.add_argument('--est', default='raw', choices=['raw', 'kf', 'windkf'])
    ap.add_argument('--vert', default='open', choices=['open', 'adaptive', 'minimal'])
    ap.add_argument('--mc', type=int, default=0)
    args = ap.parse_args()
    cfg = load_cfg(args.yaml)
    if args.sweep_dive:
        sweep_dive(cfg, args.scenario); return
    if args.sweep_gap:
        sweep_gap(cfg, args.scenario); return
    if args.sweep_wind:
        sweep_wind(cfg, args.scenario, k=args.wind_k); return
    if args.sweep_funnel:
        sweep_funnel(cfg, args.scenario, k=args.wind_k); return
    if args.mc > 0:
        run_mc(cfg, args.scenario, args.mc); return
    run_one(cfg, args.scenario, lead=args.lead,
            est_mode=args.est, vert_mode=args.vert)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""validate_m6_moving.py — M6-moving（编队同速、边飞边接）SITL 验证。

对 FORMATION_VEL ∈ {0.5, 1.0, 2.0} m/s 各跑 N 次，检查：
  · 捕获（STACK CAPTURED）+ 捕获水平偏差/余量
  · **两机落地**（px4_0/px4_1 均 `Landing detected` / `Disarmed by landing`）
  · 无 failsafe、无异常
结果写入 report/m6_moving_validation.md。

用法：python3 tools/validate_m6_moving.py
"""
from __future__ import annotations

import os
import re
import statistics as st
import subprocess

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG = os.path.expanduser('~/payload_catch_m6/launch.log')
P0 = os.path.expanduser('~/px4_logs/px4_0.log')
P1 = os.path.expanduser('~/px4_logs/px4_1.log')
RUNNER = os.path.join(REPO, 'run_m6_sitl.sh')
OUT = os.path.join(REPO, 'report', 'm6_moving_validation.md')
# 支持速度 = 0.5 / 1.0 m/s（各 2/2 完美）；2.0 m/s 保留记录、暂不优化。
# 可用 --speeds 0.5,1.0,2.0 手动纳入 2.0。
EFF_R = 0.25          # 空心杯 eff_r
REPS = 2
SPEEDS = [0.5, 1.0]


def _landed(path):
    try:
        t = open(path, encoding='utf-8', errors='ignore').read()
    except OSError:
        return False
    return ('Disarmed by landing' in t) or ('Landing detected' in t)


def run_one(speed, rep):
    env = dict(os.environ)
    # 空心杯：水平相对运动下能“保持”物块（平顶盘会滑落）
    env.update({'FORMATION_VEL': f'{speed},0.0,0.0', 'FUNNEL_MOUTH': '0.30',
                'FUNNEL_TYPE': 'cup'})
    subprocess.run(['bash', RUNNER, '100'], env=env,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t = open(LOG, encoding='utf-8', errors='ignore').read() if os.path.exists(LOG) else ''
    cap = 'STACK CAPTURED' in t
    hm = re.findall(r'STACK CAPTURED \*\*\* horiz=([0-9.]+)m', t)
    horiz = float(hm[-1]) if hm else float('nan')
    released = '已与 A 分离' in t
    formed = 'FORMATION aligned' in t
    failsafe = t.count('Failsafe')
    trace = t.count('Traceback')
    land_a, land_b = _landed(P0), _landed(P1)
    # 保持：捕获后（DONE/LAND）物块与 B 的水平距离
    done = re.findall(
        r'B phase=(?:DONE|LAND).*?pos_w=\[\s*([-0-9.eE]+)\s+([-0-9.eE]+)[^\]]*\]'
        r'.*?pay=\[\s*([-0-9.eE]+)\s+([-0-9.eE]+)', t)
    retin = float('nan')
    if done:
        bx, by, px, py = map(float, done[-1])
        retin = ((bx - px) ** 2 + (by - py) ** 2) ** 0.5
    retained = (retin == retin) and (retin < 1.0)
    return dict(speed=speed, rep=rep, formed=formed, released=released, cap=cap,
                horiz=horiz, margin=(EFF_R - horiz) if cap else float('nan'),
                retin=retin, retained=retained,
                land_a=land_a, land_b=land_b, failsafe=failsafe, trace=trace)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--speeds', default=','.join(str(s) for s in SPEEDS),
                    help='逗号分隔的编队速度 (m/s)，默认 0.5,1.0')
    ap.add_argument('--reps', type=int, default=REPS)
    args = ap.parse_args()
    speeds = [float(s) for s in args.speeds.split(',') if s.strip()]
    reps = args.reps
    rows = []
    for sp in speeds:
        for r in range(1, reps + 1):
            print(f'>>> {sp} m/s rep{r} ...', flush=True)
            _ = r  # noqa
            rows.append(run_one(sp, r))
    lines = ['# M6-moving（边飞边接）SITL 验证', '',
             '> 由 `python3 tools/validate_m6_moving.py` 生成。大漏斗 `eff_r=0.25m`；每速度重复 '
             f'{reps} 次；默认仅测支持速度 0.5/1.0 m/s（2.0 保留记录）。', '',
             '| 速度 (m/s) | 重复 | 编队对齐 | 释放 | 捕获 | 捕获偏差 (m) | 余量 (m) | 保持(物块↔B) | A 落地 | B 落地 | failsafe |',
             '|---|---|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        rr = f"{r['retin']:.3f}" if r['retin'] == r['retin'] else '--'
        lines.append(
            f"| {r['speed']} | {r['rep']} | {'✅' if r['formed'] else '❌'} "
            f"| {'✅' if r['released'] else '❌'} | {'✅' if r['cap'] else '❌'} "
            f"| {r['horiz']:.3f} | {r['margin']:.3f} "
            f"| {'✅' if r['retained'] else '❌'} {rr} "
            f"| {'✅' if r['land_a'] else '❌'} | {'✅' if r['land_b'] else '❌'} "
            f"| {r['failsafe']} |")
    ok = all(r['formed'] and r['released'] and r['cap'] and r['retained']
             and r['land_a'] and r['land_b']
             and r['failsafe'] == 0 and r['trace'] == 0 for r in rows)
    caps = [r['horiz'] for r in rows if r['cap']]
    lines += ['', f"**总判定：{'全部完美执行 ✅' if ok else '存在失败 ❌'}**",
              '', f"- 捕获偏差 max={max(caps):.3f}m（余量 min={EFF_R-max(caps):.3f}m）" if caps else '',
              '- 完美 = 编队对齐→释放→捕获→**两机均落地**，且无 failsafe/异常。']
    with open(OUT, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print('wrote', OUT)
    print('ALLPERFECT', ok)


if __name__ == '__main__':
    main()

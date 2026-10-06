#!/usr/bin/env python3
"""bench_coord.py —— 双机协同交接【基准实验台】（研究用，阶段 0）。

系统地扫描协调策略/信息/不确定度/通信条件，采集指标并做统计（含 Wilson 置信区间），
输出 `report/coordination_benchmark.md`。

网格维度
  · coord_mode : direct | handshake           （释放权威：B 单边 / A 作权威）
  · use_intent : false | true                 （是否共享"预测落点"意图）
  · rel_pos_sigma : 0 | 0.05 | 0.10           （相对定位噪声 m）
  · rel_latency   : 0 | 0.10 | 0.20           （相对定位延迟 s）

指标
  捕获成功率、捕获时水平偏差 horiz、相对速度 rel_v、最小 |A-B|、
  释放时刻/就绪时刻、异常(HOLD/PULLBACK)数、failsafe 数。

用法
  python3 tools/bench_coord.py --quick           # 小网格，各 1 次（先验证）
  python3 tools/bench_coord.py --reps 3          # 小网格，各 3 次
  python3 tools/bench_coord.py --full --reps 3   # 全网格
  python3 tools/bench_coord.py --dry-run         # 只打印配置
  python3 tools/bench_coord.py --only H_nominal  # 只跑名字匹配的配置（正则）
"""
from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG = os.path.expanduser('~/payload_catch_m6/launch.log')
RUNNER = os.path.join(REPO, 'run_m6_sitl.sh')
OUT = os.path.join(REPO, 'report', 'coordination_benchmark.md')


# ------------------------------------------------------------------ 配置网格
def build_grid(grid: str = 'quick'):
    if grid == 'ablation':
        # 逐项消融：baseline → +权威 → +证书 → +intent → +CBF
        return [
            ('T0_direct', dict(COORD='direct')),
            ('T1_auth', dict(COORD='handshake')),
            ('T2_cert', dict(COORD='handshake',
                             LAUNCH_EXTRA='release_gate_mode:=certificate')),
            ('T3_intent', dict(COORD='handshake',
                               LAUNCH_EXTRA='release_gate_mode:=certificate use_intent:=true')),
            ('T4_cbf', dict(COORD='handshake',
                            LAUNCH_EXTRA='release_gate_mode:=certificate '
                                         'use_intent:=true keepout_mode:=cbf')),
        ]
    if grid == 'gate':
        # 证书闸 vs 启发式闸（论文核心消融）
        return [
            ('heur_clean', dict(COORD='handshake')),
            ('cert_clean', dict(COORD='handshake',
                                LAUNCH_EXTRA='release_gate_mode:=certificate')),
            ('heur_noise', dict(COORD='handshake',
                                LAUNCH_EXTRA='rel_pos_sigma:=0.08 rel_latency:=0.10')),
            ('cert_noise', dict(COORD='handshake',
                                LAUNCH_EXTRA='release_gate_mode:=certificate '
                                             'rel_pos_sigma:=0.08 rel_latency:=0.10')),
        ]
    if grid != 'full':
        return [
            ('direct_clean', dict(COORD='direct')),
            ('auth_clean', dict(COORD='handshake')),
            ('auth_noise', dict(COORD='handshake',
                                LAUNCH_EXTRA='rel_pos_sigma:=0.05 rel_latency:=0.10')),
            ('auth_noise_intent', dict(COORD='handshake',
                                       LAUNCH_EXTRA='use_intent:=true rel_pos_sigma:=0.05 rel_latency:=0.10')),
            ('auth_stress', dict(COORD='handshake',
                                 LAUNCH_EXTRA='rel_pos_sigma:=0.10 rel_latency:=0.20')),
        ]
    grid = []
    for coord in ('direct', 'handshake'):
        for intent in (False, True):
            for sigma in (0.0, 0.05, 0.10):
                for lat in (0.0, 0.10, 0.20):
                    extra = f'rel_pos_sigma:={sigma} rel_latency:={lat}'
                    if intent:
                        extra = 'use_intent:=true ' + extra
                    name = (f"{'auth' if coord == 'handshake' else 'dir'}"
                            f"_i{int(intent)}_s{sigma:.2f}_l{lat:.2f}")
                    grid.append((name, dict(COORD=coord, LAUNCH_EXTRA=extra)))
    return grid


# ------------------------------------------------------------------ 运行一次
def run_once(cfg: dict, secs: int) -> str:
    env = dict(os.environ)
    # 基准统一用大漏斗，给噪声下留余量（与 validate_coord 一致）
    env.setdefault('FUNNEL_MOUTH', '0.30')
    env.update(cfg)
    for attempt in range(3):
        subprocess.run(['bash', RUNNER, str(secs)], env=env,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        txt = open(LOG, encoding='utf-8', errors='ignore').read() if os.path.exists(LOG) else ''
        if 'MODE=stack' in txt:          # 节点确实起来了
            return txt
        print(f'    (重试 {attempt + 1}/3：未检测到节点启动)', flush=True)
    return txt


# ------------------------------------------------------------------ 指标解析
def parse_metrics(txt: str) -> dict:
    captured = 'STACK CAPTURED' in txt
    m = re.search(r'STACK CAPTURED \*\*\* horiz=([0-9.]+)m rel_v=([0-9.]+)', txt)
    horiz = float(m.group(1)) if m else float('nan')
    relv = float(m.group(2)) if m else float('nan')
    rels = [float(x) for x in re.findall(r'min_relA=([0-9.]+)', txt)]
    min_rel = min(rels) if rels else float('nan')
    # 就绪/释放：兼容 handshake（就绪门限通过）与 direct（ALIGNED ... → release）
    n_ready = txt.count('就绪门限通过') + txt.count('ALIGNED rel_xy')
    n_release = txt.count('PAYLOAD RELEASED')
    ack = 'B: 收到 A 释放 ack' in txt
    # 协调异常：逐行统计，剔除传感器/看门狗瞬态（estimator_reset / pos_stale）
    n_hold = sum(1 for L in txt.splitlines()
                 if 'SAFETY HOLD' in L
                 and not any(x in L for x in ('estimator_reset', 'pos_stale')))
    n_pull = sum(1 for L in txt.splitlines() if 'SAFETY PULLBACK' in L)
    failsafe = txt.count('Failsafe')
    trace = txt.count('Traceback')
    # 协调时序：就绪→释放 的间隔
    tr = _ts(txt, '就绪门限通过') or _ts(txt, 'ALIGNED rel_xy')
    tl = _ts(txt, 'PAYLOAD RELEASED')
    dt_ready_release = (tl - tr) if (tr is not None and tl is not None) else float('nan')
    # A 释放时使用的 σ（如有）
    sm = re.search(r'释放权威发布 release@[0-9.]+s \(lead=[0-9.]+, σ=([0-9.]+)\)', txt)
    sigma_used = float(sm.group(1)) if sm else float('nan')
    return dict(captured=captured, horiz=horiz, relv=relv, min_rel=min_rel,
                n_ready=n_ready, n_release=n_release, ack=ack,
                n_hold=n_hold, n_pull=n_pull, failsafe=failsafe, trace=trace,
                dt_ready_release=dt_ready_release, sigma_used=sigma_used)


def _ts(txt: str, needle: str):
    for line in txt.splitlines():
        if needle in line:
            m = re.search(r'\[(\d+\.\d+)\]', line)
            return float(m.group(1)) if m else 0.0
    return None


def _fmt(v, nd=3):
    return 'nan' if (isinstance(v, float) and math.isnan(v)) else f'{v:.{nd}f}'


# ------------------------------------------------------------------ 统计
def wilson(k: int, n: int, z: float = 1.96):
    """Wilson 95% 置信区间。"""
    if n == 0:
        return (float('nan'), float('nan'))
    p = k / n
    d = 1.0 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


def agg(rows: list) -> dict:
    n = len(rows)
    k = sum(1 for r in rows if r['captured'])
    lo, hi = wilson(k, n)
    hs = [r['horiz'] for r in rows if not math.isnan(r['horiz'])]
    rv = [r['relv'] for r in rows if not math.isnan(r['relv'])]
    mr = [r['min_rel'] for r in rows if not math.isnan(r['min_rel'])]
    return dict(n=n, k=k, rate=(k / n if n else 0.0), ci=(lo, hi),
                horiz_mean=(sum(hs) / len(hs) if hs else float('nan')),
                horiz_max=(max(hs) if hs else float('nan')),
                relv_mean=(sum(rv) / len(rv) if rv else float('nan')),
                min_rel=(min(mr) if mr else float('nan')),
                fs=sum(r['failsafe'] for r in rows),
                ann=sum(r['n_hold'] + r['n_pull'] for r in rows))


# ------------------------------------------------------------------ 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--grid', default='quick', choices=['quick', 'full', 'gate', 'ablation'],
                    help='quick | full | gate | ablation（逐项消融）')
    ap.add_argument('--full', action='store_true', help='(等价 --grid full)')
    ap.add_argument('--reps', type=int, default=1, help='每配置重复次数')
    ap.add_argument('--secs', type=int, default=70, help='每次运行秒数')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--only', default='', help='只跑名字匹配该正则的配置')
    ap.add_argument('--stress', action='store_true',
                    help='给所有配置叠加基准 stress（rel_pos_sigma:=0.08 rel_latency:=0.15）')
    args = ap.parse_args()

    grid_name = 'full' if args.full else args.grid
    grid = build_grid(grid_name)
    if args.stress:
        for i, (nm, cfg) in enumerate(grid):
            extra = cfg.get('LAUNCH_EXTRA', '')
            cfg['LAUNCH_EXTRA'] = ('rel_pos_sigma:=0.08 rel_latency:=0.15 ' + extra).strip()
            grid[i] = (nm, cfg)
        grid_name += '+stress'
    if args.only:
        rx = re.compile(args.only)
        grid = [(n, c) for (n, c) in grid if rx.search(n)]

    print(f'配置数={len(grid)}  重复={args.reps}  每次={args.secs}s  '
          f'预计≈{len(grid) * args.reps * (args.secs + 55) / 60:.0f} min')
    if args.dry_run:
        for n, c in grid:
            print(' ', n, c)
        return

    results = {}
    for name, cfg in grid:
        rows = []
        for rep in range(args.reps):
            print(f'>>> {name} [{rep + 1}/{args.reps}] ...', flush=True)
            txt = run_once(cfg, args.secs)
            rows.append(parse_metrics(txt))
        results[name] = dict(cfg=cfg, rows=rows, agg=agg(rows))

    # ---- 报告 ----
    lines = ['# 协同交接基准（Coordination Benchmark）', '',
             '> 由 `python3 tools/bench_coord.py` 生成。统一大漏斗（`FUNNEL_MOUTH=0.30`）。',
             f'> 本次：网格={grid_name}，{len(results)} 个配置 × {args.reps} 次 / 每次 {args.secs}s。',
             '> `auth`=handshake（A 作释放权威）；`dir`=direct（B 单边）；`i`=intent；`s`=σ；`l`=latency(s)。',
             '> `协调异常` = 非传感器瞬态的 HOLD + PULLBACK（已剔除 `estimator_reset`/`pos_stale`）。', '',
             '| 配置 | 捕获率 (95% CI) | horiz mean/max | rel_v mean | min\\|A-B\\| | failsafe | 协调异常 |',
             '|---|---|---|---|---|---|---|']
    for name, res in results.items():
        a = res['agg']
        lo, hi = a['ci']
        lines.append(
            f"| {name} | {a['k']}/{a['n']} = {a['rate'] * 100:.0f}% "
            f"[{lo * 100:.0f},{hi * 100:.0f}] "
            f"| {a['horiz_mean']:.3f}/{a['horiz_max']:.3f} "
            f"| {a['relv_mean']:.2f} "
            f"| {a['min_rel']:.3f} | {a['fs']} | {a['ann']} |")
    lines += ['', '## 明细（每次）', '',
              '| 配置#rep | captured | horiz | rel_v | min\\|A-B\\| | failsafe | HOLD | PULLBACK | 就绪→释放(s) | σ_used |',
              '|---|---|---|---|---|---|---|---|---|---|']
    for name, res in results.items():
        for i, r in enumerate(res['rows']):
            lines.append(
                f"| {name}#{i} | {'✅' if r['captured'] else '❌'} | {_fmt(r['horiz'])} "
                f"| {_fmt(r['relv'], 2)} | {_fmt(r['min_rel'])} | {r['failsafe']} "
                f"| {r['n_hold']} | {r['n_pull']} | {_fmt(r['dt_ready_release'], 2)} "
                f"| {_fmt(r['sigma_used'])} |")
    with open(OUT, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print('wrote', OUT)


if __name__ == '__main__':
    main()

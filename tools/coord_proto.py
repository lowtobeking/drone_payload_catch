#!/usr/bin/env python3
"""coord_proto.py —— 交接协议的时序误差界与鲁棒性（研究 W3 / C2）。

把 handshake + commit/abort + 时钟同步 从"经验常数"升级为**有界设计**：

关键量（单向延迟上界 d_max、抖动 j、时钟误差 ε_clock、丢包 p）：
  · 释放执行误差  δ_t = d_{A→payload} + j            （要求 release_lead ≥ δ_t）
  · 取消窗口      W  = release_lead − d_max           （只有在此窗口内的失败可被 abort 拦截）
  · B 等待        handshake_timeout ≥ d_{A→B} + j
  · 就绪新鲜度    ready_timeout ≥ d_{B→A}
  · 时序一致性    |t_rel^B − t_rel^A| ≤ ε_clock + j

本工具给出：取消捕获率、设计表（给定 d_max 需要多大 release_lead）、丢包下的协议结局概率。

用法：
  python3 tools/coord_proto.py [--n N] [--seed S]
"""
from __future__ import annotations

import argparse
import numpy as np


def catch_rate(n, lead, d_max, p_loss, rng):
    """提交后，gate 在 lead 窗口内随机时刻失败时，abort 能赶在释放前到达的比例。"""
    d = rng.uniform(0.0, d_max, n)              # 单向延迟 A→payload
    t_fail = rng.uniform(0.0, lead, n)          # 失败发生时刻（相对提交）
    dropped = rng.random(n) < p_loss            # abort 丢包
    caught = (t_fail + d <= lead) & (~dropped)
    return float(caught.mean())


def design_lead(d_max, target, p_loss=0.0):
    """给定 d_max 达到取消捕获率 target 所需的 release_lead（数值反解）。"""
    if d_max <= 0.0:
        return 0.0
    lo, hi = d_max, d_max * 100.0
    rng = np.random.default_rng(0)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        c = catch_rate(200000, mid, d_max, p_loss, rng)
        if c < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def outcomes(n, p_ready, p_cmd, p_at, p_abort, rng):
    """丢包下的协议结局（独立丢包）。返回各类概率。

    消息：ready(B→A) / release_cmd(A→B) / release_at(A→payload) / abort(A→payload)
    """
    ready = rng.random(n) >= p_ready
    cmd = rng.random(n) >= p_cmd
    at = rng.random(n) >= p_at
    abort = rng.random(n) >= p_abort
    # 结局
    no_release = (~ready) | (~at)                       # 未释放（安全，任务中止）
    released = at                                       # 载荷按 t_rel 释放
    b_dive = cmd                                        # B 收到 cmd 才下潜
    lost_cmd = released & (~b_dive)                     # 释放了但 B 没下潜 → 捕获失败
    normal = released & b_dive
    return dict(no_release=float(no_release.mean()),
                released=float(released.mean()),
                lost_cmd=float(lost_cmd.mean()),
                normal=float(normal.mean()),
                abort_lost=float((~abort).mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=200000)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    N = args.n

    print('=' * 84)
    print('C2 协议时序界：取消窗口 W = release_lead − d_max')
    print('=' * 84)
    print(f"{'release_lead':>12} {'d_max':>6} {'W':>7} {'catch(均匀失败,无丢包)':>22}")
    for lead in (0.10, 0.20, 0.50, 1.00):
        for d_max in (0.05, 0.10, 0.20, 0.40):
            c = catch_rate(N, lead, d_max, 0.0, rng)
            print(f"{lead:>12.2f} {d_max:>6.2f} {max(0.0, lead - d_max):>7.2f} {c:>22.3f}")

    print('\n' + '=' * 84)
    print('设计表：达到目标取消捕获率所需的最小 release_lead（各 d_max）')
    print('=' * 84)
    print(f"{'d_max':>6} | {'lead@0.90':>10} {'lead@0.95':>10} {'lead@0.99':>10} "
          f"| {'现行 0.20 可容忍 d_max(0.9)':>24}")
    for d_max in (0.02, 0.05, 0.10, 0.20):
        l90 = design_lead(d_max, 0.90)
        l95 = design_lead(d_max, 0.95)
        l99 = design_lead(d_max, 0.99)
        # 现行 lead=0.20 下，求解使 catch=0.9 的 d_max
        lo, hi = 0.001, 0.20
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if catch_rate(N, 0.20, mid, 0.0, rng) >= 0.90:
                lo = mid
            else:
                hi = mid
        print(f"{d_max:>6.2f} | {l90:>10.3f} {l95:>10.3f} {l99:>10.3f} | {lo:>24.3f}")

    print('\n' + '=' * 84)
    print('丢包下的协议结局（release_lead=0.2; 各消息独立丢包率 p）')
    print('=' * 84)
    print(f"{'p':>6} | {'未释放(安全)':>12} {'已释放':>8} {'丢cmd(B未下潜)':>15} "
          f"{'正常':>8} {'abort丢失':>10}")
    for p in (0.0, 0.05, 0.10, 0.20):
        o = outcomes(N, p, p, p, p, rng)
        print(f"{p:>6.2f} | {o['no_release']:>12.3f} {o['released']:>8.3f} "
              f"{o['lost_cmd']:>15.4f} {o['normal']:>8.3f} {o['abort_lost']:>10.3f}")

    print('\n' + '=' * 84)
    print('与现行常数对照（launch 默认）')
    print('=' * 84)
    defaults = {'release_lead': 0.20, 'commit_hold_s': 0.20,
                'handshake_timeout': 1.0, 'ready_timeout': 0.5}
    for k, v in defaults.items():
        print(f"  {k:<20} = {v}")
    print("  可行条件：release_lead > d_max；handshake_timeout ≥ d_max+j；ready_timeout ≥ d_max")
    print("  ⇒ 现行 release_lead=0.20 只对 单向延迟 d_max<0.20s 有正的取消窗口。")


if __name__ == '__main__':
    main()

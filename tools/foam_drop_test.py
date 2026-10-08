#!/usr/bin/env python3
"""foam_drop_test.py —— 真机【泡棉恢复系数 e】实测工具（托盘末端选型用）。

物理：物体从高度 H 落到缓冲面，回弹高度 h，则恢复系数 e = √(h / H)
      （回弹高度 h = e²·H）。托盘保持条件：h ≤ 有效围挡高度，即 e²·gap ≤ h_rim。

真机测量步骤（见 `report/m6_tray_robustness.md` §7）：
  1. 把泡棉铺进托盘（与实机一致），水平放置；
  2. 让 **6cm/100g 实物方块**从已知高度 H（如 1.0m）自由落到泡棉中心；
  3. 用**手机慢动作/高速相机**拍侧视，或贴标尺读**回弹最高点** h；
  4. 重复 5–10 次（不同落点/释放姿态），记录 (H, h)；
  5. 用本工具算 e（均值/离散），拿到推荐 `TRAY_E`，并换算 `v_retain` / 允许的最大 gap。

判据：**目标 e ≤ 0.20**（回弹高度 ≤ 0.04·H）；e 太大 → 换更软/更厚泡棉，或加深围边。

用法:
  # 统一落高 H=1.0m，回弹列表
  python3 tools/foam_drop_test.py --h-drop 1.0 --rebounds 0.020 0.024 0.018 0.022
  # 每对 (H:h)
  python3 tools/foam_drop_test.py --pairs "1.0:0.020,1.0:0.024,1.2:0.030"
  # 指定围边高，输出 v_retain / 允许 gap 及 SITL 命令
  python3 tools/foam_drop_test.py --h-drop 1.0 --rebounds 0.02 0.024 --rim 0.05 --gap 1.0
"""
from __future__ import annotations

import argparse
import math
import statistics as st

G = 9.81


def analyze(pairs, rim=0.05, gap=1.0, k_cons=1.0):
    es = []
    for H, h in pairs:
        if H <= 0 or h < 0:
            continue
        es.append(math.sqrt(min(h / H, 1.0)))
    e_mean = st.mean(es)
    e_std = st.pstdev(es) if len(es) > 1 else 0.0
    e_max = max(es)
    e_cons = min(0.999, e_mean + k_cons * e_std)     # 保守：均值 + kσ
    return dict(es=es, e_mean=e_mean, e_std=e_std, e_max=e_max, e_cons=e_cons,
                rim=rim, gap=gap)


def report(a):
    n = len(a['es'])
    print('== 泡棉恢复系数 e 实测分析 ==')
    print(f'  样本数 n={n}')
    print(f'  各次 e = ' + '  '.join(f'{x:.3f}' for x in a['es']))
    print(f'  e: mean={a["e_mean"]:.3f}  std={a["e_std"]:.3f}  max={a["e_max"]:.3f}')
    print(f'  推荐 TRAY_E（保守 mean+σ）= {a["e_cons"]:.3f} → 取 {round(a["e_cons"],2):.2f}')
    print()
    e = a['e_cons']
    rim, gap = a['rim'], a['gap']
    v_retain = math.sqrt(2 * G * rim) / max(e, 1e-6)
    rebound_at_gap = e * e * gap
    max_gap = rim / (e * e)
    print(f'  设围边高 h={rim:.3f}m：')
    print(f'    v_retain = √(2gh)/e = {v_retain:.2f} m/s')
    print(f'    在 gap={gap:.2f}m 下的回弹高度 = e²·gap = {rebound_at_gap*100:.1f} cm'
          f'  → {"PASS(≤围边)" if rebound_at_gap <= rim else "FAIL(弹飞)"}')
    print(f'    免弹出最大 gap = h/e² = {max_gap:.2f} m')
    verdict = 'PASS（低回弹）' if e <= 0.20 else ('边缘' if e <= 0.30 else 'FAIL（换更软/更厚泡棉）')
    print(f'  判定：e={e:.3f} → {verdict}（目标 ≤0.20）')
    print()
    print('  SITL 复现（用实测 e）：')
    print(f'    FUNNEL_TYPE=tray TRAY_E={round(e,2):.2f} TRAY_RIM={rim} '
          f'FUNNEL_MOUTH=0.15 bash run_m6_sitl.sh 60')


def main():
    ap = argparse.ArgumentParser(description='泡棉恢复系数 e 实测分析')
    ap.add_argument('--h-drop', type=float, default=None, help='统一落高 H (m)')
    ap.add_argument('--rebounds', type=float, nargs='+', default=None, help='回弹高度列表 h (m)')
    ap.add_argument('--pairs', default=None, help='每对 H:h，逗号分隔，如 "1.0:0.02,1.0:0.024"')
    ap.add_argument('--rim', type=float, default=0.05, help='围边高 h (m)')
    ap.add_argument('--gap', type=float, default=1.0, help='预计下落高度 gap (m)')
    ap.add_argument('--k-cons', type=float, default=1.0, help='保守估计的 k（mean+kσ）')
    args = ap.parse_args()

    pairs = []
    if args.pairs:
        for p in args.pairs.split(','):
            H, h = p.split(':')
            pairs.append((float(H), float(h)))
    elif args.h_drop is not None and args.rebounds:
        pairs = [(args.h_drop, h) for h in args.rebounds]
    else:
        # 示例数据（默认可跑）
        print('（未给测量数据，使用示例：H=1.0m，回弹 0.020/0.024/0.018 m）\n')
        pairs = [(1.0, 0.020), (1.0, 0.024), (1.0, 0.018)]

    report(analyze(pairs, rim=args.rim, gap=args.gap, k_cons=args.k_cons))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""tray_sizing.py —— 圆形托盘（塑料围边 + 泡棉缓冲）选型计算器（真机末端）。

真机把末端从"空心漏斗"换成"圆形托盘"后，物理性质变了：
  · 托盘**没有杯深**，纯平盘不靠深度保持，靠【围边挡横向】+【泡棉消反弹】；
  · 恢复系数 e 是生死线：裸塑料盘 e≈0.6–0.8 → 回弹 ~20cm 必飞；软泡棉 e≈0.15 → 回弹 ~1cm。

模型（与 config 的 funnel 判据一致）：
  eff_r      = 盘内半径 − 物块等效半径
  保持判据   = 回弹高度 e²·gap ≤ 有效围挡高度 h（= 围边高 + 泡棉凹垫深）
  v_retain   = √(2·g·h) / e            （等效可承受接触速度）
  接触相对速度 v_rel = √(2·(g − a_dive)·gap)

用法:
  python3 tools/tray_sizing.py                          # 默认：内径30cm/围边5cm/e=0.15/6cm方块/gap1m
  python3 tools/tray_sizing.py --diameter 0.25          # 换 25cm 盘
  python3 tools/tray_sizing.py --e 0.40                 # 看缓冲变差会怎样
  python3 tools/tray_sizing.py --sweep-e                # e → 所需围边高度（选缓冲用）
  python3 tools/tray_sizing.py --measure-drop 1.0 0.22  # 落物试验反推 e（下落1m，回弹22cm）
"""
from __future__ import annotations

import argparse
import math

G = 9.81


def object_radius(obj_size: float, shape: str = 'cube') -> float:
    """物块等效半径：立方体按半宽（面朝下）；--corner 按半对角（角朝下，保守）。"""
    if shape == 'cube':
        return obj_size / 2.0
    if shape == 'cube_corner':
        return obj_size * math.sqrt(2.0) / 2.0
    if shape == 'sphere':
        return obj_size / 2.0
    raise ValueError(f'未知 shape: {shape}')


def analyze(diameter: float, rim: float, dish: float, e: float,
            obj_size: float, shape: str, gap: float, a_dive: float,
            mass: float, mu: float) -> dict:
    R = diameter / 2.0
    r_obj = object_radius(obj_size, shape)
    eff_r = max(0.0, R - r_obj)
    h = rim + dish                                   # 有效围挡高度
    v_rel = math.sqrt(max(0.0, 2.0 * (G - a_dive) * gap))
    rebound = e * e * gap                            # 回弹高度（无下潜时 = e²·gap）
    retain = rebound <= h + 1e-12
    v_retain = math.sqrt(2.0 * G * h) / max(e, 1e-6)
    max_gap_no_bounce = h / (e * e) if e > 1e-9 else float('inf')
    imp = mass * v_rel                               # 冲量 N·s
    energy = 0.5 * mass * v_rel * v_rel              # 动能 J
    return dict(R=R, r_obj=r_obj, eff_r=eff_r, h=h, v_rel=v_rel,
                rebound=rebound, retain=retain, v_retain=v_retain,
                max_gap_no_bounce=max_gap_no_bounce, impulse=imp,
                energy=energy, gap=gap, a_dive=a_dive, e=e, mu=mu, mass=mass)


def _print_report(a: dict) -> None:
    ok = 'PASS ✅' if a['retain'] else 'FAIL ❌（会弹出）'
    print('== 圆形托盘选型 ==')
    print(f"  落点半径 eff_r      = {a['eff_r']*100:6.1f} cm  "
          f"(盘半径 {a['R']*100:.1f} − 物半径 {a['r_obj']*100:.1f})")
    print(f"  有效围挡高度 h      = {a['h']*100:6.1f} cm")
    print(f"  接触相对速度 v_rel  = {a['v_rel']:6.2f} m/s  (gap={a['gap']:.2f}, a_dive={a['a_dive']:.1f})")
    print(f"  回弹高度 e²·gap     = {a['rebound']*100:6.1f} cm  (e={a['e']:.2f})")
    print(f"  保持判据 回弹≤h     : {ok}")
    print(f"  等效保持速度 v_retain= {a['v_retain']:6.2f} m/s")
    print(f"  免弹出最大 gap      = {a['max_gap_no_bounce']:6.2f} m  (h/e²)")
    print(f"  B 端冲击            = {a['impulse']:.2f} N·s / {a['energy']:.2f} J "
          f"(m={a['mass']:.2f} kg)")
    print()
    if not a['retain']:
        need = a['rebound']
        print(f"  ⚠️ 缓冲太弹：需把围挡 h 提到 ≥ {need*100:.1f} cm，或把 e 降到 "
              f"≤ {math.sqrt(a['h']/a['gap']):.2f}")
    print("  设计规则：h ≥ e²·gap  ⇔  e ≤ √(h/gap)")


def sweep_e(a: dict, gaps=(0.6, 0.8, 1.0)) -> None:
    print('== 缓冲恢复系数 e → 所需围挡高度 h=e²·gap (cm) ==')
    print(f"{'e':>5} | " + ' | '.join(f'gap={g:.1f}m' for g in gaps))
    for e in (0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.60, 0.70):
        cells = [f"{e*e*g*100:8.1f}" for g in gaps]
        print(f"{e:5.2f} | " + ' | '.join(cells))
    print('\n  （软泡棉 e≈0.15 → gap1m 只需 2.3cm 围挡；裸塑料 e≈0.7 → 需 49cm，不可能）')


def measure_e(h_drop: float, h_rebound: float) -> None:
    e = math.sqrt(max(0.0, h_rebound / h_drop))
    print('== 落物试验反推恢复系数 ==')
    print(f'  下落高度 H={h_drop:.3f} m，回弹高度 h={h_rebound:.3f} m')
    print(f'  e = √(h/H) = {e:.3f}')
    verdict = 'PASS（低回弹）' if e <= 0.20 else ('边缘' if e <= 0.30 else 'FAIL（太弹，换更软泡棉）')
    print(f'  目标 e ≤ 0.20 → {verdict}')


def main() -> None:
    ap = argparse.ArgumentParser(description='圆形托盘选型计算器')
    ap.add_argument('--diameter', type=float, default=0.30, help='盘内径 (m)')
    ap.add_argument('--rim', type=float, default=0.05, help='围边高度 (m)')
    ap.add_argument('--dish', type=float, default=0.0, help='泡棉凹垫深 (m)，加到围挡高度')
    ap.add_argument('--e', type=float, default=0.15, help='泡棉恢复系数')
    ap.add_argument('--obj', type=float, default=0.06, help='物块尺寸 (m)')
    ap.add_argument('--shape', default='cube',
                    choices=['cube', 'cube_corner', 'sphere'], help='物块形状')
    ap.add_argument('--gap', type=float, default=1.0, help='下落高度 (m)')
    ap.add_argument('--a-dive', type=float, default=0.0, help='B 下潜加速度 (m/s²)')
    ap.add_argument('--mass', type=float, default=0.10, help='载荷质量 (kg)')
    ap.add_argument('--mu', type=float, default=0.9, help='盘面摩擦系数（信息用）')
    ap.add_argument('--corner', action='store_true', help='按立方体半对角(角朝下)保守')
    ap.add_argument('--sweep-e', action='store_true', help='打印 e→所需围挡高度表')
    ap.add_argument('--measure-drop', nargs=2, type=float, metavar=('H_DROP', 'H_REBOUND'),
                    help='落物试验反推 e')
    args = ap.parse_args()

    if args.measure_drop:
        measure_e(args.measure_drop[0], args.measure_drop[1])
        return

    shape = 'cube_corner' if args.corner else args.shape
    a = analyze(args.diameter, args.rim, args.dish, args.e,
                args.obj, shape, args.gap, args.a_dive, args.mass, args.mu)
    _print_report(a)
    if args.sweep_e:
        print()
        sweep_e(a)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""funnel_model.py —— 末端机构的解析模型：平顶盘 vs 空心导向锥 vs 主动保持。

对比三种末端机构在 M6 垂直捕获中的能力：
  · 平顶盘（当前 x500_funnel）：载荷落在平盘上，靠摩擦+低恢复系数；
  · 空心导向锥：内壁把偏心的载荷导向中心，并阻止其横向逃逸；
  · 主动保持（低回弹锁扣/磁吸/夹爪）：一旦进入即锁定，绕过 v_retain 上限。

度量的能力：
  eff_r      —— 口内有效半径（可容忍的落点水平偏差，= 口半径 − 载荷半径）
  v_retain   —— 可承受的**垂直**接触速度（深度/恢复系数决定）
  v_h_retain —— 可承受的**水平**速度（不滑出/不逃逸）
  tilt_max   —— 平台可倾斜的上限（超过则载荷滑出）

用法: python3 tools/funnel_model.py
"""
from __future__ import annotations

import math

G = 9.81


def flat_disk(mouth_r: float, obj_r: float, mu: float = 1.0,
              depth: float = 0.30, e: float = 0.60) -> dict:
    eff_r = mouth_r - obj_r
    v_retain = math.sqrt(2 * G * depth) / max(e, 1e-6)
    # 水平：落点带水平速度 v_h，摩擦减速 μg，滑行 v_h²/(2μg) 后停；
    # 若滑行距离 > eff_r 则滑出盘边 → v_h ≤ sqrt(2μg·eff_r)
    v_h = math.sqrt(2 * mu * G * eff_r)
    tilt = math.degrees(math.atan(mu))     # 倾角超过 atan(μ) 则静止也不留
    return dict(name='平顶盘', eff_r=eff_r, v_retain=v_retain, v_h_retain=v_h,
                tilt_max=tilt, note='靠摩擦+低回弹"砸住"')


def hollow_cone(mouth_r: float, obj_r: float, depth: float, half_angle_deg: float,
                mu: float = 1.0, e: float = 0.60) -> dict:
    th = math.radians(half_angle_deg)
    eff_r = mouth_r - obj_r                        # 进入口内即可（同平盘）
    v_retain = math.sqrt(2 * G * depth) / max(e, 1e-6)
    # 水平：内壁挡住；能被兜住只要载荷爬不出锥（爬上 depth 需能量）→ v_h ≤ sqrt(2gd/e)
    v_h = math.sqrt(2 * G * depth / max(e, 1e-6))
    bottom_r = max(0.0, mouth_r - depth * math.tan(th))
    # 倾角：锥壁在斜面角内仍盛得住（近似 90°−半角 为极限）
    tilt = max(0.0, 90.0 - half_angle_deg)
    return dict(name=f'空心锥(半角{half_angle_deg:.0f}°)', eff_r=eff_r,
                v_retain=v_retain, v_h_retain=v_h, tilt_max=tilt,
                note=f'底半径≈{bottom_r:.3f}m，内壁导向+阻止逃逸')


def active_retain(mouth_r: float, obj_r: float, v_struct: float = 8.0) -> dict:
    eff_r = mouth_r - obj_r
    return dict(name='主动保持(锁扣/磁吸)', eff_r=eff_r, v_retain=v_struct,
                v_h_retain=v_struct, tilt_max=180.0, note='进入即锁定，绕过 v_retain')


def no_dive_gap(v_retain: float, g: float = G) -> float:
    """不依赖下潜（a_dive=0）时，可承受的最大下落高度：v_rel=√(2g·gap)≤v_retain。"""
    return v_retain * v_retain / (2 * g)


def main():
    mouth_r, obj_r, depth, e = 0.30, 0.05, 0.30, 0.60
    print(f"== 末端机构能力对比（口半径 {mouth_r}m, 载荷半径 {obj_r}m, 深度 {depth}m, e={e}）==")
    print(f"无下潜最大下落高度 gap = v_retain²/(2g)：")
    models = [
        flat_disk(mouth_r, obj_r, depth=depth, e=e),
        hollow_cone(mouth_r, obj_r, depth, 30, e=e),
        hollow_cone(mouth_r, obj_r, depth, 45, e=e),
        active_retain(mouth_r, obj_r, v_struct=8.0),
    ]
    print(f"{'机构':>18} {'eff_r':>6} {'v_retain':>9} {'v_h_retain':>10} "
          f"{'tilt_max':>9} {'无下潜gap':>10}")
    for m in models:
        print(f"{m['name']:>18} {m['eff_r']:6.2f} {m['v_retain']:9.2f} "
              f"{m['v_h_retain']:10.2f} {m['tilt_max']:8.0f}° "
              f"{no_dive_gap(m['v_retain']):10.3f}")
        print(f"{'':>18} {m['note']}")
    print()
    print("解读：")
    print(" · eff_r 由【口半径】决定——三种机构相同。空心锥/主动保持不增大落点半径。")
    print(" · 平顶盘的短板在【水平速度/倾角】：靠摩擦，滑出半径外即丢失。")
    print(" · 空心锥把水平容差 ≈ 从 √(2μg·eff_r) 提到 √(2gd/e)、倾角上限抬高。")
    print(" · 主动保持把 v_retain 的恢复系数上限换成【结构强度】，并几乎不怕倾角——")
    print("   这是把'砸住'升级为'导向—夹持—带走'的关键，也是 downdraft 的最终解。")


if __name__ == '__main__':
    main()

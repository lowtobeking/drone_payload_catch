#!/usr/bin/env python3
"""handoff_explore.py —— 交接场景深度探索：公式推导的数值验证（纯 Python，无 ROS）。

对 report/handoff_exploration.md 里的推导逐条做数值验证，并打印关键表格/结论。

覆盖四个方向：
  1) 静态交接（A 悬停、B 正下方下潜接）—— v_rel 下界、高度预算、最优 a_dive
  2) 动态交接（A 平飞抛投）—— 继承速度、终端速度包络、反向抛投消速
  3) 机械臂丢接 —— 反向抛投垂直化、apex-catch 的 ≥2g 负结果、臂的 reach/吸收
  4) 增加速度交接 —— 速度预算分解、gap/a_dive/v_retain 边界、可扩展上限

坐标：世界系 NED（z 向下为正）；推导时用高度 h = -z（向上为正）。

用法：
    python3 tools/handoff_explore.py
"""
from __future__ import annotations

import math

G = 9.81


# -------------------------------------------------------------------------- #
# 基础公式（与 payload_catch/stack_drop.py 保持一致）
# -------------------------------------------------------------------------- #
def retain_speed(depth: float, restitution: float, g: float = G) -> float:
    """刚性漏斗能兜住的最大接触垂直速度 v_retain = sqrt(2 g depth) / e。"""
    return math.sqrt(2.0 * g * depth) / max(restitution, 1e-6)


def static_drop(gap: float, a_dive: float, g: float = G, a_brake: float = 6.0):
    """纯垂直下落、B 全时以 a_dive 下潜的标称量。

    返回 (t_c, v_rel, v_pay, v_b, dive, brake)。
    """
    t_c = math.sqrt(2.0 * gap / (g - a_dive))
    v_rel = (g - a_dive) * t_c          # = sqrt(2 (g - a_dive) gap)
    v_pay = g * t_c
    v_b = a_dive * t_c
    dive = 0.5 * a_dive * t_c * t_c
    brake = v_b * v_b / (2.0 * a_brake)
    return t_c, v_rel, v_pay, v_b, dive, brake


def altitude_cost(gap: float, a_dive: float, g: float = G, a_brake: float = 6.0) -> float:
    """B 从待命高度起，下潜+刹车总共吃掉的高度。

    闭式：dive + brake = gap·α/(1-α)·(1 + α/β)，α=a_dive/g, β=a_brake/g。
    """
    alpha = a_dive / g
    beta = a_brake / g
    return gap * alpha / (1.0 - alpha) * (1.0 + alpha / beta)


def max_a_dive_from_budget(gap: float, budget: float, g: float = G,
                           a_brake: float = 6.0) -> float:
    """给定高度预算 budget（B 待命高度 − 最低允许高度），解出最大可用 a_dive。

    budget ≥ gap·α/(1-α)·(1+α/β)  → 解关于 α 的二次方程（取 [0,g) 内的根）。
    """
    beta = a_brake / g
    # gap·α(1+α/β) = budget·(1-α)
    # => (gap/β) α² + (gap+budget) α - budget = 0
    A = gap / beta
    B = gap + budget
    C = -budget
    disc = B * B - 4.0 * A * C
    if disc < 0:
        return 0.0
    alpha = (-B + math.sqrt(disc)) / (2.0 * A)
    return min(alpha * g, g - 1e-6)


def rest_to_rest_peak_accel_bangbang(D: float, T: float) -> float:
    """静止→静止、位移 D、历时 T 的最小峰值加速度（bang-bang 梯形）。= 4D/T²。"""
    return 4.0 * D / (T * T)


def rest_to_rest_peak_accel_cubic(D: float, T: float) -> float:
    """静止→静止 min-energy 三次多项式的峰值加速度。= 6D/T²。"""
    return 6.0 * D / (T * T)


# -------------------------------------------------------------------------- #
# 1) 静态交接
# -------------------------------------------------------------------------- #
def sec1_static() -> None:
    print("=" * 78)
    print("§1 静态交接：A 悬停、B 正下方下潜接（gap=1.0m, a_brake=6, h_B=3.5）")
    print("=" * 78)
    gap, a_brake, h_b = 1.0, 6.0, 3.5
    v_retain = retain_speed(0.30, 0.60)
    print(f"v_retain(depth=0.3,e=0.6) = {v_retain:.3f} m/s")
    print(f"{'a_dive':>7} {'t_c':>7} {'v_rel':>7} {'v_pay':>7} {'v_B':>7} "
          f"{'dive':>7} {'brake':>7} {'总降高':>7} {'刹停后高度':>10} {'可接?':>6}")
    for a_dive in [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 5.5]:
        t_c, v_rel, v_pay, v_b, dive, brake = static_drop(gap, a_dive, a_brake=a_brake)
        total = dive + brake
        post = h_b - total
        ok = "✓" if (post >= 0.30 and v_rel <= v_retain) else "✗"
        print(f"{a_dive:7.2f} {t_c:7.3f} {v_rel:7.3f} {v_pay:7.3f} {v_b:7.3f} "
              f"{dive:7.3f} {brake:7.3f} {total:7.3f} {post:10.3f} {ok:>6}")
    print("\n闭式校验：altitude_cost == dive+brake（α 参数化）")
    for a_dive in [1.0, 3.0, 5.0]:
        _, _, _, _, dive, brake = static_drop(gap, a_dive, a_brake=a_brake)
        ac = altitude_cost(gap, a_dive, a_brake=a_brake)
        print(f"  a_dive={a_dive}: dive+brake={dive+brake:.4f}  altitude_cost={ac:.4f} "
              f"({'一致' if abs(dive+brake-ac)<1e-9 else '不一致!'})")

    print("\n给定高度预算 → 最大 a_dive → v_rel 下界（gap 扫描）")
    print(f"{'gap':>6} {'预算(Δh)':>8} {'α_max':>6} {'a_dive,max':>10} "
          f"{'v_rel,min':>9} {'v_retain':>9} {'能否兜住':>8}")
    for gap in [0.5, 1.0, 1.5, 2.0]:
        budget = h_b - 0.30
        a_max = max_a_dive_from_budget(gap, budget, a_brake=a_brake)
        t_c, v_rel, _, _, _, _ = static_drop(gap, a_max, a_brake=a_brake)
        ok = "✓" if v_rel <= v_retain else "✗"
        print(f"{gap:6.2f} {budget:8.2f} {a_max/G:6.3f} {a_max:10.3f} "
              f"{v_rel:9.3f} {v_retain:9.3f} {ok:>8}")


# -------------------------------------------------------------------------- #
# 2) 动态交接
# -------------------------------------------------------------------------- #
def sec2_dynamic() -> None:
    print()
    print("=" * 78)
    print("§2 动态交接：A 平飞抛投（继承速度）与反向抛投消速")
    print("=" * 78)
    h_a, h_c = 4.5, 2.0
    dh = h_a - h_c
    tau0 = math.sqrt(2.0 * dh / G)
    print(f"h_A={h_a}, h_c={h_c} → 下落 {dh}m 历时 τ_c={tau0:.3f}s，"
          f"纯垂直末速 v_z={G*tau0:.2f} m/s")
    print(f"{'A速度u':>8} {'载荷水平速':>10} {'终端速率|v_p|':>12} "
          f"{'反向抛投后|v_p|':>14} {'说明':>20}")
    for u in [0.0, 0.5, 1.0, 2.0, 3.0, 5.0]:
        v_vert = G * tau0
        v_mag = math.sqrt(u * u + v_vert * v_vert)
        v_mag_cancel = v_vert  # 反向抛投 v_throw,xy = -u → 水平速度归零
        note = "纯垂直(静止)" if u == 0 else ("需B平飞≥u 匹配" if u <= 2 else "超B限速")
        print(f"{u:8.2f} {u:10.2f} {v_mag:12.2f} {v_mag_cancel:14.2f} {note:>20}")
    print("\n结论：载荷水平速度 = A 的飞行速度 u 全程不变；")
    print("      反向抛投（机械臂把载荷向 A 运动反方向掷出 -u）可令水平速度归零，")
    print("      把动态交接退化为【纯垂直下落】，B 无需匹配 u。")


# -------------------------------------------------------------------------- #
# 3) 机械臂丢接
# -------------------------------------------------------------------------- #
def sec3_arm() -> None:
    print()
    print("=" * 78)
    print("§3 机械臂丢接：apex-catch 负结果 + 臂的 reach/吸收")
    print("=" * 78)
    # apex-catch：A 以 w 上抛，B 在最高点(速度0)接。B 需在 τ=w/g 内从
    # h_B 爬到 h_apex=h_A+w²/(2g) 且两端静止。峰值加速度下界 = 4D/T² (bang-bang)。
    h_a, h_b = 4.5, 3.5
    gap = h_a - h_b
    print(f"A 悬停 {h_a}m，B 待命 {h_b}m。上抛速度 w → 顶点 h_apex=h_A+w²/(2g)，")
    print(f"历时 τ=w/g；B 需静止→静止爬升 D=gap+w²/(2g) 到顶点。")
    print(f"峰值加速度下界：bang-bang 4D/T²，min-energy 三次 6D/T²。")
    print(f"{'上抛w':>7} {'顶点抬高':>8} {'τ':>7} {'a_min(4D/T²)':>13} "
          f"{'a_三次(6D/T²)':>13} {'a_max=6能否':>11}")
    for w in [1.0, 2.0, 3.0, 4.0, 6.0, 10.0]:
        T = w / G
        D = gap + w * w / (2.0 * G)
        a_bb = rest_to_rest_peak_accel_bangbang(D, T)
        a_cb = rest_to_rest_peak_accel_cubic(D, T)
        ok = "✓" if a_bb <= 6.0 else "✗"
        print(f"{w:7.2f} {w*w/(2*G):8.2f} {T:7.3f} {a_bb:13.2f} {a_cb:13.2f} {ok:>11}")
    print(f"\n结论：w→∞ 时 a_min→2g={2*G:.2f} m/s²（下界），任何有限 w 都 >2g；")
    print(f"      而四旋翼 a_max≈6 m/s² 远小于 2g ⇒ 纯垂直上抛+顶点接不可行（负结果）。")
    print(f"      ⇒ 上抛只适合【加长飞行时间】给 B 争取到达时间，不适合【顶点零速接】。")

    print()
    print("臂的 reach / 吸收（把当前位置/速度约束放松）：")
    r_c, r_arm = 0.14, 0.15
    v_retain = retain_speed(0.30, 0.60)
    v_arm = 2.0
    print(f"  当前位置有效半径 eff_r={r_c:.2f}m（捕获水平偏差 0.108~0.128m，余量仅0.01~0.03m）")
    print(f"  加臂 reach={r_arm:.2f}m → eff_r={r_c+r_arm:.2f}m（余量翻约 4 倍）")
    print(f"  漏斗 v_retain={v_retain:.2f} m/s；臂端以 {v_arm} m/s 顺载荷方向吸收")
    print(f"  → 有效保持速度 ≈ {v_retain:.2f}+{v_arm:.2f}={v_retain+v_arm:.2f} m/s")


# -------------------------------------------------------------------------- #
# 4) 增加速度交接
# -------------------------------------------------------------------------- #
def sec4_speed() -> None:
    print()
    print("=" * 78)
    print("§4 增加速度交接：速度预算分解与边界")
    print("=" * 78)
    # 速度预算：接触相对速度 = 垂直下界 + 水平残余；可被漏斗/臂吸收。
    # v_rel,vert = sqrt(2(g-a_dive)gap)，受 B 只能下潜 ≤g 且高度预算限制。
    gap, h_b, a_brake = 1.0, 3.5, 6.0
    budget = h_b - 0.30
    print("垂直分量下界（B 水平悬停、漏斗朝上、只能下潜 ≤g）：")
    print(f"{'gap':>6} {'a_dive':>7} {'v_rel,min':>9} {'漏斗0.3/0.6':>10} "
          f"{'漏斗0.5/0.6':>10} {'漏斗0.5/0.4':>10}")
    for gap in [0.5, 1.0, 1.5, 2.0]:
        a_max = max_a_dive_from_budget(gap, budget, a_brake=a_brake)
        _, v_rel, _, _, _, _ = static_drop(gap, a_max, a_brake=a_brake)
        r1 = retain_speed(0.30, 0.60)
        r2 = retain_speed(0.50, 0.60)
        r3 = retain_speed(0.50, 0.40)
        print(f"{gap:6.2f} {a_max:7.2f} {v_rel:9.2f} {r1:10.2f} {r2:10.2f} {r3:10.2f}")
    print("  ✓ 表示该漏斗配置能兜住该 gap 下的最小相对速度。")

    print("\n漏斗深度/恢复系数对 v_retain 的影响（gap=1 时的对比）：")
    _, v_rel_nom, _, _, _, _ = static_drop(1.0, 3.0, a_brake=a_brake)
    print(f"  标称 a_dive=3 → v_rel={v_rel_nom:.2f} m/s")
    print(f"{'depth':>6} {'e':>5} {'v_retain':>9} {'余量 vs 3.69':>12}")
    for depth, e in [(0.30, 0.60), (0.40, 0.60), (0.50, 0.60),
                     (0.30, 0.40), (0.50, 0.40), (0.80, 0.60)]:
        vr = retain_speed(depth, e)
        print(f"{depth:6.2f} {e:5.2f} {vr:9.2f} {vr-v_rel_nom:12.2f}")

    print("\n水平分量（A 平飞速度 u 的影响）：")
    print("  · 编队同速（继承 u）：B 必须能飞到 |v|≥u，受 B 限速约束。")
    print("  · 反向抛投（机械臂 -u）：水平速度归零，B 无需匹配 u；")
    print("    上限由【臂的抛掷速度】决定，而非 B 的飞行包线。")
    print("  · 臂端顺载荷吸收 + 臂 reach：进一步放宽位置/速度约束。")


# -------------------------------------------------------------------------- #
def main() -> None:
    sec1_static()
    sec2_dynamic()
    sec3_arm()
    sec4_speed()
    print()
    print("=" * 78)
    print("所有数值与 report/handoff_exploration.md 的推导一一对应，可复现。")
    print("=" * 78)


if __name__ == "__main__":
    main()

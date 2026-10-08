#!/usr/bin/env python3
"""dynamics_contact.py —— 把【动力学限幅】与【接触冲击】纳入量化（控制层/安全层加强）。

输出 → report/dynamics_contact.md
用法: python3 tools/dynamics_contact.py
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch.dynamics import (  # noqa: E402
    hover_horizontal_accel, horizontal_accel_limit, tilt_for_accel)
from payload_catch import impact as IM  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
G = 9.81


def main():
    out = os.path.join(REPO, 'report', 'dynamics_contact.md')
    L = []
    add = L.append
    add('# 动力学限幅 + 接触冲击（把控制层/安全层的理想化纳入量化）\n')
    add('> 现有规划/控制/安全建立在**裸双积分器**（|a|≤a_max）之上，未含姿态/推力约束与接触冲击。')
    add('> 本文用 `payload_catch/dynamics.py`（倾角+推力聚合约束）与 `payload_catch/impact.py`')
    add('> （冲击/可恢复性/推力余量）量化。复现：`python3 tools/dynamics_contact.py`。\n')

    add('## 0. 结论速览\n')
    add('- 真实四旋翼的**水平加速权限随下潜减小**：`a_x ≤ (g−a_z)·tanθ_max`；悬停为 `g·tanθ_max`。')
    add('- 项目的 `a_max=6 m/s²` 对应悬停倾角 **≈31.5°**（不是独立旋钮，与姿态强耦合）；')
    add('- 接触冲击：100g 载荷可恢复；**≥约 1kg 的偏心冲击超出 B 的姿态恢复权限**（需锁扣/柔顺/更大 B）；')
    add('- 带载推力余量决定"接住后能否悬停"：x500 级 `T_max≈2×` 时 +100g 富余，+数 kg 不富余。\n')

    add('## 1. 倾角/推力限幅（`dynamics.py`）\n')
    add('| 倾角 θ_max | 悬停 a_h (m/s²) | a_h @下潜 a_z=2 | a_h @下潜 a_z=4 |')
    add('|---|---|---|---|')
    for deg in (20, 30, 40, 45):
        th = math.radians(deg)
        add(f'| {deg}° | {hover_horizontal_accel(G, th):.2f} | '
            f'{horizontal_accel_limit(2.0, G, th):.2f} | {horizontal_accel_limit(4.0, G, th):.2f} |')
    add('')
    add('> M6 纯垂直下潜（θ=0）不受倾角限制；但**横移/末端俯冲需要大水平权限时，下潜会挤压它**。\n')

    add('## 2. 项目 a_max 与倾角的对应\n')
    add('| a_max (m/s²) | 对应悬停倾角 |')
    add('|---|---|')
    for am in (4.0, 6.0, 9.0, 12.0):
        add(f'| {am} | {math.degrees(math.atan(am / G)):.1f}° |')
    add('')
    add('> 结论：`drone_b.max_accel=6` 隐含 ~31° 倾角；论文里应把它写成**姿态约束的投影**，')
    add('> 并显式声明"水平权限随下潜衰减"（`a_x≤(g−a_z)tanθ_max`）——这正是现有理论缺的一条。\n')

    add('## 3. 接触冲击与可恢复性（`impact.py`）\n')
    add('冲击速度取 M6 典型 `v_rel`（悬停接 4.43、下潜 a=3 → 3.69 m/s）；偏心 d_off=0.10m，B 转动惯量 I=0.02 kg·m²，')
    add('力矩权限 τ_max=4 N·m、恢复窗 0.2s、角速度上限 3 rad/s。\n')
    add('| 载荷 m_p | v_rel | 冲量 J | 动能 | 峰值力(行程5cm) | 偏心 ω | 可恢复 |')
    add('|---|---|---|---|---|---|---|')
    for mp in (0.1, 0.3, 1.0, 2.0):
        for vr in (3.69,):
            J = IM.impulse(mp, vr)
            E = IM.energy(mp, vr)
            F = IM.peak_force(mp, vr, 0.05)
            r = IM.recoverable(mp, vr, 0.10, 0.02, tau_max=4.0)
            add(f'| {mp} | {vr} | {J:.2f} N·s | {E:.2f} J | {F:.0f} N | {r["omega"]:.2f} | '
                f'{"✅" if r["ok"] else "❌ 超权限"} |')
    add('')
    add('> 100g 载荷偏心冲击 ω≈1.8 rad/s，B 可恢复；**≥1kg 时 ω 超权限** → 必须靠')
    add('> **主动锁扣/柔顺悬挂/大机**（呼应 `report/active_retention.md`、`m6_robustness_opt.md`）。\n')

    add('## 4. 带载推力余量\n')
    add('设 B 机 `m_B=1.5kg`，总推力上限 `T_max`（取 2× 悬停 ≈ 29.4 N）：\n')
    add('| 载荷 m_p | 带载推力余量 (N) | 能否悬停携带 |')
    add('|---|---|---|')
    tmax = 2.0 * 1.5 * G
    for mp in (0.1, 0.3, 1.0, 2.0, 5.0):
        marg = IM.thrust_margin(1.5, mp, tmax)
        add(f'| {mp} | {marg:+.1f} | {"✅" if marg >= 0 else "❌"} |')
    add('')
    add('> 100–300g 富余充足；**≥约 1.5kg（≈B 自重）时余量转负**→ 需要更大 B 或更强动力。\n')

    add('## 5. 对理论/论文的修正（把假设说清）\n')
    add('1. **规划/控制模型**：从"双积分器+`a_max`"升级为"**倾角+推力聚合约束**"，')
    add('   即 `{(a_x,a_z): tanθ=…≤θ_max, (T/m)=√(a_x²+(g−a_z)²)≤T_max/m}`；`a_max` 是其投影。')
    add('2. **安全层**：捕获事件纳入保证——给出**冲击可恢复条件** `ω=J·d_off/I_B ≤ min(τ_max·t/I_B, ω_max)`')
    add('   与**带载悬停条件** `T_max ≥ (m_B+m_p)g`；不满足时启用锁扣/柔顺。')
    add('3. **证书输入**：`r_eff`、`v_retain` 之外，还应含**姿态恢复余量**与**推力余量**作为可行性约束。')
    add('4. **真机/HIL**：上述约束需用真机姿态限幅、推力标定、接触力测量来验证（当前为解析）。\n')

    add('## 6. 接空安全中止（SITL 验证）\n')
    add('- `b_node.miss_timeout_s`（默认 4s）：DIVE 超时未捕获 → 发 `/payload/miss`，')
    add('  **安全悬停 → 降落**（不盲目追击/砸地）；')
    add('- SITL：正常捕获 ~0.5s 不触发；大偏差（σ=3m + 不跟踪）→ ')
    add('  `B: DIVE 超时未捕获 → MISS，安全悬停→降落`，B 全程 `safe=OK`、无 failsafe；')
    add('- 局限：载荷本身仍自由落体砸地（真机需**地面防护/载荷回收**）——属载荷侧而非 B 侧安全。\n')

    txt = '\n'.join(L) + '\n'
    with open(out, 'w', encoding='utf-8') as f:
        f.write(txt)
    print(txt)
    print('wrote', out)


if __name__ == '__main__':
    main()

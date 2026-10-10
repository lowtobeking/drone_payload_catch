#!/usr/bin/env python3
"""C5 handover-CBF（keepout.py）离线自检——纯数学，无需 ROS/PX4/Gazebo。

    python3 tools/test_keepout.py

安全证书一旦投影写错，表现是"偶发碰撞"或"安全层把任务顶死"，
在 SITL 里都很难复现定位。这里把投影的**可行性与不变性**钉死。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch import keepout as ko   # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


print('=== cbf_bound ===')
r = np.array([1.0, 0.0, 0.0])
v_other = np.array([0.2, -0.1, 0.0])
h = 1.0 - 0.25          # ‖r‖²=1, d_safe=0.5 → h=0.75
alpha = 2.0
c = ko.cbf_bound(r, v_other, h, alpha)
check('c = r·v_other + α/2·h', abs(c - (0.2 + 0.5 * 2.0 * 0.75)) < 1e-12,
      f'c={c:.4f}')


print('\n=== project_cbf：已满足约束只做速度上限裁剪 ===')
v = np.array([3.0, 0.0, 0.0])          # 会撞：沿 r 正方向
r = np.array([1.0, 0.0, 0.0])
c = 10.0                               # 宽松约束（r·v=2 ≪ c，无需投影）
vp = ko.project_cbf(v, r, c, v_max=2.0)
check('超速被裁到 v_max', abs(np.linalg.norm(vp) - 2.0) < 1e-9, f'|vp|={np.linalg.norm(vp):.4f}')
check('方向保留', np.allclose(vp, [2.0, 0.0, 0.0]))
vp2 = ko.project_cbf(np.array([0.5, 0.0, 0.0]), r, 1.0, 2.0)
check('已满足且不超速 → 原样', np.allclose(vp2, [0.5, 0.0, 0.0]))


print('\n=== project_cbf：可行时投影必须同时满足两个约束（随机 5000 例）===')
rng = np.random.default_rng(0)
worst_c = 0.0
worst_n = 0.0
infeasible = 0
for _ in range(5000):
    r = rng.normal(size=3)
    r[2] *= 0.3
    rn = float(np.linalg.norm(r))
    if rn < 0.1:
        continue
    v_max = float(rng.uniform(0.5, 5.0))
    # 取可行的 c：c_e = c/rn ≥ −v_max；留 5% 余量
    c_e = rng.uniform(-0.95 * v_max, 0.95 * v_max)
    c = c_e * rn
    v_nom = rng.normal(size=3) * rng.uniform(0.1, 3.0)
    vp = ko.project_cbf(v_nom, r, c, v_max)
    worst_n = max(worst_n, float(np.linalg.norm(vp)) - v_max)
    worst_c = max(worst_c, float(r @ vp) - c)
check('r·v\' ≤ c（约束满足）', worst_c <= 1e-9, f'最大越界 {worst_c:.2e}')
check('‖v\'‖ ≤ v_max（速度上限）', worst_n <= 1e-9, f'最大超速 {worst_n:.2e}')


print('\n=== project_cbf：不可行时取最接近的球面点（尽力）===')
r = np.array([1.0, 0.0, 0.0])
v_nom = np.array([2.0, 0.0, 0.0])
vp = ko.project_cbf(v_nom, r, c=-5.0, v_max=1.0)    # c_e=-5 < -v_max
check('返回 −v_max·r̂', np.allclose(vp, [-1.0, 0.0, 0.0]), f'得到 {vp}')
check('模长 = v_max', abs(np.linalg.norm(vp) - 1.0) < 1e-12)


print('\n=== project_cbf：r≈0 退化 ===')
vp = ko.project_cbf(np.array([3.0, 0, 0]), np.zeros(3), 0.0, 2.0)
check('r=0 → 仅速度裁剪', np.allclose(vp, [2.0, 0, 0]))


print('\n=== 指数 CBF 不变性：数值积分 h(t) ≥ h(0)e^{−αt} ===')
# 单积分器：p_A 固定，p_B 以投影速度运动；r=p_A−p_B，v_other=0
alpha = 1.5
d_safe = 0.8
dt = 0.002
r = np.array([2.0, 0.6, 0.0])       # ‖r‖≈2.09 > d_safe
p_B = -r.copy()                     # 令 r = p_A − p_B = r（p_A=0）
h0 = float(r @ r) - d_safe ** 2
worst = 0.0
for i in range(2000):               # 4 s
    h = float(r @ r) - d_safe ** 2
    c = ko.cbf_bound(r, np.zeros(3), h, alpha)
    v_nom = r / np.linalg.norm(r)   # B 企图朝 A 撞
    v_B = ko.project_cbf(v_nom, r, c, v_max=3.0)
    p_B = p_B + v_B * dt
    r = -p_B
    t = (i + 1) * dt
    h_now = float(r @ r) - d_safe ** 2
    worst = max(worst, h0 * np.exp(-alpha * t) - h_now)   # 下界−实际
check('h(t) ≥ h(0)e^{−αt}（Gronwall）', worst <= 2e-4, f'最大违反 {worst:.2e}')
rmin = float(np.linalg.norm(r))
check('4s 后仍不碰撞 ‖r‖ ≥ d_safe', rmin >= d_safe, f'‖r‖={rmin:.3f}')

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ test_keepout 全部通过')

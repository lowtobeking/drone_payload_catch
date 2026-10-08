#!/usr/bin/env python3
"""gen_tray.py —— 生成【圆形托盘（围边 + 泡棉缓冲）】Gazebo 模型。

动机（report/robust_geometry_and_retention.md、tools/tray_sizing.py）：真机末端由“漏斗/平盘”
换为**塑料圆形托盘（带围边）+ 泡棉缓冲**。物理保持靠：
  · 围边挡横向（防止滑出/滚出）；
  · 泡棉低恢复系数消除反弹（h ≥ e²·gap 即不弹出）；
落点半径 eff_r = 盘内半径 − 物半宽；保持速度 v_retain = √(2·g·h)/e（h = 围边高 + 凹垫深）。

用 primitive 拼（圆柱底+cushion + N 段薄板围边），不依赖 mesh，ODE 稳定。
几何（相对 x500 model frame；base_link 在 z=0.24）：
  · funnel_link 在 z=0.40；cushion 顶面 z=0.45（≈ base_link 上方 0.21m，与漏斗口一致）；
  · 围边在 z=0.45..0.50（高 0.05）。

生成：
  models/x500_tray/    —— 标准 x500 + 圆形托盘（供 SITL）
  models/funnel_tray/  —— 独立托盘（static，供离线/独立物理测试）

用法: python3 tools/gen_tray.py
"""
from __future__ import annotations

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

# ── 托盘几何/材料参数（可用环境变量覆盖，便于生成不同盘径变体）─────────────
R_IN = float(os.environ.get('TRAY_R_IN', 0.15))   # 盘内半径 (m)  → 默认内径 30cm
RIM_H = float(os.environ.get('TRAY_RIM_H', 0.05))  # 围边高度 (m)
RIM_T = 0.008       # 围边壁厚 (m)
CUSHION_T = 0.03    # 泡棉厚 (m)
BASE_T = 0.02       # 底板厚 (m)
NSEG = 16           # 围边分段
LINK_Z = 0.40       # funnel_link 在 x500 模型中的 z
CUSHION_E = 0.05    # 泡棉恢复系数（低回弹）
CUSHION_MU = 1.5    # 泡棉摩擦
RIM_E = 0.10
RIM_MU = 0.80
_OUT_X500 = os.environ.get('TRAY_OUT_X500', 'x500_tray')
_OUT_TRAY = os.environ.get('TRAY_OUT_TRAY', 'funnel_tray')


def _rim_boxes():
    """围边：N 段竖直薄板绕圆周（局部坐标，funnel_link 系）。"""
    z_lo = CUSHION_T + BASE_T          # 围边底（= cushion 顶）
    z_c = z_lo + RIM_H / 2.0           # 围边中心
    r_c = R_IN + RIM_T / 2.0           # 薄板中心半径
    chord = 2.0 * r_c * math.sin(math.pi / NSEG)
    out = []
    for i in range(NSEG):
        phi = 2.0 * math.pi * i / NSEG
        out.append((r_c * math.cos(phi), r_c * math.sin(phi), z_c, phi, chord))
    return out


def _tray_body(ind: str) -> str:
    """托盘 link 内容（不含 <link> 标签），缩进 ind。"""
    L = []
    p = ind + '  '
    L += [f'{p}<inertial><mass>0.12</mass><inertia>'
          '<ixx>6.0e-4</ixx><iyy>6.0e-4</iyy><izz>6.0e-4</izz>'
          '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>']
    # 底板（结构）
    L += [f'{p}<collision name="tray_base"><pose>0 0 {BASE_T/2:.4f} 0 0 0</pose>'
          f'<geometry><cylinder><radius>{R_IN:.4f}</radius><length>{BASE_T}</length></cylinder></geometry>'
          f'<surface><friction><ode><mu>{RIM_MU}</mu><mu2>{RIM_MU}</mu2></ode></friction>'
          f'<bounce><restitution_coefficient>{RIM_E}</restitution_coefficient></bounce></surface></collision>']
    # 泡棉缓冲（低回弹、高摩擦）——载荷落在这上面
    L += [f'{p}<collision name="tray_cushion"><pose>0 0 {BASE_T + CUSHION_T/2:.4f} 0 0 0</pose>'
          f'<geometry><cylinder><radius>{R_IN:.4f}</radius><length>{CUSHION_T}</length></cylinder></geometry>'
          f'<surface><friction><ode><mu>{CUSHION_MU}</mu><mu2>{CUSHION_MU}</mu2></ode></friction>'
          f'<bounce><restitution_coefficient>{CUSHION_E}</restitution_coefficient></bounce></surface></collision>']
    # 围边（N 段薄板）
    for i, (px, py, pz, yaw, chord) in enumerate(_rim_boxes()):
        pose = f'{px:.4f} {py:.4f} {pz:.4f} 0 0 {yaw:.4f}'
        geom = f'<geometry><box><size>{RIM_T} {chord*1.08:.4f} {RIM_H}</size></box></geometry>'
        L += [f'{p}<collision name="tray_rim_{i}"><pose>{pose}</pose>{geom}'
              f'<surface><friction><ode><mu>{RIM_MU}</mu><mu2>{RIM_MU}</mu2></ode></friction>'
              f'<bounce><restitution_coefficient>{RIM_E}</restitution_coefficient></bounce></surface></collision>']
    # 视觉：底板（灰）+ 泡棉（深灰）+ 围边（蓝）
    L += [f'{p}<visual name="tray_base_v"><pose>0 0 {BASE_T/2:.4f} 0 0 0</pose>'
          f'<geometry><cylinder><radius>{R_IN:.4f}</radius><length>{BASE_T}</length></cylinder></geometry>'
          '<material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.35 0.35 0.38 1</diffuse></material></visual>']
    L += [f'{p}<visual name="tray_cushion_v"><pose>0 0 {BASE_T + CUSHION_T/2:.4f} 0 0 0</pose>'
          f'<geometry><cylinder><radius>{R_IN:.4f}</radius><length>{CUSHION_T}</length></cylinder></geometry>'
          '<material><ambient>0.20 0.20 0.22 1</ambient><diffuse>0.20 0.20 0.22 1</diffuse></material></visual>']
    for i, (px, py, pz, yaw, chord) in enumerate(_rim_boxes()):
        pose = f'{px:.4f} {py:.4f} {pz:.4f} 0 0 {yaw:.4f}'
        L += [f'{p}<visual name="tray_rim_v_{i}"><pose>{pose}</pose>'
              f'<geometry><box><size>{RIM_T} {chord*1.08:.4f} {RIM_H}</size></box></geometry>'
              '<material><ambient>0.15 0.55 0.85 1</ambient><diffuse>0.15 0.55 0.85 1</diffuse></material></visual>']
    return '\n'.join(L)


X500_TMPL = """<?xml version="1.0" encoding="UTF-8"?>
<!-- x500_tray —— 自动生成（tools/gen_tray.py）。标准 x500 + 圆形托盘（围边+泡棉缓冲）。
     盘内半径 {rin} m（内径 {dia} cm），围边高 {rim} m，泡棉 e={e}。
     泡棉顶面 z≈{ztop:.3f}（≈ base_link 上方 0.21m）；捕获判据用 funnel_mount_height=0.21。 -->
<sdf version='1.9'>
  <model name='x500_tray'>
    <include merge='true'><uri>model://x500</uri></include>
    <link name="funnel_link">
      <pose>0 0 {z} 0 0 0</pose>
{body}
    </link>
    <joint name="funnel_joint" type="fixed"><parent>base_link</parent><child>funnel_link</child></joint>
  </model>
</sdf>
"""

STANDALONE_TMPL = """<?xml version="1.0" encoding="UTF-8"?>
<!-- {name} —— 自动生成（tools/gen_tray.py）。独立圆形托盘（static），供物理/落物测试。 -->
<sdf version='1.9'>
  <model name='{name}'>
    <static>true</static>
    <link name="{link}">
      <pose>0 0 0 0 0 0</pose>
{body}
    </link>
  </model>
</sdf>
"""


def _write(dirname, content, desc):
    d = os.path.join(REPO, 'models', dirname)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, 'model.sdf'), 'w', encoding='utf-8') as f:
        f.write(content)
    with open(os.path.join(d, 'model.config'), 'w', encoding='utf-8') as f:
        f.write(f"""<?xml version="1.0"?>
<model><name>{dirname}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<author><name>caolihao</name></author><description>{desc}</description></model>
""")
    print('wrote', os.path.join(d, 'model.sdf'))


def main():
    ztop = LINK_Z + BASE_T + CUSHION_T
    _write(_OUT_X500,
           X500_TMPL.format(z=LINK_Z, body=_tray_body('    '),
                            rin=R_IN, dia=int(R_IN * 200), rim=RIM_H, e=CUSHION_E,
                            ztop=ztop).replace("name='x500_tray'", f"name='{_OUT_X500}'"),
           'x500 + circular tray with rim and cushion (real-hardware end effector)')
    _write(_OUT_TRAY,
           STANDALONE_TMPL.format(name=_OUT_TRAY, link='tray',
                                  body=_tray_body('    ')),
           'standalone circular tray with rim and cushion')


if __name__ == '__main__':
    main()

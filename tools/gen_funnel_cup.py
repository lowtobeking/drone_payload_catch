#!/usr/bin/env python3
"""gen_funnel_cup.py —— 生成【空心导向漏斗（杯）】Gazebo 模型。

动机（report/robust_geometry_and_retention.md §B）：当前 x500_funnel 是**实心圆锥平顶盘**，
靠摩擦+低恢复系数"砸住"，偏心/带横向速度/平台倾斜时载荷会滑出。本模型升级为**空心锥杯**：
  · 口半径 R_m；内壁为收敛锥面把偏心载荷**导向中心**；底部实心杯底 → **几何围栏保持**。

用 N 个倾斜薄板(box)拼锥内壁 + 一块底圆盘（primitive 碰撞，ODE 稳，不用 mesh）。

生成：
  models/x500_funnel_cup/  —— 标准 x500 + 锥杯（供 SITL）
  models/funnel_cup/       —— 独立的锥杯（供离线/独立物理测试）
  models/funnel_flat/      —— 独立的实心平顶盘（对照）

用法: python3 tools/gen_funnel_cup.py
"""
from __future__ import annotations

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

R_MOUTH = 0.30      # 口半径
R_BOTTOM = 0.10     # 杯底半径
DEPTH = 0.14        # 杯深
WALL_T = 0.008      # 壁厚
NSEG = 16           # 分段
LINK_Z = 0.38       # funnel_link 在 x500 模型中的 z


def _wall_boxes():
    z_top, z_bot = DEPTH / 2.0, -DEPTH / 2.0
    dr, dz = R_BOTTOM - R_MOUTH, z_bot - z_top
    L = math.hypot(dr, dz)
    beta = math.atan2(-dz, -dr) if dr != 0 else 0.0
    r_mid = (R_MOUTH + R_BOTTOM) / 2.0
    pitch = -beta
    out = []
    for i in range(NSEG):
        phi = 2.0 * math.pi * i / NSEG
        chord = 2.0 * r_mid * math.sin(math.pi / NSEG)
        out.append((r_mid * math.cos(phi), r_mid * math.sin(phi), 0.0,
                    0.0, pitch, phi, L, chord))
    return out


def _cup_body(ind: str) -> str:
    """锥杯的 link 内容（不含 <link> 标签），缩进 ind。"""
    L = []
    p = ind + '  '
    L += [f'{p}<inertial><mass>0.08</mass><inertia>'
          '<ixx>4.0e-4</ixx><iyy>4.0e-4</iyy><izz>4.0e-4</izz>'
          '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>']
    L += [f'{p}<collision name="cup_bottom"><pose>0 0 {-DEPTH/2:.4f} 0 0 0</pose>'
          f'<geometry><cylinder><radius>{R_BOTTOM:.4f}</radius><length>0.02</length></cylinder></geometry>'
          '<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>'
          '<bounce><restitution_coefficient>0.03</restitution_coefficient></bounce></surface></collision>']
    for i, (px, py, pz, roll, pitch, yaw, bl, chord) in enumerate(_wall_boxes()):
        pose = f'{px:.4f} {py:.4f} {pz:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}'
        geom = (f'<geometry><box><size>{bl:.4f} {chord*1.06:.4f} {WALL_T}</size></box></geometry>')
        L += [f'{p}<collision name="cup_wall_{i}"><pose>{pose}</pose>{geom}'
              '<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>'
              '<bounce><restitution_coefficient>0.03</restitution_coefficient></bounce></surface></collision>']
    # 只保留一个 visual（底+壁简化为一个近似锥 visual 用底圆盘代表，避免视觉过密）
    L += [f'{p}<visual name="cup_vis"><pose>0 0 {-DEPTH/2:.4f} 0 0 0</pose>'
          f'<geometry><cylinder><radius>{R_BOTTOM:.4f}</radius><length>0.02</length></cylinder></geometry>'
          '<material><ambient>0.15 0.55 0.85 1</ambient><diffuse>0.15 0.55 0.85 1</diffuse></material></visual>']
    for i, (px, py, pz, roll, pitch, yaw, bl, chord) in enumerate(_wall_boxes()):
        pose = f'{px:.4f} {py:.4f} {pz:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}'
        L += [f'{p}<visual name="cup_wv_{i}"><pose>{pose}</pose>'
              f'<geometry><box><size>{bl:.4f} {chord*1.06:.4f} {WALL_T}</size></box></geometry>'
              '<material><ambient>0.85 0.55 0.15 1</ambient><diffuse>0.85 0.55 0.15 1</diffuse></material></visual>']
    return '\n'.join(L)


X500_TMPL = """<?xml version="1.0" encoding="UTF-8"?>
<!-- x500_funnel_cup —— 自动生成（tools/gen_funnel_cup.py）。标准 x500 + 空心导向锥杯。 -->
<sdf version='1.9'>
  <model name='x500_funnel_cup'>
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
<!-- {name} —— 自动生成（tools/gen_funnel_cup.py）。独立{desc}（静止），供物理测试。 -->
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

FLAT_BODY = """      <inertial><mass>0.08</mass><inertia><ixx>4e-4</ixx><iyy>4e-4</iyy><izz>4e-4</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="disk"><pose>0 0 0 0 0 0</pose>
        <geometry><cylinder><radius>0.30</radius><length>0.02</length></cylinder></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
        <bounce><restitution_coefficient>0.05</restitution_coefficient></bounce></surface></collision>
      <visual name="disk_v"><pose>0 0 0 0 0 0</pose>
        <geometry><cylinder><radius>0.30</radius><length>0.02</length></cylinder></geometry>
        <material><ambient>0.15 0.55 0.85 1</ambient><diffuse>0.15 0.55 0.85 1</diffuse></material></visual>"""


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
    _write('x500_funnel_cup',
           X500_TMPL.format(z=LINK_Z, body=_cup_body('    ')),
           'x500 + hollow conical cup funnel (guided capture + retention)')
    _write('funnel_cup',
           STANDALONE_TMPL.format(name='funnel_cup', link='cup',
                                  desc='空心导向锥杯', body=_cup_body('    ')),
           'standalone hollow conical cup funnel')
    _write('funnel_flat',
           STANDALONE_TMPL.format(name='funnel_flat', link='disk',
                                  desc='实心平顶盘', body=FLAT_BODY),
           'standalone flat disk funnel')


if __name__ == '__main__':
    main()

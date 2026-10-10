#!/usr/bin/env python3
"""配置契约测试（`config/catch_scenarios.yaml` = 单一真值源）。

    python3 tools/test_config.py

项目约定"所有几何/参数/工况只写在 yaml，不在代码里硬编码"。这个测试守住该
契约：结构完整、scenario 引用的 layout 存在、几何自洽、真机段齐全——
避免改 yaml 后在仿真/真机里才炸。
"""
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG_PATH = os.path.join(ROOT, 'config', 'catch_scenarios.yaml')

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


cfg = yaml.safe_load(open(CFG_PATH))
d = cfg.get('defaults', {})
layouts = cfg.get('layouts', {})
scenarios = cfg.get('scenarios', {})
thr = cfg.get('thresholds', {})
real = cfg.get('real', {})

print('=== 顶层结构 ===')
for k in ('defaults', 'layouts', 'scenarios', 'thresholds', 'real'):
    check(f'含 {k} 段', k in cfg)
check('至少有 1 个 layout / scenario', len(layouts) > 0 and len(scenarios) > 0)

print('\n=== defaults 数值自洽 ===')
check('g > 0', float(d.get('g', 0)) > 0)
check('control_hz > 0', float(d.get('control_hz', 0)) > 0)
check('duration_s > 0', float(d.get('duration_s', 0)) > 0)
check('drone_b.max_speed/accel > 0',
      float(d.get('drone_b', {}).get('max_speed', 0)) > 0
      and float(d.get('drone_b', {}).get('max_accel', 0)) > 0)
cap = d.get('capture', {})
check('capture.radius > 0', float(cap.get('radius', 0)) > 0)
check('capture.rel_speed > 0', float(cap.get('rel_speed', 0)) > 0)
fun = cap.get('funnel', {})
check('funnel.mouth_radius > object_radius',
      float(fun.get('mouth_radius', 0)) > float(fun.get('object_radius', 1)))
check('funnel.depth > 0 且 0 ≤ restitution < 1',
      float(fun.get('depth', 0)) > 0 and 0.0 <= float(fun.get('restitution', 1)) < 1.0)

print('\n=== thresholds ===')
for k in ('capture_success', 'miss_dist', 'rel_speed', 'peak_accel_ratio'):
    check(f'thresholds.{k} 存在', k in thr)

print('\n=== layouts 几何 ===')
for name, lay in layouts.items():
    a = lay.get('a_init', [])
    b = lay.get('b_standby', [])
    check(f'{name}: a_init/b_standby 为 3 维', len(a) == 3 and len(b) == 3)
    check(f'{name}: a_vel 为 3 维（若给出）',
          ('a_vel' not in lay) or len(lay['a_vel']) == 3)
lay_sd = layouts.get('stack_drop', {})
check('stack_drop: A 在 B 上方（NED z：a_init.z < b_standby.z）',
      lay_sd.get('a_init', [0, 0, 0])[2] < lay_sd.get('b_standby', [0, 0, 0])[2])

print('\n=== scenarios 引用完整 ===')
for name, sc in scenarios.items():
    layname = sc.get('layout')
    check(f'{name}.layout 存在', layname in layouts, f'→ {layname}')
    if sc.get('mode') == 'stack_drop':
        check(f'{name}: stack_drop 用 stack_drop layout', layname == 'stack_drop')
    # scenario 内覆盖的 funnel 也要自洽
    f = sc.get('capture', {}).get('funnel')
    if f:
        check(f'{name}: 覆盖 funnel 口半径 > 物半径',
              float(f.get('mouth_radius', 0)) > float(f.get('object_radius', 1)))

REQUIRED = [
    'M1_basic', 'M2_line_v05', 'M2_line_v10', 'M2_line_v20', 'M2_crosswind',
    'M3_high', 'M3_wind_drag', 'M3p_line_mpc', 'M3p_high_mpc',
    'M4_line_kf', 'M4_high_kf',
    'M6_stack_drop', 'M6_stack_bigfunnel', 'M6_stack_tray', 'M6_stack_lock',
]
missing = [n for n in REQUIRED if n not in scenarios]
check('文档/工具引用的工况都在', not missing, f'缺 {missing}')

print('\n=== 真机接入段 ===')
for k in ('relnav', 'calibration', 'contact'):
    check(f'real.{k} 存在', k in real)
cal = real.get('calibration', {})
check('px4_z_bias ≥ 0', float(cal.get('px4_z_bias', -1)) >= 0)
check('payload_release_offset > 0', float(cal.get('payload_release_offset', 0)) > 0)
check('relnav.a_lever/b_lever 为 3 维',
      len(real.get('relnav', {}).get('a_lever', [])) == 3
      and len(real.get('relnav', {}).get('b_lever', [])) == 3)

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print(f'✅ test_config 全部通过（{len(scenarios)} 个工况 / {len(layouts)} 个 layout）')

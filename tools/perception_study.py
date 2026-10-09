#!/usr/bin/env python3
"""perception_study.py —— 视觉感知对交接的影响（把"真值替身"换成相机模型）。

用 `payload_catch/perception.py` 的相机模型（FOV 门控 + 距离相关误差 + 深度 + 丢帧）
替换离线仿真里的"真值+高斯噪声"，量化：相机参数、释放误差、编队运动下的捕获退化。

输出 → report/perception_study.md
用法: python3 tools/perception_study.py
"""
from __future__ import annotations

import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import yaml  # noqa: E402
from payload_catch.stack_drop import simulate_stack, StackNoise  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
YAML = os.path.join(REPO, 'config', 'catch_scenarios.yaml')


def run(cfg, name, perc, rel_sigma, release_sigma, n, v_release=(0, 0, 0), seed0=0):
    d = cfg['defaults']; s0 = cfg['scenarios'][name]; lay = cfg['layouts'][s0['layout']]
    ok = 0; hz = []
    for i in range(n):
        s = {k: (dict(v) if isinstance(v, dict) else v) for k, v in s0.items()}
        if perc:
            s['perception'] = perc
        s['v_release'] = list(v_release)
        noise = StackNoise(release_pos_sigma=release_sigma, rel_pos_sigma=rel_sigma,
                           rel_latency=0.05, seed=seed0 + i)
        r, _ = simulate_stack(d, lay, s, noise=noise)
        ok += int(r.success)
        if r.success:
            hz.append(r.horiz_miss_at_capture)
    return ok, n, (st.mean(hz) if hz else float('nan'))


def main():
    cfg = yaml.safe_load(open(YAML, encoding='utf-8'))
    n = 200
    L = []
    add = L.append
    add('# 视觉感知对交接的影响（相机模型）\n')
    add('> 离线仿真默认用"真值+高斯噪声"替身。本文用相机模型 '
        '(`payload_catch/perception.py`)：FOV 门控 + 距离相关横向误差 + 深度误差 + 丢帧。')
    add('> 复现：`python3 tools/perception_study.py`。\n')

    add('## 0. 结论速览\n')
    add('- 相机误差**距离相关**（近小远大）：近场（载荷→相机 ~0.5m）误差小于固定 σ=0.05 替身；')
    add('- **大释放误差**下 FOV/丢帧才会咬人（载荷进入视场晚、横向偏差大）；')
    add('- 编队运动下相机需"盯住"横移的载荷，**窄 FOV / 高丢帧**退化明显。\n')

    cam_sets = [
        ('camera 60° k=0.003 drop0', {'fov_deg': 60, 'lateral_k': 0.003, 'depth_sigma': 0.02, 'dropout': 0.0}),
        ('camera 60° k=0.005 drop0.1', {'fov_deg': 60, 'lateral_k': 0.005, 'depth_sigma': 0.02, 'dropout': 0.1}),
        ('camera 45° k=0.008 drop0.2', {'fov_deg': 45, 'lateral_k': 0.008, 'depth_sigma': 0.03, 'dropout': 0.2}),
        ('camera 30° k=0.012 drop0.3', {'fov_deg': 30, 'lateral_k': 0.012, 'depth_sigma': 0.04, 'dropout': 0.3}),
        ('camera 10° k=0.02 drop0.5（差）', {'fov_deg': 10, 'lateral_k': 0.02, 'depth_sigma': 0.05, 'dropout': 0.5}),
    ]

    add('## 1. 标称（释放误差 σ=0.05）：相机 vs 真值替身\n')
    add('| 感知 | 成功率 | 水平偏差均值 |')
    add('|---|---|---|')
    ok, nn, hz = run(cfg, 'M6_stack_tray', None, 0.05, 0.05, n)
    add(f'| 真值替身 σ=0.05 | {ok}/{nn} ({100*ok//nn}%) | {hz:.3f} |')
    for tag, perc in cam_sets:
        ok, nn, hz = run(cfg, 'M6_stack_tray', perc, 0.0, 0.05, n)
        add(f'| {tag} | {ok}/{nn} ({100*ok//nn}%) | {hz:.3f} |')
    add('')

    add('## 2. 大释放误差（σ=0.20）：感知成为瓶颈\n')
    add('| 感知 | 成功率 | 水平偏差均值 |')
    add('|---|---|---|')
    ok, nn, hz = run(cfg, 'M6_stack_tray', None, 0.05, 0.20, n)
    add(f'| 真值替身 σ=0.05 | {ok}/{nn} ({100*ok//nn}%) | {hz:.3f} |')
    for tag, perc in (cam_sets[0], cam_sets[3], cam_sets[4]):
        ok, nn, hz = run(cfg, 'M6_stack_tray', perc, 0.0, 0.20, n)
        add(f'| {tag} | {ok}/{nn} ({100*ok//nn}%) | {hz:.3f} |')
    add('')

    add('## 3. 编队同速（v=0.5 m/s 水平）：相机需盯住横移载荷\n')
    add('| 感知 | 成功率 | 水平偏差均值 |')
    add('|---|---|---|')
    ok, nn, hz = run(cfg, 'M6_stack_bigfunnel', None, 0.05, 0.05, n, v_release=(0.5, 0, 0))
    add(f'| 真值替身 σ=0.05 | {ok}/{nn} ({100*ok//nn}%) | {hz:.3f} |')
    for tag, perc in (cam_sets[0], cam_sets[2], cam_sets[3], cam_sets[4]):
        ok, nn, hz = run(cfg, 'M6_stack_bigfunnel', perc, 0.0, 0.05, n, v_release=(0.5, 0, 0))
        add(f'| {tag} | {ok}/{nn} ({100*ok//nn}%) | {hz:.3f} |')
    add('')

    add('## 4. 对仿真/真机的意义\n')
    add('1. 相机模型比固定高斯更真实：**近场精度好、远场差、有 FOV 与丢帧**；')
    add('2. 现在可把 `/payload/state` 从"真值替身"换成相机模型（离线与 SITL 均可），')
    add('   量化"感知真实化"后的成功率——这是 sim-to-real 的关键一步；')
    add('3. 真机接入：把 `perception.py` 的相机参数换成实测（FOV/分辨率/帧率/深度），')
    add('   或直接用真实相机节点发 `/payload/state`。\n')

    out = os.path.join(REPO, 'report', 'perception_study.md')
    txt = '\n'.join(L) + '\n'
    open(out, 'w', encoding='utf-8').write(txt)
    print(txt)
    print('wrote', out)


if __name__ == '__main__':
    main()

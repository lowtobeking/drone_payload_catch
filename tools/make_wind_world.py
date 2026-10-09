#!/usr/bin/env python3
"""make_wind_world.py —— 由 SITL world 生成【带风场】的 world（注入 gz WindEffects 插件）。

SITL 默认 world 无风（`enable_wind=false`）。本工具复制该 world，并在 Physics 插件后注入
`gz-sim-wind-effects-system`，给定水平风速与方向（度，0=+X/东，逆时针）。配合载荷模型里的
`<enable_wind>true</enable_wind>`，载荷会受风漂移（用于 SITL 风鲁棒性）。

用法:
  python3 tools/make_wind_world.py --wind 3.0 --dir-deg 0 --out /tmp/default_wind.sdf
  然后: SITL_WORLD=/tmp/default_wind.sdf bash run_m6_sitl.sh 70
"""
from __future__ import annotations

import argparse
import os
import re
import sys

DEFAULT_SRC = os.path.expanduser(
    '~/drone_package_20260908/gz_overrides/worlds/default.sdf')

WIND_PLUGIN = """    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>1.0</force_approximation_scaling_factor>
      <horizontal>
        <magnitude>
          <time_for_rise>0.0</time_for_rise>
          <sin_amp>0.0</sin_amp><sin_freq>0.0</sin_freq><sin_phase>0.0</sin_phase>
          <noise>0.0</noise><steady>{mag}</steady>
        </magnitude>
        <direction>
          <time_for_rise>0.0</time_for_rise>
          <sin_amp>0.0</sin_amp><sin_freq>0.0</sin_freq><sin_phase>0.0</sin_phase>
          <noise>0.0</noise><steady>{dir_rad}</steady>
        </direction>
      </horizontal>
      <vertical>
        <magnitude>
          <time_for_rise>0.0</time_for_rise>
          <sin_amp>0.0</sin_amp><sin_freq>0.0</sin_freq><sin_phase>0.0</sin_phase>
          <noise>0.0</noise><steady>0.0</steady>
        </magnitude>
        <direction>
          <time_for_rise>0.0</time_for_rise>
          <sin_amp>0.0</sin_amp><sin_freq>0.0</sin_freq><sin_phase>0.0</sin_phase>
          <noise>0.0</noise><steady>0.0</steady>
        </direction>
      </vertical>
    </plugin>
"""


def make(src: str, out: str, wind: float, dir_deg: float):
    txt = open(src, encoding='utf-8').read()
    dir_rad = dir_deg * 3.141592653589793 / 180.0
    plugin = WIND_PLUGIN.format(mag=wind, dir_rad=dir_rad)
    # 在 Physics 插件那一行后插入
    m = re.search(r"[ \t]*<plugin[^>]*gz-sim-physics-system[^>]*/>\n", txt)
    if not m:
        print('错误：未找到 Physics 插件行', file=sys.stderr)
        return 1
    txt = txt[:m.end()] + plugin + txt[m.end():]
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    open(out, 'w', encoding='utf-8').write(txt)
    print(f'wrote {out}  (wind={wind} m/s, dir={dir_deg}°={dir_rad:.3f} rad)')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default=os.environ.get('SITL_WORLD', DEFAULT_SRC))
    ap.add_argument('--out', default='/tmp/default_wind.sdf')
    ap.add_argument('--wind', type=float, default=3.0)
    ap.add_argument('--dir-deg', type=float, default=0.0)
    a = ap.parse_args()
    raise SystemExit(make(a.src, a.out, a.wind, a.dir_deg))


if __name__ == '__main__':
    main()

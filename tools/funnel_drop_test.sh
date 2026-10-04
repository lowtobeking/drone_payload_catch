#!/usr/bin/env bash
# funnel_drop_test.sh —— 独立物理测试：把载荷投到【倾斜的】空心锥杯 vs 实心平顶盘上，
# 看是否被"导向+保持"（杯） vs "滑出"（平盘）。
#
# 用法: bash tools/funnel_drop_test.sh
set +u
MODELS="$HOME/drone_payload_catch/models"
export GZ_SIM_RESOURCE_PATH="$MODELS:${GZ_SIM_RESOURCE_PATH:-}"
TILT="${TILT:-0.0}"        # 平台倾斜 (rad)
OFFX="${OFFX:-0.15}"        # 落点相对中心偏移 (m)

run_test () {
  local name="$1" funnel="$2"
  local w="/tmp/funnel_test_$name.sdf"
  cat > "$w" <<EOF
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="ft_$name">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <gravity>0 0 -9.8</gravity>
    <include><uri>model://$funnel</uri><name>$funnel</name><pose>0 0 0 $TILT 0 0</pose></include>
    <include><uri>model://payload</uri><name>payload</name><pose>$OFFX 0 1.5 0 0 0</pose></include>
  </world>
</sdf>
EOF
  pkill -9 -f "funnel_test_$name" 2>/dev/null; sleep 1
  gz sim -s -r "$w" > "/tmp/ft_$name.log" 2>&1 &
  sleep 6
  timeout 5 gz topic -e -t /payload/odom -n 1 > "/tmp/ft_${name}_odom.txt" 2>/dev/null
  pkill -9 -f "funnel_test_$name" 2>/dev/null
  sleep 1
  python3 - "$name" "/tmp/ft_${name}_odom.txt" <<'PY'
import re, sys
name, fn = sys.argv[1], sys.argv[2]
txt = open(fn, encoding='utf-8', errors='ignore').read()
m = re.search(r'position\s*\{(.*?)\}', txt, re.S)
if m:
    blk = m.group(1)
    x = re.search(r'x:\s*([-\d.eE]+)', blk); y = re.search(r'y:\s*([-\d.eE]+)', blk); z = re.search(r'z:\s*([-\d.eE]+)', blk)
    px, py, pz = float(x.group(1)), float(y.group(1)), float(z.group(1))
    print(f'  {name:11s}: 载荷末态 ENU=({px:+.3f},{py:+.3f},{pz:+.3f})  '
          f'→ {"保持(在杯内/盘上)" if pz > -0.6 else "滑出/掉落"}')
else:
    print(f'  {name:11s}: 未取到载荷位姿（可能已掉出仿真范围）→ 滑出/掉落')
PY
}

echo "== 平台倾斜 ${TILT} rad  (落点偏移 ${OFFX} m) =="
run_test cup  funnel_cup
run_test flat funnel_flat

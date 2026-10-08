#!/usr/bin/env bash
# tray_tilt_test.sh —— 独立物理测试：把 6cm/100g 载荷投到【倾斜的】末端上，测保持能力。
#   对比：圆形托盘 funnel_tray（围边+泡棉） vs 空心锥杯 funnel_cup vs 实心平盘 funnel_flat。
#   倾斜（pitch）越大，越容易滑/滚出；围边+高摩擦+低回弹应显著抬高临界倾角。
#
# 用法: bash tools/tray_tilt_test.sh
#   TILTS_DEG="0 10 20 25 30 40 50" OFFX=0.05 DROP_H=1.5 bash tools/tray_tilt_test.sh
set +u
MODELS="$HOME/drone_payload_catch/models"
export GZ_SIM_RESOURCE_PATH="$MODELS:/home/caolihao/drone_package_20260908/PX4-Autopilot-1.16/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
DROP_H="${DROP_H:-1.5}"
OFFX="${OFFX:-0.05}"
TILTS_DEG="${TILTS_DEG:-0 10 20 25 30 40 50}"
MODELS_LIST="${MODELS_LIST:-funnel_tray funnel_cup funnel_flat}"

run_test () {
  local model="$1" tdeg="$2"
  local tr; tr=$(python3 -c "import math;print(round(math.radians($tdeg),4))")
  local w="/tmp/tray_tilt_${model}_${tdeg}.sdf"
  cat > "$w" <<EOF
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="tt">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <gravity>0 0 -9.8</gravity>
    <include><uri>model://$model</uri><name>$model</name><pose>0 0 0 0 $tr 0</pose></include>
    <include><uri>model://payload_100g</uri><name>payload</name><pose>$OFFX 0 $DROP_H 0 0 0</pose></include>
  </world>
</sdf>
EOF
  pkill -9 -f "tray_tilt_${model}_${tdeg}" 2>/dev/null; sleep 1
  gz sim -s -r "$w" > "/tmp/tt_${model}_${tdeg}.log" 2>&1 &
  sleep 6
  timeout 5 gz topic -e -t /payload/odom -n 1 > "/tmp/tt_${model}_${tdeg}_odom.txt" 2>/dev/null
  pkill -9 -f "tray_tilt_${model}_${tdeg}" 2>/dev/null
  sleep 1
  python3 - "$model" "$tdeg" "/tmp/tt_${model}_${tdeg}_odom.txt" <<'PY'
import re, sys, math
model, tdeg, fn = sys.argv[1], sys.argv[2], sys.argv[3]
txt = open(fn, encoding='utf-8', errors='ignore').read()
m = re.search(r'position\s*\{(.*?)\}', txt, re.S)
if not m:
    print(f'{model}|{tdeg}|MISS|nan|nan'); sys.exit()
b = m.group(1)
gx = lambda k: float(re.search(k+r':\s*([-\d.eE]+)', b).group(1))
x, y, z = gx('x'), gx('y'), gx('z')
dr = math.hypot(x, y)
keep = (z > -0.6) and (dr < 0.6)
print(f'{model}|{tdeg}|{"KEEP" if keep else "LOST"}|{z:+.3f}|{dr:.3f}')
PY
}

echo "== 倾斜保持测试（pitch；落点偏移 ${OFFX}m，下落 ${DROP_H}m，载荷 6cm/100g）=="
out=/tmp/tray_tilt_matrix.txt; : > "$out"
for model in $MODELS_LIST; do
  row="$model"
  for t in $TILTS_DEG; do
    res=$(run_test "$model" "$t" | tail -1)
    verdict=$(echo "$res" | cut -d'|' -f3)
    [ "$verdict" = "KEEP" ] && row="$row | ✅" || row="$row | ❌"
    echo "$res" >> "$out"
  done
  echo "$row"
done
echo
echo "对角（倾角°）: $TILTS_DEG"
echo "== 原始数据 =="; cat "$out"

#!/usr/bin/env bash
# gz_foam_drop_test.sh —— 虚拟落物台：Gazebo 里把 6cm/100g 方块从已知高度落到【托盘泡棉】，
#   记录 /world/fd/pose/info，量回弹高度 → 有效恢复系数 e = √(h/H)。
#   用于 (a) 验证 foam_drop_test.py 的测量方法；(b) 给出模型泡棉的参考 e。
#
# 用法: bash tools/gz_foam_drop_test.sh
#   HEIGHTS="0.3 0.6 1.0" bash tools/gz_foam_drop_test.sh
set +u
MODELS="$HOME/drone_payload_catch/models"
export GZ_SIM_RESOURCE_PATH="$MODELS:/home/caolihao/drone_package_20260908/PX4-Autopilot-1.16/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
HEIGHTS="${HEIGHTS:-0.3 0.6 1.0}"
SURF=0.05      # 托盘泡棉顶面（funnel_tray 局部 z）
REST=0.08      # 方块静止时中心 z（泡棉顶 + 半高 0.03）
: > /tmp/foam_e_results.txt

for H in $HEIGHTS; do
  z0=$(python3 -c "print(round($SURF+$H,3))")
  w="/tmp/foam_drop_${H}.sdf"
  cat > "$w" <<EOF
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="fd">
    <physics type="ode"><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <gravity>0 0 -9.8</gravity>
    <include><uri>model://funnel_tray</uri><name>funnel_tray</name><pose>0 0 0 0 0 0</pose></include>
  </world>
</sdf>
EOF
  pkill -9 -f "foam_drop_${H}" 2>/dev/null; sleep 1
  gz sim -s -r "$w" > "/tmp/fd_${H}.log" 2>&1 &
  sleep 4                                   # 等世界加载
  timeout 7 gz topic -e -t /world/fd/pose/info > "/tmp/fd_${H}_pose.txt" 2>/dev/null &
  sleep 1
  # 动态生成载荷（录制已开始，落体过程被完整记录）
  gz service -s /world/fd/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$MODELS/payload_100g/model.sdf\", name: \"payload\", allow_renaming: false, pose: { position: { x: 0, y: 0, z: $z0 } }" >/dev/null 2>&1
  sleep 5
  pkill -9 -f "foam_drop_${H}" 2>/dev/null
  sleep 1
  python3 - "$H" "$z0" "$REST" "/tmp/fd_${H}_pose.txt" >> /tmp/foam_e_results.txt <<'PY'
import re, sys, math
H, z0, rest, fn = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
txt = open(fn, encoding='utf-8', errors='ignore').read()
zs = []
for m in re.finditer(r'name:\s*"payload"\s*(.*?)position\s*\{([^}]*)\}', txt, re.S):
    zm = re.search(r'z:\s*([-\d.eE]+)', m.group(2))
    if zm:
        zs.append(float(zm.group(1)))
if len(zs) < 10:
    print(f'H={H:.2f}  采样不足({len(zs)}) → 跳过'); sys.exit()
zmax, zmin = max(zs), min(zs)
h_drop = max(1e-6, zmax - rest)
i_lo = zs.index(zmin)
z_reb = max(zs[i_lo:]) if i_lo < len(zs) - 1 else rest
h_reb = max(0.0, z_reb - rest)
e = math.sqrt(min(h_reb / h_drop, 1.0))
print(f'H={H:.2f}m  zmax={zmax:.3f} zmin={zmin:.3f}  下落={h_drop:.3f} 回弹={h_reb*100:.1f}cm  → e={e:.3f}')
PY
done
echo "== 虚拟落物台结果（模型泡棉 restitution=0.05）=="
cat /tmp/foam_e_results.txt
echo
echo "== 把实测 e 喂给选型工具 =="
echo "  python3 tools/foam_drop_test.py --h-drop <H> --rebounds <h1> <h2> ..."

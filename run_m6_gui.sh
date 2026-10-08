#!/usr/bin/env bash
# run_m6_gui.sh — M6 可视化仿真（Gazebo GUI 显示到 Windows 桌面，WSLg）。
# gz GUI + 2×PX4(1.16) + agent + A正上方释放 / B对正+温和下潜+刚性漏斗捕获。
# 用法: bash ~/drone_payload_catch/run_m6_gui.sh [观察秒数, 默认 110]
set +u
KEEP="${1:-110}"
B_OFFSET="${B_OFFSET:-5.0,0.0,0.0}"
B_POSE_ENU="${B_POSE_ENU:-0,5.0,0,0,0,0}"

# ── 编队同速投放（M6-moving）：设 FORMATION_VEL 非零即启用 ──
# 流程：A/B 先到同一投影点悬停 → /formation/start → 两机同向同速小速度巡航 →
#       载荷挂载到 A 随飞 → 对正后 A 释放（载荷继承 A 速度）→ B 下潜用漏斗接。
FORMATION_VEL="${FORMATION_VEL:-0.0,0.0,0.0}"
if [ "$FORMATION_VEL" = "0.0,0.0,0.0" ] || [ "$FORMATION_VEL" = "0,0,0" ]; then
  ATTACH=false
  PAYLOAD_MODEL="$HOME/drone_payload_catch/models/payload/model.sdf"
  RELEASE_Z="${RELEASE_Z:-0.15}"
  A_HOVER="${A_HOVER:-0.0,0.0,-4.5}"
  B_STANDBY="${B_STANDBY:-0.0,0.0,-3.5}"
else
  ATTACH=true
  PAYLOAD_MODEL="$HOME/drone_payload_catch/models/payload_attached/model.sdf"
  # 挂载型：offset 需大于 A 起落架（~0.23m）+ 留裕量，且 A/B 间距拉开以保证下落高度
  RELEASE_Z="${RELEASE_Z:-0.45}"
  A_HOVER="${A_HOVER:-0.0,0.0,-5.0}"
  B_STANDBY="${B_STANDBY:-0.0,0.0,-3.3}"
fi
RELEASE_OFFSET="0.0,0.0,$RELEASE_Z"

# 末端能力：FUNNEL_TYPE=flat(默认)/cup(空心导向锥杯)/tray(圆形托盘)；FUNNEL_MOUTH 指定口/盘半径。
#   例：FUNNEL_TYPE=cup bash run_m6_gui.sh 110   /   FUNNEL_MOUTH=0.30 bash run_m6_gui.sh 110
#       FUNNEL_TYPE=tray bash run_m6_gui.sh 110（圆形托盘：围边+泡棉，载荷 6cm/100g）
FUNNEL_TYPE="${FUNNEL_TYPE:-flat}"
TRAY_RIM="${TRAY_RIM:-0.05}"; TRAY_E="${TRAY_E:-0.15}"; OBJ_HALF="${OBJ_HALF:-0.03}"
if [ "$FUNNEL_TYPE" = "tray" ]; then
  FUNNEL_MOUTH="${FUNNEL_MOUTH:-0.15}"
  if [ "$FUNNEL_MOUTH" = "0.15" ]; then
    FUNNEL_SDF="${FUNNEL_SDF:-$HOME/drone_payload_catch/models/x500_tray/model.sdf}"
  else
    FUNNEL_SDF="${FUNNEL_SDF:-$HOME/drone_payload_catch/models/x500_tray_big/model.sdf}"
  fi
  FUNNEL_EFF=$(python3 -c "print(round(float('$FUNNEL_MOUTH')-float('$OBJ_HALF'),3))")
  V_RETAIN=$(python3 -c "import math;print(round(math.sqrt(2*9.81*float('$TRAY_RIM'))/float('$TRAY_E'),2))")
  FUNNEL_EXTRA="funnel_mouth_radius:=$FUNNEL_MOUTH funnel_eff_radius:=$FUNNEL_EFF funnel_depth:=$TRAY_RIM funnel_restitution:=$TRAY_E v_retain:=$V_RETAIN funnel_mount_height:=0.21"
  if [ "${ATTACH:-false}" = "true" ]; then
    PAYLOAD_MODEL="$HOME/drone_payload_catch/models/payload_attached_100g/model.sdf"   # 编队：100g 挂载型
  else
    PAYLOAD_MODEL="$HOME/drone_payload_catch/models/payload_100g/model.sdf"
  fi
  if [ "${PAYLOAD_LOCK:-0}" = "1" ]; then
    LOCK_EXTRA="lock_to_b:=true lock_model_path:=$HOME/drone_payload_catch/models/payload_lock_100g/model.sdf"
  fi
elif [ "$FUNNEL_TYPE" = "cup" ]; then
  FUNNEL_MOUTH="${FUNNEL_MOUTH:-0.30}"
  FUNNEL_SDF="${FUNNEL_SDF:-$HOME/drone_payload_catch/models/x500_funnel_cup/model.sdf}"
elif [ "${FUNNEL_MOUTH:-0.20}" = "0.20" ]; then
  FUNNEL_MOUTH="0.20"
  FUNNEL_SDF="${FUNNEL_SDF:-$HOME/drone_payload_catch/models/x500_funnel/model.sdf}"
else
  FUNNEL_SDF="${FUNNEL_SDF:-$HOME/drone_payload_catch/models/x500_funnel_big/model.sdf}"
fi
if [ "$FUNNEL_TYPE" = "tray" ]; then
  :
elif [ "$FUNNEL_MOUTH" = "0.20" ]; then
  FUNNEL_EFF="0.14"; FUNNEL_EXTRA=""
else
  FUNNEL_EFF=$(python3 -c "print(round(float('$FUNNEL_MOUTH')-0.05,3))")
  FUNNEL_EXTRA="funnel_mouth_radius:=$FUNNEL_MOUTH funnel_eff_radius:=$FUNNEL_EFF"
fi
D="$HOME/payload_catch_m6_gui"; mkdir -p "$D"; rm -f "$D"/*.log
source "$HOME/drone_payload_catch/env.sh"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"   # WSLg 兼容兜底

echo "### cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'catch_stack_launch|a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
sleep 3
rm -f /dev/shm/fastrtps_*; rm -f "$HOME/px4_logs"/px4_*.log 2>/dev/null

echo "### [1] 启动 Gazebo GUI ($SITL_WORLD) —— 请看 Windows 桌面的 Gazebo 窗口"
gz sim -r "$SITL_WORLD" > "$D/gz.log" 2>&1 &
sleep 20
head -3 "$D/gz.log" 2>/dev/null

echo "### [2] agent + A(ENU 0,0) + B(x500_funnel_1 @ENU $B_POSE_ENU)"
MicroXRCEAgent udp4 -p 8888 > "$D/agent.log" 2>&1 &
sleep 3
cd "$PX4_DIR" || exit 1
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
IFS=',' read -r BPX BPY BPZ _ <<< "$B_POSE_ENU"
# A：常规 spawn
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
  PX4_GZ_MODEL_POSE="0,0,0,0,0,0" ./build/px4_sitl_default/bin/px4 -d -i 0 \
  < /dev/null > "$HOME/px4_logs/px4_0.log" 2>&1 &
sleep 12
# B：先 create 带漏斗的 x500_funnel，再让 PX4 attach
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req "sdf_filename: \"$FUNNEL_SDF\", name: \"x500_funnel_1\", allow_renaming: false, pose: { position: { x: ${BPX:-0}, y: ${BPY:-0}, z: ${BPZ:-0} } }" >/dev/null 2>&1
sleep 2
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_NAME=x500_funnel_1 \
  ./build/px4_sitl_default/bin/px4 -d -i 1 < /dev/null > "$HOME/px4_logs/px4_1.log" 2>&1 &
# GUI 相机不跟随（固定俯视全场视角）
# 如需跟随 B 看漏斗，可手动执行：
#   gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --req 'data: "x500_funnel_1"'
#   gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --req 'x: -3.0, y: -3.0, z: 2.0'

echo "### [3] 等双机 Ready"
for k in $(seq 1 40); do
  sleep 3
  r0=$(grep -ac "Ready for takeoff" "$HOME/px4_logs/px4_0.log" 2>/dev/null)
  r1=$(grep -ac "Ready for takeoff" "$HOME/px4_logs/px4_1.log" 2>/dev/null)
  [ "${r0:-0}" -ge 1 ] && [ "${r1:-0}" -ge 1 ] && { echo "  双机 READY ~$((k*3))s"; break; }
done
sleep 8

echo "### [4] 启动 M6 任务（≈20s 后释放载荷，注意看漏斗接住橙色方块；FORMATION_VEL=$FORMATION_VEL）"
timeout $((KEEP + 40)) ros2 launch payload_catch catch_stack_launch.py \
  a_hover:="[$A_HOVER]" b_standby:="[$B_STANDBY]" b_offset:="[$B_OFFSET]" \
  release_offset:="[$RELEASE_OFFSET]" payload_release_offset:=$RELEASE_Z \
  formation_vel:="[$FORMATION_VEL]" attach_to_a:=$ATTACH \
  model_path:=$PAYLOAD_MODEL $FUNNEL_EXTRA ${LOCK_EXTRA:-} > "$D/launch.log" 2>&1 &

for t in 10 20 25 30 40 60 80 100; do
  [ "$t" -ge "$KEEP" ] && break
  sleep 10
  ev=$(grep -aoE 'CLIMB done|WAIT_A done|TRANSLATE done|ALIGNED|/formation/start|FORMATION aligned|已挂载到 A|已与 A 分离|PAYLOAD RELEASED|STACK CAPTURED|LAND：发出着陆指令' "$D/launch.log" 2>/dev/null | tail -1)
  ph=$(grep -aoE 'B phase=[A-Z]+' "$D/launch.log" 2>/dev/null | tail -1)
  echo "  --- t+${t}s: $ph  $ev"
done
sleep 8

echo "### [5] 结果"
grep -aE "MODE=stack|formation_vel|CLIMB done|WAIT_A done|TRANSLATE done|ALIGNED|/formation/start|FORMATION aligned|已挂载到 A|已与 A 分离|DIVE plan|PAYLOAD RELEASED|STACK CAPTURED|LAND：" "$D/launch.log" | tail -12
echo "  px4_0: $(grep -aE 'Armed by|Takeoff detected' "$HOME/px4_logs/px4_0.log" | tail -2 | tr '\n' ' ')"
echo "  px4_1: $(grep -aE 'Armed by|Takeoff detected' "$HOME/px4_logs/px4_1.log" | tail -2 | tr '\n' ' ')"

echo "### [6] cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'catch_stack_launch|a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
echo "M6-GUI-DONE logs: $D"

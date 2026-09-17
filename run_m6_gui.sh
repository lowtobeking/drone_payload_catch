#!/usr/bin/env bash
# run_m6_gui.sh — M6 可视化仿真（Gazebo GUI 显示到 Windows 桌面，WSLg）。
# gz GUI + 2×PX4(1.16) + agent + A正上方释放 / B对正+温和下潜+刚性漏斗捕获。
# 用法: bash ~/drone_payload_catch/run_m6_gui.sh [观察秒数, 默认 110]
set +u
KEEP="${1:-110}"
A_HOVER="${A_HOVER:-0.0,0.0,-4.5}"
B_STANDBY="${B_STANDBY:-0.0,0.0,-3.5}"
B_OFFSET="${B_OFFSET:-5.0,0.0,0.0}"
B_POSE_ENU="${B_POSE_ENU:-0,5.0,0,0,0,0}"
RELEASE_OFFSET="${RELEASE_OFFSET:-0.0,0.0,0.15}"
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
  --req "sdf_filename: \"$HOME/drone_payload_catch/models/x500_funnel/model.sdf\", name: \"x500_funnel_1\", allow_renaming: false, pose: { position: { x: ${BPX:-0}, y: ${BPY:-0}, z: ${BPZ:-0} } }" >/dev/null 2>&1
sleep 2
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_NAME=x500_funnel_1 \
  ./build/px4_sitl_default/bin/px4 -d -i 1 < /dev/null > "$HOME/px4_logs/px4_1.log" 2>&1 &
# GUI 相机跟随 B（看漏斗与落点）
sleep 2
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'data: "x500_funnel_1"' >/dev/null 2>&1
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'x: -3.0, y: -3.0, z: 2.0' >/dev/null 2>&1

echo "### [3] 等双机 Ready"
for k in $(seq 1 40); do
  sleep 3
  r0=$(grep -ac "Ready for takeoff" "$HOME/px4_logs/px4_0.log" 2>/dev/null)
  r1=$(grep -ac "Ready for takeoff" "$HOME/px4_logs/px4_1.log" 2>/dev/null)
  [ "${r0:-0}" -ge 1 ] && [ "${r1:-0}" -ge 1 ] && { echo "  双机 READY ~$((k*3))s"; break; }
done
sleep 8

echo "### [4] 启动 M6 任务（≈20s 后释放载荷，注意看漏斗接住橙色方块）"
timeout $((KEEP + 40)) ros2 launch payload_catch catch_stack_launch.py \
  a_hover:="[$A_HOVER]" b_standby:="[$B_STANDBY]" b_offset:="[$B_OFFSET]" \
  release_offset:="[$RELEASE_OFFSET]" > "$D/launch.log" 2>&1 &

for t in 10 20 25 30 40 60 80 100; do
  [ "$t" -ge "$KEEP" ] && break
  sleep 10
  ev=$(grep -aoE 'CLIMB done|ALIGNED[^ ]*|PAYLOAD RELEASED|STACK CAPTURED' "$D/launch.log" 2>/dev/null | tail -1)
  ph=$(grep -aoE 'B phase=[A-Z]+' "$D/launch.log" 2>/dev/null | tail -1)
  echo "  --- t+${t}s: $ph  $ev"
done
sleep 8

echo "### [5] 结果"
grep -aE "MODE=stack|CLIMB done|ALIGNED|DIVE plan|PAYLOAD RELEASED|STACK CAPTURED" "$D/launch.log" | tail -8
echo "  px4_0: $(grep -aE 'Armed by|Takeoff detected' "$HOME/px4_logs/px4_0.log" | tail -2 | tr '\n' ' ')"
echo "  px4_1: $(grep -aE 'Armed by|Takeoff detected' "$HOME/px4_logs/px4_1.log" | tail -2 | tr '\n' ' ')"

echo "### [6] cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'catch_stack_launch|a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
echo "M6-GUI-DONE logs: $D"

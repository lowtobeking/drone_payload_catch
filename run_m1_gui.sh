#!/usr/bin/env bash
# run_m1_gui.sh — M1 可视化仿真（Gazebo GUI 显示到 Windows 桌面，WSLg）。
# gz GUI + 2×PX4(1.16) + agent + A悬停释放 / B会合捕获（acados MPC）。
# 用法: bash ~/drone_payload_catch/run_m1_gui.sh [观察秒数, 默认 110]
set +u
KEEP="${1:-110}"
A_HOVER="${A_HOVER:-0.0,0.0,-3.0}"
B_STANDBY="${B_STANDBY:-3.0,0.0,-3.0}"
B_OFFSET="${B_OFFSET:-3.0,0.0,0.0}"
B_POSE_ENU="${B_POSE_ENU:-0,3.0,0,0,0,0}"
CTRL="${CTRL:-mpc}"
D="$HOME/payload_catch_gui"; mkdir -p "$D"; rm -f "$D"/*.log
source "$HOME/drone_payload_catch/env.sh"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"   # WSLg 兼容兜底
python3 "$HOME/drone_payload_catch/tools/prebuild_mpc.py" 2>&1 | tail -1

echo "### [0] cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'catch_launch|a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
sleep 3
rm -f /dev/shm/fastrtps_*; rm -f "$HOME/px4_logs"/px4_*.log 2>/dev/null

echo "### [1] 启动 Gazebo GUI ($SITL_WORLD) —— 请到 Windows 桌面看 Gazebo 窗口"
gz sim -r "$SITL_WORLD" > "$D/gz.log" 2>&1 &
sleep 20
head -3 "$D/gz.log" 2>/dev/null

echo "### [2] agent + 2×PX4 (A=ENU 0,0  B=ENU $B_POSE_ENU)"
MicroXRCEAgent udp4 -p 8888 > "$D/agent.log" 2>&1 &
sleep 3
cd "$PX4_DIR" || exit 1
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
POSES=("0,0,0,0,0,0" "$B_POSE_ENU")
for i in "${!POSES[@]}"; do
  PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
    PX4_GZ_MODEL_POSE="${POSES[$i]}" \
    ./build/px4_sitl_default/bin/px4 -d -i "$i" < /dev/null > "$HOME/px4_logs/px4_$i.log" 2>&1 &
  echo "  px4 -i $i pose ${POSES[$i]}"
  [ "$i" -lt 1 ] && sleep 12
done

echo "### [3] 等双机 Ready"
for k in $(seq 1 40); do
  sleep 3
  r0=$(grep -ac "Ready for takeoff" "$HOME/px4_logs/px4_0.log" 2>/dev/null)
  r1=$(grep -ac "Ready for takeoff" "$HOME/px4_logs/px4_1.log" 2>/dev/null)
  [ "${r0:-0}" -ge 1 ] && [ "${r1:-0}" -ge 1 ] && { echo "  双机 READY ~$((k*3))s"; break; }
done
sleep 8

echo "### [4] 启动任务（控制器=$CTRL）—— 约 25s 后 A 释放载荷，B 会合捕获"
timeout $((KEEP + 40)) ros2 launch payload_catch catch_launch.py \
  controller:="$CTRL" a_hover:="[$A_HOVER]" b_standby:="[$B_STANDBY]" b_offset:="[$B_OFFSET]" \
  > "$D/launch.log" 2>&1 &

for t in 15 25 35 50 70; do
  [ "$t" -ge "$KEEP" ] && break
  sleep 15
  echo "  --- t+${t}s: $(grep -aoE 'B phase=[A-Z]+' "$D/launch.log" 2>/dev/null | tail -1)  $(grep -aoE '\*\*\* CAPTURED \*\*\* d=[0-9.]+m rel_v=[0-9.]+m/s' "$D/launch.log" 2>/dev/null | tail -1)"
done
sleep 10

echo "### [5] 结果"
grep -aE "PLAN ok|CAPTURED|payload released" "$D/launch.log" | tail -5
echo "  px4_0: $(grep -aE 'Armed by|Takeoff detected|Failsafe' "$HOME/px4_logs/px4_0.log" | tail -2 | tr '\n' ' ')"
echo "  px4_1: $(grep -aE 'Armed by|Takeoff detected|Failsafe' "$HOME/px4_logs/px4_1.log" | tail -2 | tr '\n' ' ')"

echo "### [6] cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'catch_launch|a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
echo "M1-GUI-DONE logs: $D"

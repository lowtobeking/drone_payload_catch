#!/usr/bin/env bash
# run_m6_sitl.sh — M6 垂直堆叠投放端到端：gz + 2×PX4(1.16) + agent + A正上方释放 / B对正+温和下潜软捕获。
# 场景：A 悬停 4.5m，B 从水平 5m 外起飞 → 飞到 A 正下方 3.5m → 水平速度归零/投影重合 → 释放。
# 用法: bash ~/drone_payload_catch/run_m6_sitl.sh [运行秒数, 默认 70]
set +u
RUN_S="${1:-70}"
A_HOVER="${A_HOVER:-0.0,0.0,-4.5}"          # A 悬停/释放点（世界 NED）
B_STANDBY="${B_STANDBY:-0.0,0.0,-3.5}"      # B 待命点（世界 NED，A 正下方）
B_OFFSET="${B_OFFSET:-5.0,0.0,0.0}"         # B 的 PX4 原点在世界 NED（= 地面 5m 间距）
B_POSE_ENU="${B_POSE_ENU:-0,5.0,0,0,0,0}"   # B 的 Gazebo 出生 ENU（y=5 → NED north=5）
RELEASE_OFFSET="${RELEASE_OFFSET:-0.0,0.0,0.15}"   # 载荷相对 A 的释放偏移（NED，向下 0.15m）
BASE="$HOME/payload_catch_ws"
D="$HOME/payload_catch_m6"; mkdir -p "$D"; rm -f "$D"/*.log
source "$HOME/drone_payload_catch/env.sh"

echo "### cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'catch_stack_launch|a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
sleep 3
rm -f /dev/shm/fastrtps_*; rm -f "$HOME/px4_logs"/px4_*.log 2>/dev/null

echo "### gz($SITL_WORLD) + agent"
gz sim -s -r "$SITL_WORLD" > "$D/gz.log" 2>&1 &
sleep 15
MicroXRCEAgent udp4 -p 8888 > "$D/agent.log" 2>&1 &
sleep 3

echo "### spawn 2×PX4 (A=ENU 0,0  B=ENU $B_POSE_ENU)"
cd "$PX4_DIR" || exit 1
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
POSES=("0,0,0,0,0,0" "$B_POSE_ENU")
IFS=',' read -r BPX BPY BPZ _ <<< "$B_POSE_ENU"
for i in "${!POSES[@]}"; do
  if [ "$i" -eq 1 ]; then
    # B：先手动 create 带刚性漏斗的 x500_funnel（实体名 x500_funnel_1），再让 PX4 attach
    echo "  create B 模型 x500_funnel_1 @ENU ($BPX,$BPY,$BPZ)"
    gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
      --req "sdf_filename: \"$HOME/drone_payload_catch/models/x500_funnel/model.sdf\", name: \"x500_funnel_1\", allow_renaming: false, pose: { position: { x: ${BPX:-0}, y: ${BPY:-0}, z: ${BPZ:-0} } }" >/dev/null 2>&1
    sleep 2
    PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_NAME=x500_funnel_1 \
      ./build/px4_sitl_default/bin/px4 -d -i "$i" < /dev/null > "$HOME/px4_logs/px4_$i.log" 2>&1 &
    echo "  px4 -i $i attach x500_funnel_1"
  else
    PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
      PX4_GZ_MODEL_POSE="${POSES[$i]}" \
      ./build/px4_sitl_default/bin/px4 -d -i "$i" < /dev/null > "$HOME/px4_logs/px4_$i.log" 2>&1 &
    echo "  px4 -i $i pose ${POSES[$i]}"
  fi
  [ "$i" -lt 1 ] && sleep 12
done
for k in $(seq 1 40); do
  sleep 3
  r0=$(grep -ac "Ready for takeoff" "$HOME/px4_logs/px4_0.log" 2>/dev/null)
  r1=$(grep -ac "Ready for takeoff" "$HOME/px4_logs/px4_1.log" 2>/dev/null)
  if [ "${r0:-0}" -ge 1 ] && [ "${r1:-0}" -ge 1 ]; then echo "  双机 READY ~$((k*3))s"; break; fi
done
echo "  就绪后再等 8s 让 EKF 稳定"; sleep 8

echo "### launch payload_catch M6（A/B/payload）"
timeout $((RUN_S + 40)) ros2 launch payload_catch catch_stack_launch.py \
  a_hover:="[$A_HOVER]" b_standby:="[$B_STANDBY]" b_offset:="[$B_OFFSET]" \
  release_offset:="[$RELEASE_OFFSET]" > "$D/launch.log" 2>&1 &
sleep "$RUN_S"

echo "### 结果"
echo "--- 关键事件 ---"
grep -aE "MODE=stack|CLIMB done|WAIT_A done|TRANSLATE done|ALIGNED|DIVE plan|PAYLOAD RELEASED|STACK CAPTURED|LAND：" "$D/launch.log" | tail -10
echo "--- B/载荷 末尾 ---"; grep -aE "B phase=" "$D/launch.log" | tail -4
echo "--- px4 events ---"; for i in 0 1; do echo "px4_$i:"; grep -aE "Ready for takeoff|Armed by|Takeoff detected|Failsafe" "$HOME/px4_logs/px4_$i.log" | tail -3; done

echo "### cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'catch_stack_launch|a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
echo "M6-SITL-DONE logs: $D"

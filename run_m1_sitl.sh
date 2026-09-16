#!/usr/bin/env bash
# run_m1_sitl.sh — M1 端到端：gz + 2×PX4(1.16) + agent + A悬停释放 / B会合捕获。
# 用法: bash ~/drone_payload_catch/run_m1_sitl.sh [运行秒数, 默认 60]
set +u
RUN_S="${1:-60}"
BASE="$HOME/payload_catch_ws"
D="$HOME/payload_catch_sitl"; mkdir -p "$D"; rm -f "$D"/*.log
source "$HOME/drone_payload_catch/env.sh"

echo "### cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
sleep 3
rm -f /dev/shm/fastrtps_*; rm -f "$HOME/px4_logs"/px4_*.log 2>/dev/null

echo "### gz($SITL_WORLD) + agent"
gz sim -s -r "$SITL_WORLD" > "$D/gz.log" 2>&1 &
sleep 15
MicroXRCEAgent udp4 -p 8888 > "$D/agent.log" 2>&1 &
sleep 3

echo "### spawn 2×PX4 (A=ENU 0,0  B=ENU 0,0.2)"
cd "$PX4_DIR" || exit 1
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
POSES=("0,0,0,0,0,0" "0,0.2,0,0,0,0")
for i in "${!POSES[@]}"; do
  PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
    PX4_GZ_MODEL_POSE="${POSES[$i]}" \
    ./build/px4_sitl_default/bin/px4 -d -i "$i" < /dev/null > "$HOME/px4_logs/px4_$i.log" 2>&1 &
  echo "  px4 -i $i pose ${POSES[$i]}"
  [ "$i" -lt 1 ] && sleep 5
done
for i in 0 1; do
  for k in $(seq 1 15); do
    sleep 2
    grep -aq "Ready for takeoff" "$HOME/px4_logs/px4_$i.log" && { echo "  px4_$i READY"; break; }
  done
done

echo "### launch payload_catch (A/B/payload)"
timeout $((RUN_S + 40)) ros2 launch payload_catch catch_launch.py > "$D/launch.log" 2>&1 &
sleep "$RUN_S"

echo "### 结果"
echo "--- CAPTURED? ---"; grep -aE "CAPTURED|payload released|rendezvous" "$D/launch.log" | tail -6
echo "--- B/载荷 世界位置(末尾) ---"; grep -aE "solve|CAPTURED" "$D/launch.log" | tail -4
echo "--- gz 模型列表 ---"; timeout 6 gz model --list 2>/dev/null | tail -4
echo "--- px4 events ---"; for i in 0 1; do echo "px4_$i:"; grep -aE "Ready for takeoff|Armed by|Takeoff detected|Preflight Fail" "$HOME/px4_logs/px4_$i.log" | tail -3; done

echo "### cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'catch_launch|a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
echo "M1-SITL-DONE logs: $D"

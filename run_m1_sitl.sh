#!/usr/bin/env bash
# run_m1_sitl.sh — M1 端到端：gz + 2×PX4(1.16) + agent + A悬停释放 / B会合捕获。
# 用法: bash ~/drone_payload_catch/run_m1_sitl.sh [运行秒数, 默认 60]
set +u
RUN_S="${1:-60}"
A_HOVER="${A_HOVER:-0.0,0.0,-3.0}"        # 世界 NED（launch a_hover）
B_STANDBY="${B_STANDBY:-0.2,0.0,-2.8}"    # 世界 NED（launch b_standby）
B_OFFSET="${B_OFFSET:-0.2,0.0,0.0}"       # B 的 PX4 原点在世界 NED（launch b_offset）
B_POSE_ENU="${B_POSE_ENU:-0,0.2,0,0,0,0}" # B 的 Gazebo 出生 ENU
BASE="$HOME/payload_catch_ws"
D="$HOME/payload_catch_sitl"; mkdir -p "$D"; rm -f "$D"/*.log
source "$HOME/drone_payload_catch/env.sh"
# 预热 acados MPC（消除 SITL 启动期编译尖峰）
python3 "$HOME/drone_payload_catch/tools/prebuild_mpc.py" 2>&1 | tail -1

echo "### cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
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
for i in "${!POSES[@]}"; do
  PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
    PX4_GZ_MODEL_POSE="${POSES[$i]}" \
    ./build/px4_sitl_default/bin/px4 -d -i "$i" < /dev/null > "$HOME/px4_logs/px4_$i.log" 2>&1 &
  echo "  px4 -i $i pose ${POSES[$i]}"
  [ "$i" -lt 1 ] && sleep 12
done
for k in $(seq 1 40); do
  sleep 3
  r0=$(grep -ac "Ready for takeoff" "$HOME/px4_logs/px4_0.log" 2>/dev/null)
  r1=$(grep -ac "Ready for takeoff" "$HOME/px4_logs/px4_1.log" 2>/dev/null)
  if [ "${r0:-0}" -ge 1 ] && [ "${r1:-0}" -ge 1 ]; then echo "  双机 READY ~$((k*3))s"; break; fi
done
echo "  就绪后再等 8s 让 EKF 稳定"; sleep 8

# 起飞前 live 自检（可选，PREFLIGHT=1）：只读探测运行中的 PX4 就绪（话题/EKF 有效）
if [ "${PREFLIGHT:-0}" = "1" ]; then
  echo "### 起飞前 live 自检（PREFLIGHT=1）"
  PF_EXTRA=""; [ "${PREFLIGHT_GPS:-0}" = "1" ] && PF_EXTRA="--gps"
  if ! python3 "$HOME/drone_payload_catch/tools/preflight_check.py" --live --logs $PF_EXTRA; then
    echo "❌ 起飞前 live 自检未通过 → 放弃本次起飞"
    for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent; do pkill -9 -f "$p" 2>/dev/null; done
    exit 1
  fi
  if [ "${PREFLIGHT_PARAMS:-0}" = "1" ]; then
    for inst in 0 1; do
      port=$((18570 + inst))
      echo "### 飞控参数检查 drone$inst (udpout:127.0.0.1:$port)"
      if ! python3 "$HOME/drone_payload_catch/tools/preflight_params.py" \
             --mavlink "udpout:127.0.0.1:$port" --profile sitl; then
        echo "❌ 飞控参数检查未通过 → 放弃本次起飞"
        for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent; do pkill -9 -f "$p" 2>/dev/null; done
        exit 1
      fi
    done
  fi
fi

echo "### launch payload_catch (A/B/payload)"
timeout $((RUN_S + 40)) ros2 launch payload_catch catch_launch.py controller:="${CTRL:-pd}" a_hover:="[$A_HOVER]" b_standby:="[$B_STANDBY]" b_offset:="[$B_OFFSET]" > "$D/launch.log" 2>&1 &
sleep "$RUN_S"

echo "### 结果"
echo "--- CAPTURED? ---"; grep -aE "CAPTURED|payload released|rendezvous" "$D/launch.log" | tail -6
echo "--- B/载荷 世界位置(末尾) ---"; grep -aE "solve|CAPTURED" "$D/launch.log" | tail -4
echo "--- gz 模型列表 ---"; timeout 6 gz model --list 2>/dev/null | tail -4
echo "--- px4 events ---"; for i in 0 1; do echo "px4_$i:"; grep -aE "Ready for takeoff|Armed by|Takeoff detected|Preflight Fail" "$HOME/px4_logs/px4_$i.log" | tail -3; done

echo "### cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'catch_launch|a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
echo "M1-SITL-DONE logs: $D"

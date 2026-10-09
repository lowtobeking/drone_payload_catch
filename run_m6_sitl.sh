#!/usr/bin/env bash
# run_m6_sitl.sh — M6 垂直堆叠投放端到端：gz + 2×PX4(1.16) + agent + A正上方释放 / B对正+温和下潜软捕获。
# 场景：A 悬停 4.5m，B 从水平 5m 外起飞 → 飞到 A 正下方 3.5m → 水平速度归零/投影重合 → 释放。
# 用法: bash ~/drone_payload_catch/run_m6_sitl.sh [运行秒数, 默认 70]
set +u
RUN_S="${1:-70}"
B_OFFSET="${B_OFFSET:-5.0,0.0,0.0}"         # B 的 PX4 原点在世界 NED（= 地面 5m 间距）
B_POSE_ENU="${B_POSE_ENU:-0,5.0,0,0,0,0}"   # B 的 Gazebo 出生 ENU（y=5 → NED north=5）

# 编队同速投放（M6-moving）：FORMATION_VEL 非零即启用
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
  RELEASE_Z="${RELEASE_Z:-0.45}"
  A_HOVER="${A_HOVER:-0.0,0.0,-5.0}"
  B_STANDBY="${B_STANDBY:-0.0,0.0,-3.3}"
fi
RELEASE_OFFSET="0.0,0.0,$RELEASE_Z"                # 载荷相对 A 的释放偏移（NED，向下）
LAUNCH_EXTRA="${LAUNCH_EXTRA:-}"                    # 额外 launch 参数（供扫描/试验覆盖）

# 研究特性预设：MODE=full 开启 握手+证书闸+CBF+intent+延迟鲁棒 CBF（默认 baseline 不变）
MODE="${MODE:-baseline}"
if [ "$MODE" = "full" ]; then
  COORD=handshake
  LAUNCH_EXTRA="release_gate_mode:=certificate keepout_mode:=cbf use_intent:=true keepout_delay_s:=0.05 $LAUNCH_EXTRA"
fi

# 协同释放握手：COORD=handshake 时 B 报就绪、A 作释放权威并 ack。
COORD="${COORD:-direct}"
[ "$COORD" = "handshake" ] && LAUNCH_EXTRA="coord_mode:=handshake $LAUNCH_EXTRA"
# 意图升级：WIND_EST="wx,wy,wz" 时 A 广播预测落点（含风漂移），B 对齐落点。
WIND_EST="${WIND_EST:-}"
[ -n "$WIND_EST" ] && LAUNCH_EXTRA="use_intent:=true wind_est:=[$WIND_EST] $LAUNCH_EXTRA"
# 安全层：SAFETY_FLOOR 给定 σ 下限 (m)，keep-out = min_ab_gap + 2σ。
SAFETY_FLOOR="${SAFETY_FLOOR:-}"
[ -n "$SAFETY_FLOOR" ] && LAUNCH_EXTRA="rel_sigma_floor:=$SAFETY_FLOOR $LAUNCH_EXTRA"
# PX4 风估计：PX4_WIND=1 时 A 订阅 /fmu/out/wind，并用其预测落点（同时开 use_intent）。
PX4_WIND="${PX4_WIND:-0}"
[ "$PX4_WIND" = "1" ] && LAUNCH_EXTRA="use_px4_wind:=true use_intent:=true $LAUNCH_EXTRA"
# 控制：ZEM 终端导引增益。
ZEM="${ZEM:-}"
[ -n "$ZEM" ] && LAUNCH_EXTRA="zem_gain:=$ZEM $LAUNCH_EXTRA"

# 主动保持（B 侧锁扣）：PAYLOAD_LOCK=1 时用带 DetachableJoint 的载荷模型，
# 捕获后 b_node 请求把载荷锁到 B 的漏斗 link。
PAYLOAD_LOCK="${PAYLOAD_LOCK:-0}"
LOCK_EXTRA=""
if [ "$PAYLOAD_LOCK" = "1" ]; then
  # 初始仍用普通载荷（自由落体）；捕获时 payload_node 在 B 漏斗处重生成带关节的载荷
  LOCK_EXTRA="lock_to_b:=true lock_model_path:=$HOME/drone_payload_catch/models/payload_lock/model.sdf"
fi

# 末端能力：FUNNEL_TYPE=flat(默认)/cup(空心导向锥杯)/tray(圆形托盘)；FUNNEL_MOUTH 指定口/盘半径。
#   flat + 0.20   → x500_funnel（实心平顶盘）
#   flat + >0.20  → x500_funnel_big（大平顶盘）
#   cup           → x500_funnel_cup（空心锥杯：导向+保持）
#   tray          → x500_tray（圆形托盘：围边+泡棉缓冲），载荷 6cm/100g
#   例：FUNNEL_TYPE=cup  bash run_m6_sitl.sh 70
#       FUNNEL_TYPE=tray bash run_m6_sitl.sh 70
FUNNEL_TYPE="${FUNNEL_TYPE:-flat}"
TRAY_RIM="${TRAY_RIM:-0.05}"    # 围边高 (m)：有效围挡高度
TRAY_E="${TRAY_E:-0.15}"        # 泡棉恢复系数（低回弹）
OBJ_HALF="${OBJ_HALF:-0.03}"    # 物块半宽（6cm 方块 → 0.03）
if [ "$FUNNEL_TYPE" = "tray" ]; then
  # FUNNEL_MOUTH = 盘内半径（默认 0.15 → 内径 30cm）；eff_r = 盘内半径 − 物半宽
  FUNNEL_MOUTH="${FUNNEL_MOUTH:-0.15}"
  if [ "$FUNNEL_MOUTH" = "0.15" ]; then
    FUNNEL_SDF="${FUNNEL_SDF:-$HOME/drone_payload_catch/models/x500_tray/model.sdf}"
  else
    FUNNEL_SDF="${FUNNEL_SDF:-$HOME/drone_payload_catch/models/x500_tray_big/model.sdf}"
  fi
  FUNNEL_DEPTH="$TRAY_RIM"
  FUNNEL_REST="$TRAY_E"
  FUNNEL_EFF=$(python3 -c "print(round(float('$FUNNEL_MOUTH')-float('$OBJ_HALF'),3))")
  V_RETAIN=$(python3 -c "import math;print(round(math.sqrt(2*9.81*float('$FUNNEL_DEPTH'))/float('$FUNNEL_REST'),2))")
  FUNNEL_EXTRA="funnel_mouth_radius:=$FUNNEL_MOUTH funnel_eff_radius:=$FUNNEL_EFF funnel_depth:=$FUNNEL_DEPTH funnel_restitution:=$FUNNEL_REST v_retain:=$V_RETAIN funnel_mount_height:=0.21"
  if [ "$ATTACH" = "true" ]; then
    PAYLOAD_MODEL="$HOME/drone_payload_catch/models/payload_attached_100g/model.sdf"   # 编队：100g 挂载型
  else
    PAYLOAD_MODEL="$HOME/drone_payload_catch/models/payload_100g/model.sdf"
  fi
  # 主动锁扣：托盘也保留 funnel_link 名，直接复用（100g 版）
  if [ "$PAYLOAD_LOCK" = "1" ]; then
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
  :    # tray 分支已装配 FUNNEL_EXTRA / FUNNEL_EFF
elif [ "$FUNNEL_MOUTH" = "0.20" ]; then
  FUNNEL_EFF="0.14"
  FUNNEL_EXTRA=""
else
  FUNNEL_EFF=$(python3 -c "print(round(float('$FUNNEL_MOUTH')-0.05,3))")
  FUNNEL_EXTRA="funnel_mouth_radius:=$FUNNEL_MOUTH funnel_eff_radius:=$FUNNEL_EFF"
fi
BASE="$HOME/payload_catch_ws"
D="$HOME/payload_catch_m6"; mkdir -p "$D"; rm -f "$D"/*.log
source "$HOME/drone_payload_catch/env.sh"

# SITL 风场：WIND>0 时生成带风 world（载荷 enable_wind=true → 受风漂移）
if [ "${WIND:-0}" != "0" ]; then
  python3 "$HOME/drone_payload_catch/tools/make_wind_world.py" \
    --src "$SITL_WORLD" --out /tmp/default_wind.sdf \
    --wind "$WIND" --dir-deg "${WIND_DIR:-0}" >&2
  export SITL_WORLD=/tmp/default_wind.sdf
fi

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
    echo "  create B 模型 x500_funnel_1 @ENU ($BPX,$BPY,$BPZ)  sdf=$FUNNEL_SDF"
    gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
      --req "sdf_filename: \"$FUNNEL_SDF\", name: \"x500_funnel_1\", allow_renaming: false, pose: { position: { x: ${BPX:-0}, y: ${BPY:-0}, z: ${BPZ:-0} } }" >/dev/null 2>&1
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
  release_offset:="[$RELEASE_OFFSET]" payload_release_offset:=$RELEASE_Z \
  formation_vel:="[$FORMATION_VEL]" attach_to_a:=$ATTACH model_path:=$PAYLOAD_MODEL \
  $FUNNEL_EXTRA $LOCK_EXTRA $LAUNCH_EXTRA > "$D/launch.log" 2>&1 &
echo "  end-effector: type=$FUNNEL_TYPE mouth(盘内半径)=$FUNNEL_MOUTH eff=$FUNNEL_EFF v_retain=${V_RETAIN:-4.04}" >&2
sleep "$RUN_S"

echo "### 结果"
echo "--- 关键事件 ---"
grep -aE "MODE=stack|formation_vel|CLIMB done|WAIT_A done|TRANSLATE done|ALIGNED|/formation/start|FORMATION aligned|已与 A 分离|DIVE plan|PAYLOAD RELEASED|STACK CAPTURED|LAND：" "$D/launch.log" | tail -12
echo "--- B/载荷 末尾 ---"; grep -aE "B phase=" "$D/launch.log" | tail -4
echo "--- px4 events ---"; for i in 0 1; do echo "px4_$i:"; grep -aE "Ready for takeoff|Armed by|Takeoff detected|Failsafe" "$HOME/px4_logs/px4_$i.log" | tail -3; done

echo "### cleanup"
for p in 'px4 -d -i' 'gz sim' MicroXRCEAgent 'catch_stack_launch|a_node|b_node|payload_node'; do pkill -9 -f "$p" 2>/dev/null; done
echo "M6-SITL-DONE logs: $D"

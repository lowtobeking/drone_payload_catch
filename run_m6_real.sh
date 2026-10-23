#!/usr/bin/env bash
# run_m6_real.sh —— **真机** M6 任务的启动封装（无 Gazebo / 无 PX4 SITL）。
#
# 前置（详见 report/real_hardware_bringup.md）：
#   1. 两机 PX4 已上电、uXRCE-DDS agent 已连（A 在 /fmu/...，B 在 /px4_1/fmu/...，按你实际命名空间改）；
#   2. RTK（或 UWB/动捕）已发布到 A_RTK / B_RTK 话题；
#   3. **安全**：安全员就位、RC 接管可用、围栏/高度已设；
#   4. 标定参数（杆臂/原点是 world_offset）已回填 config 的 real: 段或本脚本变量。
#
# 用法:
#   A_RTK=/rtk/a B_RTK=/rtk/b B_OFFSET="5.0,0.0,0.0" bash run_m6_real.sh
set +u
source "$HOME/drone_payload_catch/env.sh"

# ── 可覆盖参数 ──
A_HOVER="${A_HOVER:-0.0,0.0,-4.5}"       # A 悬停点（相对 B 的 PX4 世界 NED）
B_STANDBY="${B_STANDBY:-0.0,0.0,-3.5}"   # B 待命点
B_OFFSET="${B_OFFSET:-0.0,0.0,0.0}"      # B 的 PX4 原点在 relnav 世界系（标定）
A_RTK="${A_RTK:-/rtk/a}"
B_RTK="${B_RTK:-/rtk/b}"
RTK_TYPE="${RTK_TYPE:-navsatfix}"        # navsatfix | array
# 必须实测：天线→PX4 机体定位原点，禁止复用旧的天线→托盘杆臂。
[ -n "${A_LEVER:-}" ] && [ -n "${B_LEVER:-}" ] \
  || { echo "拒绝启动：必须填写实测 A_LEVER/B_LEVER（天线到机体原点）"; exit 1; }
FUNNEL_MOUTH="${FUNNEL_MOUTH:-0.15}"     # 托盘内半径（30cm 盘 → 0.15）
COORD="${COORD:-handshake}"

echo "!! 真机启动前请确认：安全员就位 / RC 可接管 / 围栏高度已设 / 电机桨已装好"

# 强制门禁：不可跳过，必须分别读取两台飞控；节点还会复核实时健康。
if [ "${SKIP_PREFLIGHT:-0}" = "1" ]; then
  echo "拒绝启动：SKIP_PREFLIGHT 已禁用"; exit 1
fi
[ -n "${PARAM_MAVLINK_A:-}" ] && [ -n "${PARAM_MAVLINK_B:-}" ] \
  || { echo "拒绝启动：必须设置 PARAM_MAVLINK_A 和 PARAM_MAVLINK_B"; exit 1; }
PARAM_PROFILE="${PARAM_PROFILE:-outdoor}"
PF_ARGS=(--live --rc --param-profile "$PARAM_PROFILE"
         --param-mavlink-a "$PARAM_MAVLINK_A" --param-mavlink-b "$PARAM_MAVLINK_B")
PF_GPS=true
case "$PARAM_PROFILE" in
  outdoor) PF_ARGS+=(--gps) ;;
  indoor) PF_GPS=false ;;
  *) echo "真机必须选择 outdoor 或 indoor 参数策略"; exit 1 ;;
esac

echo "   A_HOVER=$A_HOVER B_STANDBY=$B_STANDBY B_OFFSET=$B_OFFSET"
echo "   RTK A=$A_RTK B=$B_RTK type=$RTK_TYPE  lever A=$A_LEVER B=$B_LEVER"
read -r -p "确认全部就绪？输入 yes 继续：" _ans
[ "$_ans" = "yes" ] || { echo "已取消"; exit 1; }
PF_REPORT=$(mktemp /tmp/payload_catch_preflight.XXXXXX.json) || exit 1
python3 "$HOME/drone_payload_catch/tools/preflight_check.py" "${PF_ARGS[@]}" \
  --write-approval "$PF_REPORT" \
  || { echo "自检未通过，拒绝起飞"; exit 1; }


ros2 launch payload_catch catch_real_launch.py \
  servo_config:="${SERVO_CONFIG:-$HOME/drone_payload_catch/config/payload_servo.yaml}" \
  preflight_report:="$PF_REPORT" preflight_require_gps:="$PF_GPS" \
  a_hover:="[$A_HOVER]" b_standby:="[$B_STANDBY]" b_offset:="[$B_OFFSET]" \
  source:=rtk a_rtk_topic:="$A_RTK" b_rtk_topic:="$B_RTK" rtk_type:="$RTK_TYPE" \
  a_lever:="[$A_LEVER]" b_lever:="[$B_LEVER]" \
  funnel_mouth_radius:=$FUNNEL_MOUTH funnel_eff_radius:=0.12 \
  coord_mode:=$COORD contact_detect:=true

# 说明：
#  · 载荷状态 /payload/state 需由真实感知节点发布（视觉/动捕/UWB tag 或弹道预测）；
#    缺失真实感知或 /payload/hardware_ready 时拒飞，不用解析预测冒充观测。
#  · 捕获由【接触检测】触发（加速度尖峰/速度反转/外部 /payload/contact 开关）。

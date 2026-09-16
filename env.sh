#!/usr/bin/env bash
# drone_payload_catch 环境（本机已验证可用的 PX4-1.16 SITL）
#   source ~/drone_payload_catch/env.sh
#
# 本机验证结论（2026-09-16，详见 report/env_bringup.md）：
#   · 必须用 PX4-1.16 树（~/drone_package_20260908/PX4-Autopilot-1.16）。
#     $HOME/PX4-Autopilot(main) 的 x500 外层模型 IMU 无噪声 → Gyro STALE，已弃用。
#   · px4_msgs 必须 @ v1.16.2（话题带 _v1 后缀：vehicle_status_v1；local_position 无后缀）。
#   · gz 用自建"全系统 world"$SITL_WORLD（含 Imu/NavSat/Sensors 等 system 插件 + 球坐标）。
#   · MicroXRCEAgent 基于 Fast-DDS → RMW 必须 rmw_fastrtps_cpp（本机全局是 cyclonedds）。

BASE="${PAYLOAD_CATCH_BASE:-$HOME/drone_package_20260908}"

# ── PX4 / SITL ────────────────────────────────────────────────────────────
export PX4_DIR="${PX4_DIR:-$BASE/PX4-Autopilot-1.16}"
export SITL_WORLD="${SITL_WORLD:-$BASE/gz_overrides/worlds/default.sdf}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"

# ── acados ────────────────────────────────────────────────────────────────
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$BASE/acados}"
export ACADOS_INSTALL_DIR="$ACADOS_SOURCE_DIR"
export LD_LIBRARY_PATH="/usr/local/lib:$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"

# ── ROS 2 ─────────────────────────────────────────────────────────────────
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION_OVERRIDE:-rmw_fastrtps_cpp}"
set +u
source /opt/ros/jazzy/setup.bash
# px4_msgs v1.16.2（参考项目工作区）
[ -f "$BASE/ros2_ws/install/setup.bash" ] && source "$BASE/ros2_ws/install/setup.bash"
# 本项目工作区（若已构建）
[ -f "$HOME/payload_catch_ws/install/setup.bash" ] && source "$HOME/payload_catch_ws/install/setup.bash"


echo "[env] PX4_DIR=$PX4_DIR"
echo "[env] SITL_WORLD=$SITL_WORLD"
echo "[env] ACADOS_SOURCE_DIR=$ACADOS_SOURCE_DIR"
echo "[env] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

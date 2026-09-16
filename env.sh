#!/usr/bin/env bash
# drone_payload_catch 环境变量（source 后使用）
#   source ~/drone_payload_catch/env.sh
#
# 依本机实际路径填写；不同机器（仿真机/Jetson/RPi）路径不同，故不写死进 ~/.bashrc。

# ── acados ────────────────────────────────────────────────────────────────
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/drone_package_20260908/acados}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"

# ── ROS 2 ─────────────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

# MicroXRCEAgent 基于 Fast-DDS 编译，ROS 2 侧必须用 fastrtps（否则互相看不到话题，
# 症状是"零报错但一个话题都没有"）。本机全局 RMW 是 cyclonedds，故在此覆盖。
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION_OVERRIDE:-rmw_fastrtps_cpp}"

# ── PX4 SITL（PX4 main：先起 gz，再起 px4 -i N 挂模型）─────────────────────
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
if [ -f "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh" ]; then
    source "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"
fi
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4001}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"

# ── 项目自身 ──────────────────────────────────────────────────────────────
export PAYLOAD_CATCH_DIR="${PAYLOAD_CATCH_DIR:-$HOME/drone_payload_catch}"
echo "[env] ACADOS_SOURCE_DIR=$ACADOS_SOURCE_DIR"
echo "[env] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[env] PX4_DIR=$PX4_DIR  PX4_SIM_MODEL=$PX4_SIM_MODEL"

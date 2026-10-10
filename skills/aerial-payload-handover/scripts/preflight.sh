#!/usr/bin/env bash
# 起飞前自检（preflight）：配置/依赖/PX4/RMW/ROS；--sitl 严格，--live 探测运行中的 PX4。
# 用法：bash preflight.sh [--sitl] [--live]
set +u
REPO="${DRONE_PAYLOAD_CATCH:-$HOME/drone_payload_catch}"; cd "$REPO" || exit 1
exec python3 tools/preflight_check.py "$@"

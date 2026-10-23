#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --worker ]]; then
    cd "$PAYLOAD_CATCH_TEST_ROOT"
    source ./env.sh >/dev/null
    export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
    export DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
    export QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 GZ_IP=127.0.0.1
    export GZ_PARTITION=payload_a_solo_20261019
    export LD_LIBRARY_PATH="$HOME/drone_package_20260908/agent_install/lib:$LD_LIBRARY_PATH"
    exec python3 tools/run_a_solo_gazebo.py --worker
fi
[[ "$EUID" -eq 0 ]]
ip link set lo up
exec runuser -u "$PAYLOAD_CATCH_TEST_USER" -- env \
    PAYLOAD_CATCH_HOST_NETNS="$PAYLOAD_CATCH_HOST_NETNS" \
    PAYLOAD_CATCH_TEST_ROOT="$PAYLOAD_CATCH_TEST_ROOT" \
    ROS_LOCALHOST_ONLY=1 bash "$0" --worker

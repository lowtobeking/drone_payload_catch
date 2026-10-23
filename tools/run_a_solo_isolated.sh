#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --worker ]]; then
    cd "$PAYLOAD_CATCH_TEST_ROOT"
    if [[ -f ./env.sh ]]; then
        source ./env.sh >/dev/null
    else
        set +u
        source /opt/ros/humble/setup.bash
        : "${PX4_MSGS_SETUP:?explicit independent message overlay required}"
        source "$PX4_MSGS_SETUP"
        source "$PAYLOAD_CATCH_TEST_ROOT/../ws/install/setup.bash"
        set -u
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    fi
    export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
    exec python3 tools/check_a_solo_ros.py
fi
[[ "$EUID" -eq 0 ]]
ip link set lo up
exec runuser -u "$PAYLOAD_CATCH_TEST_USER" -- env \
    PAYLOAD_CATCH_HOST_NETNS="$PAYLOAD_CATCH_HOST_NETNS" \
    PAYLOAD_CATCH_TEST_ROOT="$PAYLOAD_CATCH_TEST_ROOT" \
    PX4_MSGS_SETUP="${PX4_MSGS_SETUP:-}" ROS_LOCALHOST_ONLY=1 bash "$0" --worker

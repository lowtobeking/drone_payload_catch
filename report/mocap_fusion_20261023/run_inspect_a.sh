#!/bin/bash
set -eo pipefail
cd -- "$1"
source /home/nvidia/payload_catch_deploy/current/src/deploy/runtime_env.sh
source /home/nvidia/payload_catch_deploy/current/ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$1/ros_logs"
exec /usr/bin/timeout --signal=INT --kill-after=5s 85s /usr/bin/python3 inspect_a.py

#!/bin/bash
set -eo pipefail
cd -- "$1"
source /home/nvidia/payload_catch_deploy/current/src/deploy/runtime_env.sh
source /home/nvidia/payload_catch_deploy/current/ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_LOG_DIR="$2/ros_logs"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 receive_ground_runtime_a.py "$2"

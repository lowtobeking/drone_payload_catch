#!/bin/bash
set -eo pipefail
cd -- "$1"
source /home/nvidia/payload_catch_deploy/current/src/deploy/runtime_env.sh
source /home/nvidia/payload_catch_deploy/current/ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=191
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$2/ros_logs"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 diagnostic_runtime.py "$2"

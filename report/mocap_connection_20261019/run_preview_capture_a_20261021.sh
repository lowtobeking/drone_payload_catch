#!/bin/bash
set -eo pipefail
cd -- "$1"
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=191
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$2/ros_logs"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 preview_runtime.py "$2"

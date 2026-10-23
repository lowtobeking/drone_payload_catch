#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/payload_catch/managed_20261016/px4_msgs_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 /opt/payload_catch/managed_20261016/ground_status.py

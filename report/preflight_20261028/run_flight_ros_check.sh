#!/usr/bin/env bash
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/caolihao/drone_package_20260908/ros2_ws/install/setup.bash
exec /usr/bin/python3 report/preflight_20261028/check_flight_ros.py

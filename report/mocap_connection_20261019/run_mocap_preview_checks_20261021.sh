#!/bin/bash
set -eo pipefail
cd /home/caolihao/drone_payload_catch
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=191
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/home/caolihao/drone_payload_catch/report/mocap_connection_20261019/preview_ros_logs
python3 tools/test_mocap_transform.py >report/mocap_connection_20261019/preview_unit_tests.log 2>&1
timeout --signal=TERM --kill-after=3s 20s python3 tools/check_mocap_preview_ros.py >report/mocap_connection_20261019/preview_ros_check.log 2>&1

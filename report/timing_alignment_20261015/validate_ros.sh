#!/usr/bin/env bash
set -e
cd /home/caolihao/drone_payload_catch
source ./env.sh >/dev/null
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=97
python3 tools/check_servo_launch.py
python3 tools/check_fleet_launch.py
python3 tools/check_servo_ros.py
python3 tools/check_fleet_ros.py

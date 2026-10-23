#!/bin/bash
set -eo pipefail
cd -- /home/caolihao/drone_payload_catch
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=192
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="$PWD/report/mocap_fusion_20261023/ros_logs"
source /opt/ros/humble/setup.bash
source /home/caolihao/drone_package_20260908/ros2_ws/install/setup.bash
/usr/bin/python3 tools/test_mocap_odometry.py > report/mocap_fusion_20261023/unit_validation.log 2>&1
/usr/bin/python3 tools/test_mocap_receive_ground.py > report/mocap_fusion_20261023/receive_ground_validation.log 2>&1
/usr/bin/python3 tools/test_mocap_heading_candidate.py > report/mocap_fusion_20261023/heading_candidate_validation.log 2>&1
/usr/bin/python3 tools/test_mocap_transform.py > report/mocap_fusion_20261023/transform_regression.log 2>&1
/usr/bin/python3 tools/check_mocap_odometry_ros.py > report/mocap_fusion_20261023/ros_validation.log 2>&1
/bin/cat report/mocap_fusion_20261023/unit_validation.log report/mocap_fusion_20261023/receive_ground_validation.log report/mocap_fusion_20261023/heading_candidate_validation.log report/mocap_fusion_20261023/transform_regression.log report/mocap_fusion_20261023/ros_validation.json

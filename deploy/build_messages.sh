#!/bin/bash
set -eo pipefail
prefix=/opt/payload_catch/managed_20261016
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
ws="$prefix/px4_msgs_ws"
[[ ! -e "$ws/src/px4_msgs" ]]
git clone --no-hardlinks /home/nvidia/ros2_ws/src/px4_msgs "$ws/src/px4_msgs"
git -C "$ws/src/px4_msgs" checkout --detach 392e831c1f659429ca83902e66820d7094591410
git -C "$ws/src/px4_msgs" remote remove origin
cd "$ws"
colcon build --executor sequential --symlink-install --packages-select px4_msgs
source "$ws/install/setup.bash"
python3 -c 'from px4_msgs.msg import VehicleStatus; print("Independent px4_msgs built successfully")'

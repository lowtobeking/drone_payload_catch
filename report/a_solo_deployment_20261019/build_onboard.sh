#!/bin/bash
set -eo pipefail
release="$1"
[[ "$release" =~ ^[0-9a-f]{64}$ ]]
here="$(cd "$(dirname "$0")" && pwd)"
unset PYTHONHOME PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH VIRTUAL_ENV
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
source /opt/payload_catch/managed_20261016/px4_msgs_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 "$here/installer.py" install --bundle "$here/payload-catch-a-solo.tar.gz" --role a --base /home/nvidia/payload_catch_deploy
src="/home/nvidia/payload_catch_deploy/releases/$release/src"
[[ -f "$src/../BUILT" ]]
/usr/bin/python3 "$here/installer.py" verify "$src"

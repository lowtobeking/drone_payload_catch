#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
set -u
python3 /home/nvidia/a_action_fleet_deploy_20261105.py install \
  --bundle /home/nvidia/a_action_20261105_source.tar.gz --role a \
  2>&1 | tee /home/nvidia/a_action_20261105_install.log

#!/usr/bin/env bash
set -eo pipefail
source /home/nvidia/payload_catch_deploy/current/src/deploy/runtime_env.sh
source /home/nvidia/payload_catch_deploy/current/ws/install/setup.bash
/usr/bin/python3 /home/nvidia/payload_catch_a_fusion_20261104/check_odom_graph.py
exec /usr/bin/python3 /home/nvidia/payload_catch_a_fusion_20261104/check_odom_time.py

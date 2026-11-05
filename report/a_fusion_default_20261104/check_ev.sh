#!/usr/bin/env bash
set -eo pipefail
source /home/nvidia/payload_catch_deploy/current/src/deploy/runtime_env.sh
source /home/nvidia/payload_catch_deploy/current/ws/install/setup.bash
exec /usr/bin/python3 /home/nvidia/payload_catch_a_fusion_20261104/check_ev_flags.py

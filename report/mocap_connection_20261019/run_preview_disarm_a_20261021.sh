#!/bin/bash
set -eo pipefail
cd -- "$1"
source /home/nvidia/payload_catch_deploy/current/src/deploy/runtime_env.sh
source /home/nvidia/payload_catch_deploy/current/ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/timeout --signal=TERM --kill-after=3s 15s /usr/bin/python3 preview_disarm_check.py

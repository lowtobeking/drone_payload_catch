#!/usr/bin/env bash
set -eo pipefail
release="$(realpath /home/nvidia/payload_catch_deploy/current)"
source "$release/src/deploy/runtime_env.sh"
source "$release/ws/install/setup.bash"
export PYTHONPATH="$release/src:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 /tmp/a_solo_height_capture_20261104.py --seconds 90 --height 0.2

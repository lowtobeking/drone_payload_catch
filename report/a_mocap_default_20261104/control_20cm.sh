#!/usr/bin/env bash
set -eo pipefail
base=/home/nvidia/payload_catch_deploy
root="$base/current/src"
export FLEET_HOST_PROFILE="$base/host/runtime.yaml"
source "$root/deploy/runtime_env.sh"
source "$base/current/ws/install/setup.bash"
export PYTHONPATH="$root:$PYTHONPATH"
exec /usr/bin/python3 "$root/tools/a_solo_control.py" "$@"

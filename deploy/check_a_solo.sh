#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
base="${FLEET_BASE:-/home/nvidia/payload_catch_deploy}"
source "$root/deploy/runtime_env.sh"
source "$root/../ws/install/setup.bash"
export PYTHONPATH="$root:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 "$root/tools/a_solo_entry.py" check --base "$base"

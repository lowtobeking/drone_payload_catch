#!/usr/bin/env bash
# Ordinary-user read-only infrastructure check, no task nodes or actuator writes.
set -eo pipefail
runtime_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$runtime_root/deploy/runtime_env.sh"
[[ -n "${FLEET_HOST_PROFILE:-}" ]] || { echo 'Host runtime profile required' >&2; exit 1; }
exec /usr/bin/python3 "$runtime_root/tools/jetson_runtime.py" check --profile "$FLEET_HOST_PROFILE" "$@"

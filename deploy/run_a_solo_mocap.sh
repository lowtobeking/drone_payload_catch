#!/usr/bin/env bash
# A-only mission uses the boot-managed mocap bridge.
set -eo pipefail
if [[ $# != 3 ]]; then
  echo 'Usage: bash deploy/run_a_solo_mocap.sh A_SOLO_CONFIG RECEIPT HOST_PROFILE' >&2
  exit 2
fi
config="$(realpath "$1")"
receipt="$(realpath -m "$2")"
profile="$(realpath "$3")"
root="$(cd "$(dirname "$0")/.." && pwd)"
if ! systemctl is-active --quiet payload-catch-mocap-a.service; then
  echo 'A mocap bridge service is not active' >&2
  exit 1
fi
export FLEET_HOST_PROFILE="$profile"
source "$root/deploy/runtime_env.sh"
source "$root/../ws/install/setup.bash"
export PYTHONPATH="$root:$PYTHONPATH"
/usr/bin/python3 "$root/tools/a_solo_preflight.py" \
  --config "$config" --host-profile "$profile" --mavlink passive:eno1 --out "$receipt"
exec /bin/bash "$root/deploy/run_a_solo.sh" "$config" "$receipt" "$profile" execute

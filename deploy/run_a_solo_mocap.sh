#!/usr/bin/env bash
# A-only: stream mocap into PX4, check the fused state, then run the existing solo flight.
set -euo pipefail
if [[ $# != 6 ]]; then
  echo 'Usage: bash deploy/run_a_solo_mocap.sh A_SOLO_CONFIG RECEIPT HOST_PROFILE VRPN_READER MAVLINK_ENDPOINT LOG_DIR' >&2
  exit 2
fi
config="$(realpath "$1")"
receipt="$(realpath -m "$2")"
profile="$(realpath "$3")"
reader="$(realpath "$4")"
mavlink="$5"
log_dir="$(realpath -m "$6")"
fleet_base="${FLEET_BASE:-$HOME/payload_catch_deploy}"
release="$(realpath "$fleet_base/current")"
root="$release/src"
export FLEET_HOST_PROFILE="$profile"
export PYTHONDONTWRITEBYTECODE=1
source "$root/deploy/runtime_env.sh"
source "$release/ws/install/setup.bash"
export PYTHONPATH="$root"

/usr/bin/python3 - "$config" <<'PY'
import sys
from payload_catch.solo_validation import load_config
if load_config(sys.argv[1]).positioning != "external_local":
    raise SystemExit("A solo configuration must select external_local")
PY
mkdir -p -- "$log_dir"
bash "$root/deploy/run_a_mocap_flight.sh" \
  "$root/config/mocap_a.solo_flight.yaml" "$reader" "$log_dir" &
bridge_pid=$!
cleanup() {
  trap - EXIT INT TERM
  kill "$bridge_pid" 2>/dev/null || true
  wait "$bridge_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
/usr/bin/python3 "$root/tools/a_solo_preflight.py" \
  --config "$config" --host-profile "$profile" --mavlink "$mavlink" --out "$receipt"
if ! kill -0 "$bridge_pid" 2>/dev/null; then
  echo "mocap stream stopped; see $log_dir" >&2
  exit 1
fi
bash "$root/deploy/run_a_solo.sh" "$config" "$receipt" "$profile" execute

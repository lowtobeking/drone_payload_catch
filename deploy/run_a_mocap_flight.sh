#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 3 ]]; then
  echo "usage: run_a_mocap_flight.sh CONFIG READER LOG_DIR" >&2
  exit 2
fi
cfg="$1"
reader="$2"
log_dir="$3"
root="$(cd "$(dirname "$0")/.." && pwd)"
source "$root/deploy/runtime_env.sh"
source "$root/../ws/install/setup.bash"
mkdir -p -- "$log_dir/source"
export PYTHONPATH="$root"
export PYTHONDONTWRITEBYTECODE=1

/usr/bin/python3 -m payload_catch.mocap_source_node \
  --reader "$reader" --bind-ip 192.168.1.5 --seconds 0 --continuous-ground \
  --log-dir "$log_dir/source" > "$log_dir/source.log" 2>&1 &
source_pid=$!
/usr/bin/python3 -m payload_catch.mocap_odometry_node \
  --config "$cfg" --send-to-px4 --flight-receive-time --seconds 0 --continuous-flight \
  > "$log_dir/bridge.log" 2>&1 &
bridge_pid=$!
cleanup() {
  trap - EXIT INT TERM
  kill "$source_pid" "$bridge_pid" 2>/dev/null || true
  wait "$source_pid" "$bridge_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
if wait -n "$source_pid" "$bridge_pid"; then
  echo "mocap source or bridge exited unexpectedly" >&2
else
  echo "mocap source or bridge failed; see $log_dir" >&2
fi
exit 1

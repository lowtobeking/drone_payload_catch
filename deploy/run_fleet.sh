#!/usr/bin/env bash
# Starts only the installed role. Never installs dependencies, starts agents or arms directly.
set -eo pipefail
if [[ $# != 3 || ! "$1" =~ ^[ab]$ ]]; then
  echo 'Usage: bash deploy/run_fleet.sh a|b /absolute/fleet.yaml /absolute/preflight.json'; exit 2
fi
role="$1"
fleet_config="$(realpath "$2")"
approval="$(realpath -m "$3")"
fleet_base="${FLEET_BASE:-$HOME/payload_catch_deploy}"
exec 9>"$fleet_base/mission.lock"
flock -n 9 || { echo 'Mission already running'; exit 1; }
release="$(realpath "$fleet_base/current")"
[[ "$(cat "$fleet_base/ROLE")" == "$role" && "$(cat "$release/ROLE")" == "$role" ]] || { echo 'Role mismatch'; exit 1; }
[[ -f "$release/BUILT" ]] || { echo 'Release not built'; exit 1; }
[[ -z "${ROS_DISTRO:-}" || "$ROS_DISTRO" == humble ]] || { echo 'Start in a fresh Humble shell'; exit 1; }
source "$release/src/deploy/runtime_env.sh"
source "$release/ws/install/setup.bash"
release_id="$(python3 "$release/src/tools/fleet_deploy.py" verify "$release/src")"
network_dir="$fleet_base/network-$role"
python3 "$release/src/tools/fleet_configure.py" "$fleet_config" --role "$role" --out "$network_dir" --check-host
source "$network_dir/network.env"
if [[ -n "${FLEET_HOST_PROFILE:-}" ]]; then
  python3 "$release/src/tools/jetson_runtime.py" check --profile "$FLEET_HOST_PROFILE"     --fleet "$fleet_config" --role "$role"
fi
# Missing/expired receipt keeps both task nodes blocked; no automatic preflight bypass.
exec ros2 launch payload_catch "catch_real_${role}_launch.py" \
  fleet_config:="$fleet_config" release_id:="$release_id" preflight_report:="$approval" \
  servo_external:="${FLEET_SERVO_EXTERNAL:-false}"

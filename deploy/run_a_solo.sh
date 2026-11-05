#!/usr/bin/env bash
# Read-only by default; explicit execute still waits for separate operator START.
set -eo pipefail
if [[ $# != 4 || ! "$4" =~ ^(check|execute)$ ]]; then
    echo 'Usage: bash deploy/run_a_solo.sh /absolute/a_solo.yaml /absolute/receipt.json /absolute/host.runtime.yaml check|execute'
    exit 2
fi
config="$(realpath "$1")"
receipt="$(realpath -m "$2")"
profile="$(realpath "$3")"
mode="$4"
if [[ "$mode" == execute ]] && ! systemctl is-active --quiet payload-catch-mocap-a.service; then
    echo 'A mocap bridge service is not active' >&2
    exit 1
fi
fleet_base="${FLEET_BASE:-$HOME/payload_catch_deploy}"
exec 9>"$fleet_base/mission.lock"
flock -n 9 || { echo 'A/dual mission already running'; exit 1; }
release="$(realpath "$fleet_base/current")"
[[ "$(cat "$fleet_base/ROLE")" == a && "$(cat "$release/ROLE")" == a ]] || { echo 'A role required'; exit 1; }
[[ -f "$release/BUILT" ]] || { echo 'Release not built'; exit 1; }
[[ -z "${ROS_DISTRO:-}" || "$ROS_DISTRO" == humble ]] || { echo 'Fresh Humble shell required'; exit 1; }
source "$release/src/deploy/runtime_env.sh"
source "$release/ws/install/setup.bash"
python3 "$release/src/tools/fleet_deploy.py" verify "$release/src"
sudo -n /usr/bin/env ROS_DOMAIN_ID="$ROS_DOMAIN_ID" RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
    ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" /usr/bin/python3 "$release/src/tools/a_solo_runtime.py" --profile "$profile"
execute=false
[[ "$mode" == execute ]] && execute=true
exec ros2 launch payload_catch a_solo_validation_launch.py \
    config:="$config" receipt:="$receipt" source_root:="$release/src" host_profile:="$profile" execute:="$execute"

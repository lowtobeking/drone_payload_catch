#!/usr/bin/env bash
# Source only. Adopt verified existing infrastructure without changing services.
_payload_runtime_env() {
  local runtime_root profile temporary
  runtime_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
  profile="${FLEET_HOST_PROFILE:-${FLEET_BASE:-$HOME/payload_catch_deploy}/host/runtime.yaml}"
  if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
    echo 'Open a fresh shell: ROS Humble required' >&2; return 1
  fi
  # Do not inherit Python/ROS overlays or virtualenv executables from the old project.
  unset PYTHONHOME PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH VIRTUAL_ENV
  export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  source /opt/ros/humble/setup.bash || return 1
  if [[ -f "$profile" ]]; then
    temporary="$(mktemp)" || return 1
    if ! /usr/bin/python3 "$runtime_root/tools/jetson_runtime.py" environment --profile "$profile" --out "$temporary"; then
      rm -f -- "$temporary"; return 1
    fi
    source "$temporary"
    rm -f -- "$temporary"
  elif [[ -n "${FLEET_HOST_PROFILE:-}" ]]; then
    echo 'Explicit host runtime profile missing' >&2; return 1
  fi
  if [[ -z "${PX4_MSGS_SETUP:-}" || ! -f "$PX4_MSGS_SETUP" ]]; then
    echo 'Set PX4_MSGS_SETUP to the validated px4_msgs overlay, or install host/runtime.yaml' >&2; return 1
  fi
  source "$PX4_MSGS_SETUP" || return 1
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
}
if _payload_runtime_env; then
  unset -f _payload_runtime_env
else
  unset -f _payload_runtime_env
  return 1
fi

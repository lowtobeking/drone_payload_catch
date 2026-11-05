#!/usr/bin/env bash
set -eo pipefail
root=/home/nvidia/payload_catch_deploy/current/src
base=/home/nvidia/payload_catch_a_fusion_20261104
reader=/home/nvidia/payload_catch_mocap_fusion_20261026/continuous_ground_de7d9c91c69d703b/mocap_vrpn_reader
timeout --signal=TERM --kill-after=5s 50s /bin/bash "$root/deploy/run_a_mocap_flight.sh" "$root/config/mocap_a.solo_flight.yaml" "$reader" "$base/in_window" > "$base/in_window_launcher.log" 2>&1 &
bridge=$!
cleanup() {
  kill "$bridge" 2>/dev/null || true
  wait "$bridge" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
sleep 8
/bin/bash "$base/check_odom.sh"
/bin/bash "$base/check_ev.sh"

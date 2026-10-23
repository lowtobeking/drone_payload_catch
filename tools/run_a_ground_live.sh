#!/bin/bash
# Run on Jetson A. Real telemetry, no flight approval, no hardware actuator writes.
set -eo pipefail
base="${FLEET_BASE:-$HOME/payload_catch_deploy}"
source_root="$(realpath "$base/current/src")"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ "$(cat "$base/ROLE")" == a && -f "$source_root/../BUILT" ]]
source "$source_root/deploy/runtime_env.sh"
source "$source_root/../ws/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
unset PYTHONOPTIMIZE
# Refuse startup on a different/unhealthy communication runtime.
/usr/bin/python3 "$source_root/tools/jetson_runtime.py" check --profile "$FLEET_HOST_PROFILE" >/dev/null
report="$(mktemp -d "$base/ground/reports/run-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
echo "GROUND_REPORT=$report"
/usr/bin/timeout --signal=TERM --kill-after=5s 80s /usr/bin/python3 "$script_dir/run_a_ground_live.py" \
  --base "$base" --out "$report/result.json" "$@" 2>&1 | tee "$report/run.log"
echo "GROUND_CHECK_COMPLETE=$report"
#!/usr/bin/env bash
# M6 垂直投放蒙特卡洛。用法: bash scripts/run_m6_mc.sh [scenario] [n]
set +u
REPO="${DRONE_PAYLOAD_CATCH:-$HOME/drone_payload_catch}"; cd "$REPO" || exit 1
SCEN="${1:-M6_stack_tray}"; N="${2:-300}"
echo "== M6 蒙特卡洛: $SCEN n=$N =="
exec python3 tools/stack_run.py --scenario "$SCEN" --mc "$N"

#!/usr/bin/env bash
# SITL: M6-moving + 大托盘（40cm）。用法: bash scripts/run_sitl_tray.sh [vel=0.5] [lock=0]
# 需先 source env.sh（本脚本会自动 source）。无窗口。
set +u
REPO="${DRONE_PAYLOAD_CATCH:-$HOME/drone_payload_catch}"; cd "$REPO" || exit 1
VEL="${1:-0.5}"; LOCK="${2:-0}"
source "$REPO/env.sh" >/dev/null 2>&1 || true
echo "== SITL M6-moving vel=${VEL} 大托盘 PAYLOAD_LOCK=${LOCK} =="
FORMATION_VEL="${VEL},0.0,0.0" FUNNEL_TYPE=tray FUNNEL_MOUTH=0.20 PAYLOAD_LOCK="$LOCK" \
  bash run_m6_sitl.sh 75

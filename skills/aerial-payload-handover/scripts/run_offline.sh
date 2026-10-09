#!/usr/bin/env bash
# 离线体检（M1–M4 会合类）。用法: bash scripts/run_offline.sh [--all|--scenario NAME|--plot|--mc]
set +u
REPO="${DRONE_PAYLOAD_CATCH:-$HOME/drone_payload_catch}"; cd "$REPO" || { echo "repo 未找到: $REPO"; exit 1; }
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/drone_package_20260908/acados}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
[ $# -eq 0 ] && set -- --all
exec python3 tools/offline_run.py "$@"

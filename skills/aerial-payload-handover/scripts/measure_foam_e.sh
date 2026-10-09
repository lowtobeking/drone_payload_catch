#!/usr/bin/env bash
# 泡棉恢复系数 e 换算。用法: bash scripts/measure_foam_e.sh --h-drop 1.0 --rebounds 0.020 0.024 0.018
set +u
REPO="${DRONE_PAYLOAD_CATCH:-$HOME/drone_payload_catch}"; cd "$REPO" || exit 1
exec python3 tools/foam_drop_test.py "$@"

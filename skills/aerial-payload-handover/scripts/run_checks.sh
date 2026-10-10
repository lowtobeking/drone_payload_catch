#!/usr/bin/env bash
# 一键离线自检（纯模块 + tools/test_*.py + acados + offline --all）
# 用法：bash run_checks.sh [--quick]
set +u
REPO="${DRONE_PAYLOAD_CATCH:-$HOME/drone_payload_catch}"; cd "$REPO" || exit 1
exec bash tools/run_checks.sh "$@"

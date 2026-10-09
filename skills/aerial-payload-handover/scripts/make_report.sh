#!/usr/bin/env bash
# 生成 Word 仿真报告。用法: bash scripts/make_report.sh [--out /path/x.docx] [--fast]
set +u
REPO="${DRONE_PAYLOAD_CATCH:-$HOME/drone_payload_catch}"; cd "$REPO" || exit 1
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/drone_package_20260908/acados}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
exec python3 tools/make_docx_report.py "$@"

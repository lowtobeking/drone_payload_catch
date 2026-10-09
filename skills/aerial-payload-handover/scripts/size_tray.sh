#!/usr/bin/env bash
# 圆形托盘选型。用法: bash scripts/size_tray.sh --diameter 0.30 --rim 0.05 --e 0.15 --obj 0.06 --gap 1.0
set +u
REPO="${DRONE_PAYLOAD_CATCH:-$HOME/drone_payload_catch}"; cd "$REPO" || exit 1
exec python3 tools/tray_sizing.py "$@"

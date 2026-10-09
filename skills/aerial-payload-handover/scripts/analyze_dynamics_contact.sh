#!/usr/bin/env bash
# 动力学限幅 + 接触冲击（写 report/dynamics_contact.md）
set +u
REPO="${DRONE_PAYLOAD_CATCH:-$HOME/drone_payload_catch}"; cd "$REPO" || exit 1
exec python3 tools/dynamics_contact.py "$@"

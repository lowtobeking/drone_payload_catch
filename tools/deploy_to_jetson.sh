#!/usr/bin/env bash
# Old mpc_control uploader retired. Explicit staged payload_catch deployment only.
set -e
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$root/tools/fleet_deploy.py" "$@"

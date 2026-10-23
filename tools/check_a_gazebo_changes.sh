#!/usr/bin/env bash
set -euo pipefail
cd /home/caolihao/drone_payload_catch
source ./env.sh >/dev/null
export PYTHONDONTWRITEBYTECODE=1
python3 tools/test_a_solo.py
python3 tools/test_fcu_params.py
python3 tools/test_takeoff_gate.py
git diff --check

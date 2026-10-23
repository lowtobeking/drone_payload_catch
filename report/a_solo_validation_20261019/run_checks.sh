#!/usr/bin/env bash
set -euo pipefail
cd /home/caolihao/drone_payload_catch
export PYTHONDONTWRITEBYTECODE=1
python3 report/a_solo_validation_20261019/parse_sources.py
python3 tools/test_a_solo.py -v > report/a_solo_validation_20261019/unit.log 2>&1
source ./env.sh > report/a_solo_validation_20261019/environment.log
python3 tools/test_takeoff_gate.py -v > report/a_solo_validation_20261019/dual_gate_regression.log 2>&1
python3 tools/test_safety_runtime.py -v > report/a_solo_validation_20261019/safety_regression.log 2>&1
python3 tools/test_timed_release.py -v > report/a_solo_validation_20261019/release_regression.log 2>&1

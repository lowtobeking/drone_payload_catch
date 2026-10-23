#!/bin/bash
# Onboard acceptance only: no real flight/PWM commands.
set -eo pipefail
release="$1"
[[ "$release" =~ ^[0-9a-f]{64}$ ]]
src="/home/nvidia/payload_catch_deploy/releases/$release/src"
export FLEET_HOST_PROFILE=/home/nvidia/payload_catch_deploy/host/runtime.yaml
source "$src/deploy/runtime_env.sh"
source "$src/../ws/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
cd "$src"
python3 tools/fleet_deploy.py verify "$src"
bash -n deploy/run_a_solo.sh
python3 tools/test_a_solo.py
python3 tools/test_a_solo_parameters.py
python3 tools/test_fcu_params.py
python3 tools/test_takeoff_gate.py
python3 tools/a_solo_preflight.py --help
python3 tools/a_solo_control.py --help
ros2 launch payload_catch a_solo_validation_launch.py --show-args
sudo -n /usr/bin/env PX4_MSGS_SETUP="$PX4_MSGS_SETUP" /usr/bin/python3 tools/run_a_solo_isolated.py
python3 tools/fleet_deploy.py verify "$src"
python3 /home/nvidia/payload_catch_a_solo_20261019/mark_validated.py "$release"

#!/usr/bin/env bash
set -eo pipefail
cd /home/caolihao/drone_payload_catch
source ./env.sh > report/a_solo_validation_20261019/environment_build.log
export PYTHONDONTWRITEBYTECODE=1
colcon --log-base report/a_solo_validation_20261019/colcon_log build \
    --base-paths /home/caolihao/drone_payload_catch --packages-select payload_catch \
    --build-base report/a_solo_validation_20261019/build \
    --install-base report/a_solo_validation_20261019/install \
    > report/a_solo_validation_20261019/build.log 2>&1
source report/a_solo_validation_20261019/install/setup.bash
ros2 launch payload_catch a_solo_validation_launch.py --show-args \
    > report/a_solo_validation_20261019/launch_args.log 2>&1
python3 tools/a_solo_preflight.py --help > report/a_solo_validation_20261019/preflight_help.log
python3 tools/a_solo_control.py --help > report/a_solo_validation_20261019/control_help.log

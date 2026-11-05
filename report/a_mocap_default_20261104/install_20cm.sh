#!/usr/bin/env bash
set -eo pipefail
base=/home/nvidia/payload_catch_deploy
stage=/home/nvidia/payload_catch_a_deploy_20261104_mocap_20cm
release_id=345ee81922950034228d6493a23a3cb64b18e91265cd401fcbd78a5dce070f6f
reader=/home/nvidia/payload_catch_mocap_fusion_20261026/continuous_ground_de7d9c91c69d703b/mocap_vrpn_reader
[[ "$(tr -d '\000' </proc/device-tree/serial-number)" == 1612425621719 ]]
[[ "$(readlink -f "$base/current")" == "$base/releases/bd31da18d716454db89180402baa2cce4f99354804c72c250af2f3aaaf06d10d" ]]
[[ "$(systemctl is-active payload-catch-agent.service)" == active ]]
[[ "$(sha256sum "$reader" | cut -d ' ' -f 1)" == da05ac0e45403b19f8125e02ac6e4e3dfaf7aa2344ca911ea6bca71ecde0e387 ]]
source "$base/current/src/deploy/runtime_env.sh"
source "$base/current/ws/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 "$stage/fleet_deploy.py" install --bundle "$stage/source_20cm.tar.gz" --role a --base "$base"
/usr/bin/python3 "$stage/fleet_deploy.py" activate --ground-confirmed --release "$release_id" --base "$base"
mkdir -p "$base/host" "$base/mocap_logs"
install -m 0755 "$reader" "$base/host/mocap_vrpn_reader"
[[ "$(sha256sum "$base/host/mocap_vrpn_reader" | cut -d ' ' -f 1)" == da05ac0e45403b19f8125e02ac6e4e3dfaf7aa2344ca911ea6bca71ecde0e387 ]]
sudo -n install -m 0644 "$base/current/src/deploy/payload-catch-mocap-a.service" /etc/systemd/system/payload-catch-mocap-a.service
sudo -n systemctl daemon-reload
sudo -n systemctl enable --now payload-catch-mocap-a.service

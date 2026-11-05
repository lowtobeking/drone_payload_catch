#!/usr/bin/env bash
set -eo pipefail
base=/home/nvidia/payload_catch_deploy
stage=/home/nvidia/payload_catch_a_deploy_20261104_mocap_default
release_id=bd31da18d716454db89180402baa2cce4f99354804c72c250af2f3aaaf06d10d
reader=/home/nvidia/payload_catch_mocap_fusion_20261026/continuous_ground_de7d9c91c69d703b/mocap_vrpn_reader
[[ "$(tr -d '\000' </proc/device-tree/serial-number)" == 1612425621719 ]]
[[ "$(readlink -f "$base/current")" == "$base/releases/4a63f47ae3e50d819c5a940cd19ca42c75125ddc3e4c9f91f9864075b8de8d2f" ]]
[[ "$(systemctl is-active payload-catch-agent.service)" == active ]]
[[ "$(sha256sum "$reader" | cut -d ' ' -f 1)" == da05ac0e45403b19f8125e02ac6e4e3dfaf7aa2344ca911ea6bca71ecde0e387 ]]
source "$base/current/src/deploy/runtime_env.sh"
source "$base/current/ws/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 "$stage/fleet_deploy.py" install --bundle "$stage/source.tar.gz" --role a --base "$base"
/usr/bin/python3 "$stage/fleet_deploy.py" activate --ground-confirmed --release "$release_id" --base "$base"
mkdir -p "$base/host" "$base/mocap_logs"
install -m 0755 "$reader" "$base/host/mocap_vrpn_reader"
[[ "$(sha256sum "$base/host/mocap_vrpn_reader" | cut -d ' ' -f 1)" == da05ac0e45403b19f8125e02ac6e4e3dfaf7aa2344ca911ea6bca71ecde0e387 ]]
sudo -n install -m 0644 "$base/current/src/deploy/payload-catch-mocap-a.service" /etc/systemd/system/payload-catch-mocap-a.service
sudo -n systemctl daemon-reload
sudo -n systemctl enable --now payload-catch-mocap-a.service

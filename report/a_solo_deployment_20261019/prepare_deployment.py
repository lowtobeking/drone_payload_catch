#!/usr/bin/env python3
"""Prepare an integrity-checked A release locally; never contacts hardware."""
from pathlib import Path
import ast
import hashlib
import json
import shutil
import subprocess
import sys
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root))
from tools.fleet_deploy import bundle,read_bundle,make_manifest,snapshot
report=Path(__file__).resolve().parent
target=report/"payload-catch-a-solo.tar.gz"
if target.exists() and read_bundle(target)[1] != make_manifest(snapshot(root)):
    archive=report/"bundles"
    archive.mkdir(exist_ok=True)
    old_release=read_bundle(target)[1]["release_id"]
    target.rename(archive/(old_release+".tar.gz"))
if target.exists():
    files,manifest=read_bundle(target)
    release=manifest["release_id"]
else:
    release=bundle(root,target)
    files,manifest=read_bundle(target)
assert manifest["release_id"]==release
required=("payload_catch/solo_validation.py","payload_catch/solo_validation_node.py",
          "tools/a_solo_preflight.py","tools/a_solo_control.py","tools/a_solo_runtime.py",
          "launch/a_solo_validation_launch.py","deploy/run_a_solo.sh","config/a_solo.example.yaml")
for name in required:
    assert name in files,name
    if name.endswith(".py"):
        ast.parse(files[name].decode("utf-8"),name)
assert b"a_solo_validation = payload_catch.solo_validation_node:main" in files["setup.py"]
for name in ("tools/fleet_deploy.py","tools/run_a_solo_isolated.py","tools/check_a_solo_ros.py"):
    ast.parse(files[name].decode("utf-8"),name)
shutil.copy2(root/"tools/fleet_deploy.py",report/"installer.py")
shell="""#!/bin/bash
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
"""
(report/"validate_onboard.sh").write_text(shell,encoding="utf-8",newline="\n")
subprocess.run(["/bin/bash","-n",str(report/"validate_onboard.sh")],cwd=root,timeout=10,check=True)
metadata=dict(status="prepared_locally_not_uploaded",role="a",release=release,
              bundle=str(target),bundle_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
              files=len(files),required_entries=list(required),remote_changed=False,
              target_base="/home/nvidia/payload_catch_deploy",
              last_known_release="13c1e49da415339aa236e19547c63a809de15e55235584917a700e964dd1702a",
              blocker="SSH 192.168.155.186 timed out; Windows Ethernet disconnected; new A IP required",
              activation_requires="Live identity, disarmed/landed state and task/service compatibility checks")
(report/"deployment_status.json").write_text(json.dumps(metadata,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
(report/"release_id.txt").write_text(release+"\n",encoding="utf-8")
print(json.dumps(metadata,indent=2,ensure_ascii=False),flush=True)

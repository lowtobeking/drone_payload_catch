"""File-only deployment selection. Does not launch ROS, tests or actuators."""
from pathlib import Path
import hashlib
import json
import os
import platform
import re
import socket
import sys
import yaml

BASE=Path("/home/nvidia/payload_catch_deploy")
WORK=Path("/home/nvidia/payload_catch_a_solo_20261019")
RELEASE=sys.argv[1]
assert re.fullmatch(r"[0-9a-f]{64}",RELEASE)
SOURCE=BASE/"releases"/RELEASE/"src"
sys.path.insert(0,str(SOURCE))
from tools.fleet_deploy import atomic_link,ensure_stopped,running_lock,select,verify

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def check_identity():
    assert socket.gethostname()=="nvidia-desktop"
    assert platform.machine()=="aarch64"
    assert Path("/proc/device-tree/serial-number").read_text().strip("\0\n")=="1612425621719"
    assert (BASE/"ROLE").read_text().strip()=="a"

def restore(snapshot):
    assert (BASE/"current").resolve().name==RELEASE
    select(BASE,Path(snapshot["current"]).name)
    atomic_link(BASE,"current",Path(snapshot["current"]))
    atomic_link(BASE,"previous",Path(snapshot["previous"]))

check_identity()
with running_lock(BASE),ensure_stopped(BASE):
    if "--rollback" in sys.argv:
        snapshot=json.loads((WORK/"deploy_only_before.json").read_text(encoding="utf-8"))
        restore(snapshot)
        print(json.dumps(dict(rolled_back=True,current=str((BASE/"current").resolve())),indent=2))
        raise SystemExit(0)
    target=select(BASE,RELEASE)
    assert (BASE/"current").resolve().name=="13c1e49da415339aa236e19547c63a809de15e55235584917a700e964dd1702a"
    snapshot=dict(current=str((BASE/"current").resolve(strict=True)),
                  previous=str((BASE/"previous").resolve(strict=True)),
                  host_profile_sha256=sha(BASE/"host/runtime.yaml"),
                  boot_config_sha256=sha(Path("/boot/extlinux/extlinux.conf")),
                  created_config=False)
    state=WORK/"deploy_only_before.json"
    assert not state.exists(),"Deployment snapshot already exists; inspect before retry"
    state.write_text(json.dumps(snapshot,indent=2)+"\n",encoding="utf-8")
    config=BASE/"a_solo.yaml"
    try:
        if not config.exists():
            cfg=yaml.safe_load((SOURCE/"config/a_solo.example.yaml").read_text(encoding="utf-8"))
            cfg["hostname"]=socket.gethostname()
            cfg["board_serial"]="1612425621719"
            assert cfg["site_checked"] is False and cfg["payload_removed"] is False
            with config.open("x",encoding="utf-8") as handle:
                yaml.safe_dump(cfg,handle,sort_keys=False)
            snapshot["created_config"]=True
            snapshot["config_sha256"]=sha(config)
            state.write_text(json.dumps(snapshot,indent=2)+"\n",encoding="utf-8")
        atomic_link(BASE,"previous",Path(snapshot["current"]))
        atomic_link(BASE,"current",target)
        assert (BASE/"current").resolve(strict=True)==target
        assert (BASE/"previous").resolve(strict=True)==Path(snapshot["current"])
        assert verify(SOURCE)["release_id"]==RELEASE
        assert sha(BASE/"host/runtime.yaml")==snapshot["host_profile_sha256"]
        assert sha(Path("/boot/extlinux/extlinux.conf"))==snapshot["boot_config_sha256"]
        result=dict(status="deployed_not_run",release=RELEASE,current=str(target),
                    previous=snapshot["current"],config=str(config),
                    arm_build_passed=True,onboard_ros_acceptance_passed=False,
                    acceptance_note="Previous isolated ROS test failed on report directory permission; not rerun by user request.",
                    real_flight_started=False,pwm_started=False,tests_started_this_step=False,
                    services_modified=False,autostart_configured=False,
                    startup_gate_unchanged=True)
        (WORK/"deployment_result.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
        print(json.dumps(result,indent=2),flush=True)
    except BaseException:
        if (BASE/"current").resolve()==target:
            restore(snapshot)
        else:
            atomic_link(BASE,"previous",Path(snapshot["previous"]))
        raise

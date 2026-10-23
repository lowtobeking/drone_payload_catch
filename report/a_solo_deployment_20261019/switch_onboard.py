"""Select only a validated release; leave flight and PWM stopped."""
from pathlib import Path
import hashlib
import json
import os
import re
import socket
import subprocess
import sys
import yaml
base=Path("/home/nvidia/payload_catch_deploy")
work=Path(__file__).resolve().parent
expected_old="13c1e49da415339aa236e19547c63a809de15e55235584917a700e964dd1702a"
release=sys.argv[1]
assert re.fullmatch(r"[0-9a-f]{64}",release)
source=base/"releases"/release/"src"
sys.path.insert(0,str(source))
from tools.fleet_deploy import running_lock,ensure_stopped,select,atomic_link,verify
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def run(args,name,timeout=40,env=None):
    p=subprocess.run(args,cwd=source,timeout=timeout,capture_output=True,text=True,encoding="utf-8",errors="replace",env=env)
    (work/(name+".stdout.log")).write_text(p.stdout,encoding="utf-8")
    (work/(name+".stderr.log")).write_text(p.stderr,encoding="utf-8")
    if p.returncode:raise RuntimeError(name+" failed: "+p.stdout+"\n"+p.stderr)
    return p.stdout
def guard():
    run(["sudo","-n","/bin/bash","/opt/payload_catch/managed_20261016/ground_env.sh"],"switch_ground_guard")
def restore(snapshot):
    guard()
    assert (base/"current").resolve().name==release,"Selected release has changed"
    atomic_link(base,"current",Path(snapshot["current"]))
    atomic_link(base,"previous",Path(snapshot["previous"]))
    for name in snapshot.get("created_files",[]):
        path=base/name
        expected=snapshot["created_files"][name]
        if path.is_file() and sha(path)==expected:
            path.rename(work/(name+".rolled_back"))
    print("Rolled back to",snapshot["current"],flush=True)
with running_lock(base),ensure_stopped(base):
    if "--rollback" in sys.argv:
        restore(json.loads((work/"before_switch.json").read_text(encoding="utf-8")))
        raise SystemExit(0)
    target=select(base,release)
    assert (base/"current").resolve().name==expected_old
    assert (work/"onboard_validated.json").is_file(),"Onboard validation marker missing"
    marker=json.loads((work/"onboard_validated.json").read_text(encoding="utf-8"))
    assert marker["release"]==release and marker["passed"] is True
    guard()
    snapshot=dict(current=str((base/"current").resolve(strict=True)),
        previous=str((base/"previous").resolve(strict=True)),created_files={},
        host_profile_sha256=sha(base/"host/runtime.yaml"),
        boot_config_sha256=sha(Path("/boot/extlinux/extlinux.conf")))
    config=base/"a_solo.yaml"
    if not config.exists():
        cfg=yaml.safe_load((source/"config/a_solo.example.yaml").read_text(encoding="utf-8"))
        cfg["hostname"]=socket.gethostname()
        cfg["board_serial"]=Path("/proc/device-tree/serial-number").read_text().strip("\0\n")
        assert cfg["site_checked"] is False and cfg["payload_removed"] is False
        config.write_text(yaml.safe_dump(cfg,sort_keys=False),encoding="utf-8")
        snapshot["created_files"]["a_solo.yaml"]=sha(config)
    (work/"before_switch.json").write_text(json.dumps(snapshot,indent=2)+"\n",encoding="utf-8")
    try:
        atomic_link(base,"previous",Path(snapshot["current"]))
        atomic_link(base,"current",target)
        env=dict(os.environ,FLEET_HOST_PROFILE=str(base/"host/runtime.yaml"),PYTHONDONTWRITEBYTECODE="1")
        output=run(["/bin/bash",str(source/"deploy/check_jetson.sh"),"--out",str(work/"managed_check.json")],
                   "managed_check",env=env)
        check=json.loads((work/"managed_check.json").read_text(encoding="utf-8"))
        assert check["passed"] is True and check["actuator_commands_sent"]==0
        assert sha(base/"host/runtime.yaml")==snapshot["host_profile_sha256"]
        assert sha(Path("/boot/extlinux/extlinux.conf"))==snapshot["boot_config_sha256"]
        assert verify(source)["release_id"]==release
        result=dict(passed=True,current=str(target),previous=snapshot["current"],
                    real_flight_started=False,pwm_started=False,services_modified=False,
                    config=str(config),config_requires_site_and_payload_confirmation=True)
        (work/"deployment_result.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
        print(json.dumps(result,indent=2),flush=True)
    except BaseException:
        restore(snapshot)
        raise

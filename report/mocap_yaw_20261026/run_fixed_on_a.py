"""Run bounded disarmed fixed-reference fusion, FC-only reboot and verified parameter rollback."""
import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys

WORK = Path(__file__).resolve().parent
PROJECT = WORK.parents[1]
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
SSH = [r"C:/Windows/System32/OpenSSH/ssh.exe", "-b", "192.168.1.4",
       "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "ServerAliveInterval=5",
       "-o", "ServerAliveCountMax=2", "-o", "StrictHostKeyChecking=yes",
       "-o", "HostKeyAlias=192.168.155.186", "-i", str(Path.home()/".ssh/id_ed25519"),
       "nvidia@192.168.1.5"]


def main():
    sources = {name: (PROJECT / name).read_bytes() for name in [
        "payload_catch/mocap_transform.py", "payload_catch/mocap_odometry.py",
        "payload_catch/mocap_odometry_node.py", "payload_catch/mocap_source_node.py",
        "config/mocap_a.xyz_ground.yaml"]}
    sources["payload_catch/__init__.py"] = b""
    sources["mocap_vrpn_reader.cpp"] = (PROJECT / "tools/mocap_vrpn_reader.cpp").read_bytes()
    sources["fixed_runtime_a.py"] = (WORK / "fixed_runtime_a.py").read_bytes()
    sources["fixed_guard_a.py"] = (WORK / "fixed_guard_a.py").read_bytes()
    sources["receive_parameter_guard_a.py"] = (WORK.parent / "mocap_fusion_20261023/receive_parameter_guard_a.py").read_bytes()
    sources["backup_parameters_a.py"] = (WORK.parent / "mocap_fusion_20261023/backup_parameters_a.py").read_bytes()
    sources["run_fixed_a.sh"] = (WORK / "run_fixed_a.sh").read_bytes()
    sources["yaw_plateau.py"] = (WORK/"yaw_plateau.py").read_bytes()
    hashes = {k: hashlib.sha256(v).hexdigest() for k,v in sources.items()}
    digest = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
    remote = "/home/nvidia/payload_catch_mocap_fusion_20261026/auto_yaw_" + digest[:16]
    runid = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    local = WORK / ("lift_reference_" + runid)
    local.mkdir()
    logs = remote + "/logs/" + runid
    def run(argv, label, data=None, timeout=20, required=True):
        proc = subprocess.run(SSH + [shlex.join(argv)], input=data, cwd=WORK,
                              capture_output=True, timeout=timeout, shell=False)
        (local / (label + ".stdout.log")).write_bytes(proc.stdout)
        (local / (label + ".stderr.log")).write_bytes(proc.stderr)
        if proc.returncode and required:
            print(proc.stdout.decode("utf-8", errors="replace"), flush=True)
            print(proc.stderr.decode("utf-8", errors="replace"), file=sys.stderr)
            raise RuntimeError(label + " exit " + str(proc.returncode))
        return proc
    # Read-only live status gate using the already verified previous diagnostic.
    old = "/home/nvidia/payload_catch_mocap_preview_20261021/9695526c87f980d0"
    print(run(["/bin/bash", old + "/run_disarm.sh", old], "disarm_gate", timeout=25)
          .stdout.decode("utf-8", errors="replace"), flush=True)
    payload = ["from pathlib import Path", "import ast, hashlib, subprocess",
               "assert Path('/proc/device-tree/serial-number').read_text().strip('\\0\\n')=='1612425621719'",
               "root=Path(" + repr(remote) + ")", "root.mkdir(parents=True,exist_ok=True)"]
    for name, data in sources.items():
        if name.endswith(".py"):
            ast.parse(data, name)
        if name.endswith(".sh") and (b"\r" in data or data.startswith(b"\xef\xbb\xbf")):
            raise RuntimeError("invalid Bash encoding")
        payload.extend(["p=root/" + repr(name), "p.parent.mkdir(parents=True,exist_ok=True)",
                        "data=" + repr(data),
                        "assert not p.exists() or p.read_bytes()==data, 'immutable stage mismatch'",
                        "p.write_bytes(data)", "assert hashlib.sha256(p.read_bytes()).hexdigest()==" + repr(hashes[name])])
        if name.endswith(".py"):
            payload.append("ast.parse(p.read_text(encoding='utf-8'))")
        elif name.endswith(".sh"):
            payload.append("subprocess.run(['/bin/bash','-n',str(p)],check=True,timeout=5)")
    args = ["/usr/bin/g++", "-std=c++17", "-Wall", "-Wextra", "-I/opt/ros/humble/include",
            str(remote + "/mocap_vrpn_reader.cpp"), "-L/opt/ros/humble/lib",
            "-lvrpn", "-lquat", "-lpthread", "-o", remote + "/mocap_vrpn_reader"]
    payload.extend(["subprocess.run(" + repr(args[:6] + ["-fsyntax-only", args[6]]) + ",check=True,timeout=20)",
                    "subprocess.run(" + repr(args) + ",check=True,timeout=30)",
                    "print('verified staged bridge diagnostic')"])
    code = "\n".join(payload) + "\n"
    ast.parse(code)
    (local / "stage_remote.py").write_text(code, encoding="utf-8")
    (local / "manifest.json").write_text(json.dumps({
        "remote": remote, "remote_logs": logs, "sha256": hashes, "digest": digest,
        "send_to_px4": True, "current_switched": False, "temporary_parameters":{"EKF2_EV_CTRL":11,"EKF2_MAG_TYPE":5,"EKF2_HGT_REF":3}, "automatic_rollback":True}, indent=2), encoding="utf-8")
    print(run(["/usr/bin/python3", "-"], "stage_verified", code.encode(), timeout=60)
          .stdout.decode("utf-8", errors="replace"), flush=True)
    unit="payload-catch-ground-yaw-"+runid.lower()
    launch=run(["sudo","-n","systemd-run","--unit="+unit,
        "--description=Bounded disarmed mocap yaw acceptance",
        "--property=User=nvidia","--property=WorkingDirectory="+remote,
        "--property=RuntimeMaxSec=950","--property=TimeoutStopSec=660",
        "--property=KillMode=process",
        "/usr/bin/timeout","--signal=TERM","--kill-after=660s","310s",
        "/bin/bash",remote+"/run_fixed_a.sh",remote,logs],"launch_detached",timeout=15)
    m=json.loads((local/"manifest.json").read_text())
    m.update(unit=unit,detached=True)
    (local/"manifest.json").write_text(json.dumps(m,indent=2),encoding="utf-8")
    print(launch.stdout.decode("utf-8",errors="replace"),flush=True)
    print("Started bounded independent A-machine unit",unit,"EVIDENCE",local,flush=True)


if __name__ == "__main__":
    main()

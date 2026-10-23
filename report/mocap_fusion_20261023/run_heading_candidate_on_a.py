"""Stage diagnostic bridge only; do not switch current, edit params or publish FC input."""
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
SSH = [r"C:/Windows/System32/OpenSSH/ssh.exe", "-b", "192.168.1.123",
       "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "ServerAliveInterval=5",
       "-o", "ServerAliveCountMax=2", "-o", "StrictHostKeyChecking=yes",
       "-o", "HostKeyAlias=192.168.155.186", "-i", str(Path.home()/".ssh/id_ed25519"),
       "nvidia@192.168.1.101"]


def main():
    import math
    import numpy as np
    import yaml
    probes = sorted(p for p in WORK.glob("heading_probe_20*") if (p/"validation.json").exists())
    probe = probes[-1]
    evidence = json.loads((probe/"validation.json").read_text(encoding="utf-8"))
    if not evidence["passed"]:
        raise RuntimeError("heading probe failed")
    a = math.radians(evidence["candidate_local_to_px4_yaw"]["mean_deg"])
    # A new magnetic snapshot must not silently rotate the common frame after a move.
    anchor_path = WORK/"heading_movement_baseline.json"
    if anchor_path.exists():
        anchor = json.loads(anchor_path.read_text(encoding="utf-8"))
        previous = anchor["reference"]["yaw_offset_rad"]
        change = math.degrees(math.atan2(math.sin(a-previous),math.cos(a-previous)))
        if abs(change) > 5.:
            raise RuntimeError("heading reference changed by %.3f degrees; independent frame calibration required; no SSH staging or parameter writes performed" % change)
    cfg = yaml.safe_load((PROJECT/"config/mocap_a.receive_ground.yaml").read_text(encoding="utf-8"))
    base = cfg["transform"]["world_to_local"]
    c, t = math.cos(a), math.sin(a)
    cfg["transform"]["world_to_local"] = (np.array([[c,-t,0.],[t,c,0.],[0.,0.,1.]]) @ np.array(base)).tolist()
    rows = [json.loads(line) for line in (probe/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
    collection = max(r["data"]["receive_wall_us"] for r in rows if r["kind"]=="raw")
    cfg["output_pose_frame"] = "px4_heading_aligned_ned"
    cfg["heading_reference"] = dict(method="px4_magnetic_snapshot", north_verified=False,
        yaw_offset_rad=a, std_deg=evidence["candidate_local_to_px4_yaw"]["std_deg"],
        span_deg=evidence["candidate_local_to_px4_yaw"]["span_deg"],
        collected_wall_us=collection, boot_id="a006295d-9ab4-41c6-a277-f0a2455a86a8",
        base_world_to_local=base, evidence_directory=probe.name)
    candidate = yaml.safe_dump(cfg,sort_keys=False).encode("utf-8")
    sources = {name: (PROJECT / name).read_bytes() for name in [
        "payload_catch/mocap_transform.py", "payload_catch/mocap_odometry.py",
        "payload_catch/mocap_odometry_node.py", "payload_catch/mocap_source_node.py",
        "config/mocap_a.bridge.yaml", "config/mocap_a.receive_ground.yaml"]}
    sources["config/mocap_a.receive_ground.yaml"] = candidate
    sources["payload_catch/__init__.py"] = b""
    sources["mocap_vrpn_reader.cpp"] = (PROJECT / "tools/mocap_vrpn_reader.cpp").read_bytes()
    sources["receive_ground_runtime_a.py"] = (WORK / "receive_ground_runtime_a.py").read_bytes()
    sources["receive_parameter_guard_a.py"] = (WORK / "receive_parameter_guard_a.py").read_bytes()
    sources["backup_parameters_a.py"] = (WORK / "backup_parameters_a.py").read_bytes()
    sources["run_receive_ground.sh"] = (WORK / "run_receive_ground_a.sh").read_bytes()
    hashes = {k: hashlib.sha256(v).hexdigest() for k,v in sources.items()}
    digest = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
    remote = "/home/nvidia/payload_catch_mocap_fusion_20261023/heading_candidate_" + digest[:16]
    runid = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    local = WORK / ("heading_candidate_" + runid)
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
        "send_to_px4": True, "current_switched": False}, indent=2), encoding="utf-8")
    print(run(["/usr/bin/python3", "-"], "stage_verified", code.encode(), timeout=60)
          .stdout.decode("utf-8", errors="replace"), flush=True)
    try:
        proc = run(["/usr/bin/timeout", "--signal=TERM", "--kill-after=12s", "170s",
                    "/bin/bash", remote + "/run_receive_ground.sh", remote, logs],
                   "ground_test", timeout=185, required=False)
        print(proc.stdout.decode("utf-8", errors="replace"), flush=True)
        test_exit_code = proc.returncode
    finally:
        for name in ["validation.json", "process_cleanup.json", "samples.jsonl",
                     "bridge.log", "source.log", "source/reader_process.json",
                      "source/reader_cleanup.json", "source/native.stderr.log", "task_processes.json",
                     "parameters_before.json", "parameter_lease_result.json", "parameter_guard.log",
                     "backup_ready.json", "enabled.json"]:
            fetched = run(["/bin/cat", logs + "/" + name], "fetch_" + name.replace("/", "_"),
                          required=False)
            if fetched.returncode == 0:
                target = local / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(fetched.stdout)
            else:
                print("fetch failed", name, fetched.returncode, file=sys.stderr)
    print("EVIDENCE", local, flush=True)
    audit = (WORK / "final_audit_a.py").read_bytes()
    ast.parse(audit)
    proc = run(["/usr/bin/python3", "-"], "final_audit", audit, required=False)
    print(proc.stdout.decode("utf-8", errors="replace"), flush=True)
    if test_exit_code:
        raise SystemExit(test_exit_code)


if __name__ == "__main__":
    main()

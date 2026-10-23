"""Windows SSH orchestrator: stage bounded read-only diagnostics and retain evidence."""
import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys

WORK = Path(__file__).resolve().parent
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
SSH = [r"C:/Windows/System32/OpenSSH/ssh.exe", "-b", "192.168.1.2",
       "-o", "BatchMode=yes", "-o", "ConnectTimeout=6",
       "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2",
       "-o", "StrictHostKeyChecking=yes", "-o", "HostKeyAlias=192.168.155.186",
       "-i", str(Path.home()/".ssh/id_ed25519"), "nvidia@192.168.1.4"]
RUN = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
REMOTE = "/home/nvidia/payload_catch_mocap_fusion_20261023/inspect_" + RUN
LOCAL = WORK / ("inspect_" + RUN)


def run(argv, label, data=None, timeout=15, required=True):
    proc = subprocess.run(SSH + [shlex.join(argv)], cwd=WORK, input=data,
                          capture_output=True, timeout=timeout, shell=False)
    (LOCAL / (label + ".stdout.log")).write_bytes(proc.stdout)
    (LOCAL / (label + ".stderr.log")).write_bytes(proc.stderr)
    if proc.returncode and required:
        sys.stderr.write(proc.stderr.decode("utf-8", errors="replace"))
        raise RuntimeError(label + " exit=" + str(proc.returncode))
    return proc


def main():
    LOCAL.mkdir()
    names = ["inspect_a.py", "run_inspect_a.sh"]
    payload = ["from pathlib import Path", "import ast, hashlib, subprocess",
               "assert Path('/proc/device-tree/serial-number').read_text().strip('\\0\\n')=='1612425621719'",
               "root=Path(" + repr(REMOTE) + ")", "root.mkdir(parents=True,exist_ok=False)"]
    hashes = {}
    for name in names:
        data = (WORK / name).read_bytes()
        hashes[name] = hashlib.sha256(data).hexdigest()
        if name.endswith(".py"):
            ast.parse(data.decode("utf-8"), name)
        if name.endswith(".sh") and (b"\r" in data or data.startswith(b"\xef\xbb\xbf")):
            raise RuntimeError("invalid Bash encoding")
        payload.extend(["p=root/" + repr(name), "p.write_bytes(" + repr(data) + ")",
                        "assert hashlib.sha256(p.read_bytes()).hexdigest()==" + repr(hashes[name])])
        if name.endswith(".py"):
            payload.append("ast.parse(p.read_text(encoding='utf-8'))")
        else:
            payload.append("subprocess.run(['/bin/bash','-n',str(p)],check=True,timeout=5)")
    code = "\n".join(payload) + "\n"
    ast.parse(code)
    (LOCAL / "stage_remote.py").write_text(code, encoding="utf-8")
    (LOCAL / "manifest.json").write_text(json.dumps({
        "remote": REMOTE, "sha256": hashes, "read_only_fc": True}, indent=2), encoding="utf-8")
    run(["/usr/bin/python3", "-"], "stage", code.encode("utf-8"))
    try:
        result = run(["/bin/bash", REMOTE + "/run_inspect_a.sh", REMOTE],
                     "inspection", timeout=100)
        print(result.stdout.decode("utf-8", errors="replace"))
    finally:
        for name in ["identity.json", "live_inspection.json", "parameters_backup.json",
                     "raw_vrpn.jsonl", "vrpn.stderr.log", "process_cleanup.json"]:
            result = run(["/bin/cat", REMOTE + "/" + name], "fetch_" + name, required=False)
            if result.returncode == 0:
                (LOCAL / name).write_bytes(result.stdout)
            else:
                print("fetch failed", name, result.returncode, file=sys.stderr)
    print("EVIDENCE", LOCAL, flush=True)


if __name__ == "__main__":
    main()

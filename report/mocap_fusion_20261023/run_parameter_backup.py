"""Windows one-shot runner for passive FC receive; leaves all services intact."""
import ast
from datetime import datetime, timezone
import json
from pathlib import Path
import shlex
import subprocess
import sys
WORK = Path(__file__).resolve().parent
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
SSH = [r"C:/Windows/System32/OpenSSH/ssh.exe", "-b", "192.168.1.2",
       "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "ServerAliveInterval=5",
       "-o", "ServerAliveCountMax=2", "-o", "StrictHostKeyChecking=yes",
       "-o", "HostKeyAlias=192.168.155.186", "-i", str(Path.home()/".ssh/id_ed25519"),
       "nvidia@192.168.1.4"]


def main():
    name = "parameters_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    local = WORK / name
    local.mkdir()
    remote = "/home/nvidia/payload_catch_mocap_fusion_20261023/" + name
    data = (WORK / "backup_parameters_a.py").read_bytes()
    ast.parse(data)
    code = ("from pathlib import Path\nimport ast\n"
            "assert Path('/proc/device-tree/serial-number').read_text().strip('\\0\\n')=='1612425621719'\n"
            "p=Path(" + repr(remote) + ")\np.mkdir(parents=True,exist_ok=False)\n"
            "data=" + repr(data) + "\nast.parse(data)\n"
            "(p/'backup_parameters_a.py').write_bytes(data)\n")
    ast.parse(code)
    (local / "stage_remote.py").write_text(code, encoding="utf-8")
    (local / "manifest.json").write_text(json.dumps({"remote": remote, "read_only_fc": True}),
                                       encoding="utf-8")
    for label, argv, stdin, timeout in [
        ("stage", ["/usr/bin/python3", "-"], code.encode("utf-8"), 15),
        ("backup", ["sudo", "-n", "/usr/bin/env",
                    "PYTHONPATH=/home/nvidia/.local/lib/python3.10/site-packages",
                    "/usr/bin/timeout", "--signal=TERM", "--kill-after=3s",
                    "60s", "/usr/bin/python3", remote + "/backup_parameters_a.py"], None, 70),
        ("fetch", ["/bin/cat", remote + "/parameters_backup_passive.json"], None, 15)]:
        proc = subprocess.run(SSH + [shlex.join(argv)], input=stdin, cwd=WORK,
                              capture_output=True, timeout=timeout, shell=False)
        (local / (label + ".stdout.log")).write_bytes(proc.stdout)
        (local / (label + ".stderr.log")).write_bytes(proc.stderr)
        print(label, "exit", proc.returncode, flush=True)
        if proc.stdout and label != "fetch":
            print(proc.stdout.decode("utf-8", errors="replace"), flush=True)
        if proc.returncode:
            print(proc.stderr.decode("utf-8", errors="replace"), file=sys.stderr)
            raise SystemExit(proc.returncode)
        if label == "fetch":
            (local / "parameters_backup.json").write_bytes(proc.stdout)
    print("EVIDENCE", local, flush=True)


if __name__ == "__main__":
    main()

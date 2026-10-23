"""Windows bounded passive-wire probe runner; retain all stderr and exit status."""
import ast
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys

WORK = Path(__file__).resolve().parent
data = (WORK / "probe_wire_time_a.py").read_bytes()
ast.parse(data)
out = WORK / ("wire_time_" + datetime.now().strftime("%Y%m%dT%H%M%S"))
out.mkdir()
proc = subprocess.run(
    [r"C:/Windows/System32/OpenSSH/ssh.exe", "-b", "192.168.1.2",
     "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "ServerAliveInterval=5",
     "-o", "ServerAliveCountMax=2", "-o", "StrictHostKeyChecking=yes",
     "-o", "HostKeyAlias=192.168.155.186", "-i", str(Path.home()/".ssh/id_ed25519"),
     "nvidia@192.168.1.4", "sudo -n /usr/bin/timeout --signal=TERM --kill-after=3s 18s /usr/bin/python3 -"],
    input=data, cwd=WORK, capture_output=True, timeout=28, shell=False)
(out / "stdout.log").write_bytes(proc.stdout)
(out / "stderr.log").write_bytes(proc.stderr)
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
print(proc.stdout.decode("utf-8", errors="replace"))
print(proc.stderr.decode("utf-8", errors="replace"), file=sys.stderr)
if proc.returncode == 0:
    result = json.loads(proc.stdout.decode("utf-8"))
    (out / "result.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
print("SSH_EXIT", proc.returncode, "EVIDENCE", out)
raise SystemExit(proc.returncode)

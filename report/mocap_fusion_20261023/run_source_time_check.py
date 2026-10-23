"""Run bounded read-only A clock probe; log local and remote times."""
import ast
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
WORK = Path(__file__).resolve().parent
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
script = WORK / "check_source_time_a.py"
data = script.read_bytes()
ast.parse(data, str(script))
stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
local = WORK / ("source_time_" + stamp)
local.mkdir()
start = datetime.now().astimezone().isoformat()
proc = subprocess.run([r"C:/Windows/System32/OpenSSH/ssh.exe", "-b", "192.168.1.2",
    "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "ServerAliveInterval=5",
    "-o", "ServerAliveCountMax=2", "-o", "StrictHostKeyChecking=yes",
    "-o", "HostKeyAlias=192.168.155.186", "-i", str(Path.home()/".ssh/id_ed25519"),
    "nvidia@192.168.1.4", "/usr/bin/python3 -"],
    input=data, cwd=WORK, capture_output=True, timeout=30, shell=False)
(local / "stdout.log").write_bytes(proc.stdout)
(local / "stderr.log").write_bytes(proc.stderr)
record = dict(local_start=start, local_end=datetime.now().astimezone().isoformat(),
              ssh_exit=proc.returncode)
if proc.returncode == 0:
    record["remote"] = json.loads(proc.stdout.decode("utf-8"))
(local / "result.json").write_text(json.dumps(record, indent=2, ensure_ascii=False),
                                  encoding="utf-8")
print(json.dumps(record, indent=2, ensure_ascii=False))
if proc.stderr:
    print(proc.stderr.decode("utf-8", errors="replace"), file=sys.stderr)
print("EVIDENCE", local)
raise SystemExit(proc.returncode)

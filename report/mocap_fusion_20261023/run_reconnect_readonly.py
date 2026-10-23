"""SSH exact-device read-only reconnect check; preserve evidence locally."""
import ast
from pathlib import Path
import subprocess
import sys
WORK = Path(__file__).resolve().parent
data = (WORK/"reconnect_readonly_a.py").read_bytes()
ast.parse(data)
p = subprocess.run([r"C:/Windows/System32/OpenSSH/ssh.exe", "-b", "192.168.1.123",
    "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "StrictHostKeyChecking=yes",
    "-o", "HostKeyAlias=192.168.155.186", "-i", str(Path.home()/".ssh/id_ed25519"),
    "nvidia@192.168.1.101", "/usr/bin/python3 -"], cwd=WORK, input=data,
    timeout=45, capture_output=True, shell=False)
(WORK/"reconnect_readonly.stdout.log").write_bytes(p.stdout)
(WORK/"reconnect_readonly.stderr.log").write_bytes(p.stderr)
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
print(p.stdout.decode("utf-8",errors="replace"))
print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
if p.returncode == 0:
    (WORK/"reconnect_readonly.json").write_bytes(p.stdout)
raise SystemExit(p.returncode)

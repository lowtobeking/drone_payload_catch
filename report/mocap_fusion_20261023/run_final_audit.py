"""Run read-only final audit via stdin and preserve stdout/stderr."""
import ast
from pathlib import Path
import subprocess
import sys
WORK = Path(__file__).resolve().parent
script = WORK / "final_audit_a.py"
ast.parse(script.read_text(encoding="utf-8"))
proc = subprocess.run([r"C:/Windows/System32/OpenSSH/ssh.exe", "-b", "192.168.1.2",
    "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "StrictHostKeyChecking=yes",
    "-o", "HostKeyAlias=192.168.155.186", "-i", str(Path.home()/".ssh/id_ed25519"),
    "nvidia@192.168.1.4", "/usr/bin/python3 -"],
    input=script.read_bytes(), cwd=WORK, capture_output=True, timeout=20, shell=False)
(WORK / "final_audit.stdout.log").write_bytes(proc.stdout)
(WORK / "final_audit.stderr.log").write_bytes(proc.stderr)
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
print(proc.stdout.decode("utf-8", errors="replace"))
print(proc.stderr.decode("utf-8", errors="replace"), file=sys.stderr)
if proc.returncode == 0:
    (WORK / "final_audit.json").write_bytes(proc.stdout)
raise SystemExit(proc.returncode)

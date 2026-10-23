"""Bounded SSH read-only network-time query runner."""
import ast
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
WORK = Path(__file__).resolve().parent
data = (WORK/"query_server_ntp_a.py").read_bytes()
ast.parse(data)
out = WORK/("server_ntp_"+datetime.now().strftime("%Y%m%dT%H%M%S"))
out.mkdir()
proc = subprocess.run([r"C:/Windows/System32/OpenSSH/ssh.exe","-b","192.168.1.123",
    "-o","BatchMode=yes","-o","ConnectTimeout=6","-o","StrictHostKeyChecking=yes",
    "-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),
    "nvidia@192.168.1.101","/usr/bin/python3 -"],input=data,cwd=WORK,
    capture_output=True,timeout=18,shell=False)
(out/"stdout.log").write_bytes(proc.stdout)
(out/"stderr.log").write_bytes(proc.stderr)
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
print(proc.stdout.decode("utf-8",errors="replace"))
print(proc.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
if proc.returncode==0:
    (out/"result.json").write_bytes(proc.stdout)
print("EVIDENCE",out)
raise SystemExit(proc.returncode)

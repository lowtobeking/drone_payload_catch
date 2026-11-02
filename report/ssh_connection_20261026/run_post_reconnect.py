import ast
from pathlib import Path
import subprocess
import sys
work=Path(__file__).resolve().parent
code=(work/"post_reconnect_a.py").read_bytes()
ast.parse(code)
cmd=["C:/Windows/System32/OpenSSH/ssh.exe","-F","NUL","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.5","sudo -n /usr/bin/python3 -"]
p=subprocess.run(cmd,input=code,cwd=work,timeout=40,capture_output=True,shell=False)
(work/"post_reconnect.stdout.log").write_bytes(p.stdout)
(work/"post_reconnect.stderr.log").write_bytes(p.stderr)
sys.stdout.reconfigure(encoding="utf-8")
print(p.stdout.decode("utf-8",errors="replace"))
print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
sys.exit(p.returncode)

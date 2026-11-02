"""Execute final read-only audit and route-helper commit over ordinary Wi-Fi SSH."""
import ast
from pathlib import Path
import subprocess
import sys
WORK=Path(__file__).resolve().parent
code=(WORK/"final_network_a.py").read_bytes()
ast.parse(code)
cmd=["C:/Windows/System32/OpenSSH/ssh.exe","-F","NUL","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.5","sudo -n /usr/bin/python3 -"]
p=subprocess.run(cmd,input=code,cwd=WORK,capture_output=True,timeout=50,shell=False)
(WORK/"final_network.stdout.log").write_bytes(p.stdout)
(WORK/"final_network.stderr.log").write_bytes(p.stderr)
sys.stdout.reconfigure(encoding="utf-8")
print(p.stdout.decode("utf-8",errors="replace"))
print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
sys.exit(p.returncode)

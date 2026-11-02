"""Stop the exact continuous session and wait for parameter restoration."""
import ast
import json
from pathlib import Path
import subprocess
WORK=Path(__file__).resolve().parent
manifest=json.loads((WORK/"active_session.json").read_text(encoding="utf-8"))
code="from pathlib import Path\nPath("+repr(manifest["remote_logs"]+"/stop.request") + ").touch()\nprint('stop requested')\n"
ast.parse(code)
ssh=["C:/Windows/System32/OpenSSH/ssh.exe","-F","NUL","-b","192.168.1.4","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","HostKeyAlias=192.168.155.186","-o","StrictHostKeyChecking=yes","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.5","/usr/bin/python3 -"]
proc=subprocess.run(ssh,input=code.encode(),cwd=WORK,capture_output=True,timeout=20,shell=False)
print(proc.stdout.decode("utf-8",errors="replace"),end="")
print(proc.stderr.decode("utf-8",errors="replace"),end="")
raise SystemExit(proc.returncode)

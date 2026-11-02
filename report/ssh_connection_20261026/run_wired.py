"""Execute checked local audit code over strictly authenticated SSH."""
import ast
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
WORK=Path(__file__).resolve().parent
mode=sys.argv[1] if len(sys.argv)>1 else "audit"
path=WORK/(mode+"_a.py")
code=path.read_bytes()
ast.parse(code)
cmd=["C:/Windows/System32/OpenSSH/ssh.exe","-F","NUL","-b","10.41.10.50","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@10.41.10.100","/usr/bin/python3 -"]
p=subprocess.run(cmd,input=code,cwd=WORK,capture_output=True,timeout=60,shell=False)
stamp=datetime.now().strftime("%Y%m%dT%H%M%S")
(WORK/(mode+"_"+stamp+".stdout.log")).write_bytes(p.stdout)
(WORK/(mode+"_"+stamp+".stderr.log")).write_bytes(p.stderr)
sys.stdout.reconfigure(encoding="utf-8")
print(p.stdout.decode("utf-8",errors="replace"))
print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
if p.returncode==0:
    try:
        result=json.loads(p.stdout)
    except ValueError:
        pass
    else:
        (WORK/(mode+"_"+stamp+".json")).write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
sys.exit(p.returncode)

"""Connect from historical laptop .2; read identity and routes only."""
import ast
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
WORK=Path(__file__).resolve().parent
REMOTE="""import json
from pathlib import Path
import socket
import subprocess
def run(args):
    p=subprocess.run(args,capture_output=True,text=True,timeout=5,check=True)
    return p.stdout.strip()
print(json.dumps(dict(hostname=socket.gethostname(),serial=Path('/proc/device-tree/serial-number').read_bytes().rstrip(bytes([0,10,13])).decode(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),addresses=run(['hostname','-I']),route_to_laptop=run(['ip','route','get','192.168.1.2']))))
"""
def main():
    sys.stdout.reconfigure(encoding="utf-8")
    ast.parse(REMOTE)
    stamp=datetime.now().strftime("%Y%m%dT%H%M%S")
    for ip in ("192.168.1.5","192.168.1.4"):
        command=["C:/Windows/System32/OpenSSH/ssh.exe","-F","NUL","-b","192.168.1.2","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","ConnectionAttempts=1","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@"+ip,"/usr/bin/python3 -"]
        try:
            proc=subprocess.run(command,input=REMOTE.encode(),cwd=WORK,capture_output=True,timeout=15,shell=False)
        except subprocess.TimeoutExpired as error:
            (WORK/("historical_"+stamp+"_"+ip+".timeout.log")).write_bytes((error.stdout or b"")+(error.stderr or b""))
            continue
        (WORK/("historical_"+stamp+"_"+ip+".stdout.log")).write_bytes(proc.stdout)
        (WORK/("historical_"+stamp+"_"+ip+".stderr.log")).write_bytes(proc.stderr)
        print(ip,"exit",proc.returncode,proc.stdout.decode(errors="replace"),proc.stderr.decode(errors="replace"),flush=True)
        if proc.returncode==0:
            result=json.loads(proc.stdout)
            if result["serial"]=="1612425621719":
                result.update(ip=ip,source="192.168.1.2",time=stamp,read_only=True)
                (WORK/"historical_connection_success.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
                return 0
    return 2
if __name__=="__main__":
    sys.exit(main())

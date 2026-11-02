"""Discover A through existing SSH key and confirm its serial; read-only."""
import ast
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
from pathlib import Path
import socket
import subprocess
import sys
WORK=Path(__file__).resolve().parent
SOURCE="192.168.1.4"
REMOTE="""import json
from pathlib import Path
import socket
import subprocess
serial=Path('/proc/device-tree/serial-number').read_text().strip('\\x00\\n')
p=subprocess.run(['/bin/hostname','-I'],capture_output=True,text=True,timeout=5,check=True)
print(json.dumps(dict(hostname=socket.gethostname(),serial=serial,addresses=p.stdout.strip(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),uptime=Path('/proc/uptime').read_text().split()[0])))
"""
def probe(ip):
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as sock:
            sock.settimeout(.8)
            sock.bind((SOURCE,0))
            sock.connect((ip,22))
            return ip
    except OSError:
        return None
def main():
    sys.stdout.reconfigure(encoding="utf-8")
    ast.parse(REMOTE)
    with ThreadPoolExecutor(max_workers=24) as pool:
        found=sorted(ip for ip in pool.map(probe,["192.168."+str(net)+"."+str(i) for net in range(4) for i in range(1,255) if (net,i) not in {(1,4),(1,1)}]) if ip)
    (WORK/"discovery.json").write_text(json.dumps(dict(source=SOURCE,ssh_candidates=found,time=datetime.now().astimezone().isoformat()),indent=2)+"\n",encoding="utf-8")
    print("SSH candidates:",found,flush=True)
    verified=[]
    for ip in found:
        cmd=["C:/Windows/System32/OpenSSH/ssh.exe","-b",SOURCE,"-o","BatchMode=yes","-o","ConnectTimeout=5","-o","ConnectionAttempts=1","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@"+ip,"/usr/bin/python3 -"]
        try:
            p=subprocess.run(cmd,input=REMOTE.encode(),cwd=WORK,timeout=14,capture_output=True,shell=False)
        except subprocess.TimeoutExpired as e:
            (WORK/(ip+".timeout.log")).write_bytes((e.stdout or b"")+(e.stderr or b""))
            print(ip,"SSH timeout",flush=True)
            continue
        (WORK/(ip+".stdout.log")).write_bytes(p.stdout)
        (WORK/(ip+".stderr.log")).write_bytes(p.stderr)
        print(ip,"exit",p.returncode,p.stdout.decode("utf-8",errors="replace"),p.stderr.decode("utf-8",errors="replace"),flush=True)
        if p.returncode==0:
            result=json.loads(p.stdout)
            if result.get("serial")=="1612425621719":
                result["ip"]=ip
                verified.append(result)
                break
    (WORK/"connection_result.json").write_text(json.dumps(dict(verified=verified,read_only=True,persistent_sessions=0),ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    if not verified:
        sys.exit(2)
if __name__=="__main__":
    main()

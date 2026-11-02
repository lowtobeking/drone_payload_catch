"""Verify Wi-Fi SSH and exact device identity via explicit interface transport."""
import ast
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
WORK=Path(__file__).resolve().parent
REMOTE="""import json
from pathlib import Path
import subprocess
def run(args):
    p=subprocess.run(args,capture_output=True,text=True,timeout=8)
    return dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
print(json.dumps(dict(serial=Path('/proc/device-tree/serial-number').read_bytes().rstrip(bytes([0])).decode(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),peer=Path('/run/payload-catch-wifi-peer.json').read_text(),wifi_route=run(['ip','route','get','192.168.1.4']),fc_route=run(['ip','route','get','10.41.10.2']),mesh_route=run(['ip','route','get','192.168.1.203']),agent=run(['ps','-p','1155,1156','-o','pid,ppid,lstart,args']))))
"""
def main():
    sys.stdout.reconfigure(encoding="utf-8")
    ast.parse(REMOTE)
    proxy=str(Path(sys.executable)).replace("\\","/")+" -B "+str(WORK/"wifi_socket_proxy.py").replace("\\","/")
    cmd=["C:/Windows/System32/OpenSSH/ssh.exe","-F","NUL","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-o","ProxyCommand="+proxy,"-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.5","/usr/bin/python3 -"]
    p=subprocess.run(cmd,input=REMOTE.encode(),cwd=WORK,timeout=30,capture_output=True,shell=False)
    (WORK/"wifi_explicit.stdout.log").write_bytes(p.stdout)
    (WORK/"wifi_explicit.stderr.log").write_bytes(p.stderr)
    print(p.stdout.decode("utf-8",errors="replace"),flush=True)
    print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr,flush=True)
    if p.returncode:
        sys.exit(p.returncode)
    data=json.loads(p.stdout)
    if data["serial"]!="1612425621719" or "wlP1p1s0" not in data["wifi_route"]["stdout"]:
        raise RuntimeError("Wi-Fi identity/route mismatch")
    (WORK/"wifi_explicit_verified.json").write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
if __name__=="__main__":
    main()

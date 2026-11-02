"""Retry discovery using explicit Wi-Fi socket interface and longer deadlines."""
import ast
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import importlib.util
import json
from pathlib import Path
import socket
import subprocess
import sys
import time

WORK=Path(__file__).resolve().parent
def probe(ip):
    started=time.monotonic()
    item={"ip":ip}
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.IPPROTO_IP,31,socket.htonl(23))
            sock.settimeout(2.)
            sock.bind(("192.168.1.4",0))
            sock.connect((ip,22))
            item["tcp"]="connected"
            try:
                item["banner"]=sock.recv(256).decode("ascii",errors="replace")
            except socket.timeout:
                item["banner"]="timeout"
    except OSError as e:
        item.update(tcp="failed",error=str(e))
    item["seconds"]=round(time.monotonic()-started,3)
    return item
def main():
    sys.stdout.reconfigure(encoding="utf-8")
    addresses=["192.168."+str(net)+"."+str(i) for net in range(4) for i in range(1,255) if (net,i) not in {(1,1),(1,4)}]
    results=[]
    with ThreadPoolExecutor(max_workers=40) as pool:
        for item in pool.map(probe,addresses):
            results.append(item)
    found=[r for r in results if r["tcp"]=="connected"]
    stamp=datetime.now().astimezone().strftime("%Y%m%dT%H%M%S")
    (WORK/("retry_"+stamp+".json")).write_text(json.dumps({"time":stamp,"results":results},ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("TCP22 connected:",json.dumps(found,ensure_ascii=False),flush=True)
    spec=importlib.util.spec_from_file_location("connect_a",WORK/"connect_a.py")
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ast.parse(module.REMOTE)
    verified=[]
    for result in found:
        ip=result["ip"]
        cmd=["C:/Windows/System32/OpenSSH/ssh.exe","-b","192.168.1.4","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","ConnectionAttempts=1","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@"+ip,"/usr/bin/python3 -"]
        p=subprocess.run(cmd,input=module.REMOTE.encode(),cwd=WORK,timeout=20,capture_output=True,shell=False)
        (WORK/(ip+"."+stamp+".stdout.log")).write_bytes(p.stdout)
        (WORK/(ip+"."+stamp+".stderr.log")).write_bytes(p.stderr)
        print(ip,"exit",p.returncode,p.stdout.decode("utf-8",errors="replace"),p.stderr.decode("utf-8",errors="replace"),flush=True)
        if p.returncode==0:
            identity=json.loads(p.stdout)
            if identity.get("serial")=="1612425621719":
                verified.append(dict(ip=ip,**identity))
                break
    (WORK/("retry_connection_"+stamp+".json")).write_text(json.dumps({"verified":verified,"read_only":True,"persistent_sessions":0},ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("Verified A:",json.dumps(verified,ensure_ascii=False),flush=True)
    return 0 if verified else 2
if __name__=="__main__":
    sys.exit(main())

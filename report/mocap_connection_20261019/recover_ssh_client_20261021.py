"""Use temporary source address; identify A by existing SSH host key."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import socket
import subprocess
import sys
work=Path(__file__).resolve().parent
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
def probe(host):
    try:
        with socket.create_connection((host,22),timeout=0.6,source_address=("192.168.1.3",0)) as conn:
            conn.settimeout(0.6)
            conn.recv(128)
            return host
    except OSError:
        return None
def main():
    candidates=[host for host in ["192.168.1.4","192.168.1.101"] if probe(host)]
    if not candidates:
        with ThreadPoolExecutor(max_workers=24) as pool:
            candidates=[h for h in pool.map(probe,["192.168.1."+str(i) for i in range(4,255)]) if h]
    print("SSH_CANDIDATES",candidates,flush=True)
    for host in candidates[:16]:
        base=[r"C:/Windows/System32/OpenSSH/ssh.exe","-b","192.168.1.3","-o","BatchMode=yes","-o","ConnectTimeout=4","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",r"C:/Users/caoli/.ssh/id_ed25519","nvidia@"+host]
        p=subprocess.run(base+["hostname"],cwd=work,capture_output=True,timeout=10,shell=False)
        print(json.dumps({"host":host,"exit":p.returncode,"stdout":p.stdout.decode(errors="replace"),"stderr":p.stderr.decode(errors="replace")}),flush=True)
        if p.returncode==0 and p.stdout.strip()==b"nvidia-desktop":
            p=subprocess.run(base+["/usr/bin/python3 -"],input=(work/"probe_planeA_remote_20261021.py").read_bytes(),cwd=work,capture_output=True,timeout=90,shell=False)
            (work/"planeA_remote_20261021.stdout.log").write_bytes(p.stdout)
            (work/"planeA_remote_20261021.stderr.log").write_bytes(p.stderr)
            print(p.stdout.decode("utf-8",errors="replace"),flush=True)
            print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr,flush=True)
            (work/"recovery_result_20261021.json").write_text(json.dumps({"host":host,"remote_exit":p.returncode}),encoding="utf-8")
            return p.returncode
    (work/"recovery_result_20261021.json").write_text(json.dumps({"connected":False,"candidates":candidates}),encoding="utf-8")
    return 2
if __name__=="__main__":
    raise SystemExit(main())

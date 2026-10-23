from pathlib import Path
import json
import subprocess
import sys
work=Path(__file__).resolve().parent
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
base=[r"C:/Windows/System32/OpenSSH/ssh.exe","-o","BatchMode=yes","-o","ConnectTimeout=5","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",r"C:/Users/caoli/.ssh/id_ed25519"]
host="nvidia@192.168.1.4"
def remote(mode):
    p=subprocess.run(base+["-b","192.168.1.3",host,"/usr/bin/python3 - "+mode],input=(work/"repair_qgc_route_remote_20261021.py").read_bytes(),cwd=work,capture_output=True,timeout=65,shell=False)
    (work/("qgc_route_"+mode+"_20261021.stdout.log")).write_bytes(p.stdout)
    (work/("qgc_route_"+mode+"_20261021.stderr.log")).write_bytes(p.stderr)
    print(p.stdout.decode("utf-8",errors="replace"),flush=True)
    print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr,flush=True)
    return p.returncode
def main():
    committed=False
    try:
        if remote("prepare"):
            return 1
        p=subprocess.run(base+[host,"/bin/cat /proc/device-tree/serial-number"],cwd=work,capture_output=True,timeout=12,shell=False)
        print("NORMAL_SSH",p.returncode,repr(p.stdout),p.stderr.decode(errors="replace"),flush=True)
        if p.returncode or p.stdout.rstrip(bytes([0,10,13]))!=b"1612425621719":
            return 1
        if remote("commit"):
            return 1
        committed=True
        (work/"qgc_normal_ssh_verified_20261021.json").write_text(json.dumps({"host":"192.168.1.4","source":"192.168.1.124","serial":"1612425621719","normal_ssh":True}),encoding="utf-8")
        return 0
    finally:
        if not committed:
            remote("rollback")
if __name__=="__main__":
    raise SystemExit(main())

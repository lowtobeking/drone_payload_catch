"""Bounded SSH discovery on current lab /24; no device changes."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import json
import socket
from attempt_wifi import ssh
work=Path(__file__).resolve().parent
def probe(host):
    try:
        with socket.create_connection((host,22),timeout=0.65) as conn:
            conn.settimeout(0.7)
            return {"host":host,"banner":conn.recv(128).decode("ascii",errors="replace").strip()}
    except OSError:
        return None
def main():
    with ThreadPoolExecutor(max_workers=24) as pool:
        found=[r for r in pool.map(probe,["192.168.1."+str(i) for i in range(1,255) if i!=2]) if r]
    result={"ssh_hosts":found}
    try:
        with socket.create_connection(("192.168.1.111",3883),timeout=3) as conn:
            conn.settimeout(3)
            result["vrpn_greeting"]=repr(conn.recv(64))
    except OSError as exc:
        result["vrpn_error"]=str(exc)
    for item in found:
        p=ssh(item["host"],["hostname"],timeout=10)
        item.update(ssh_returncode=p.returncode,stdout=p.stdout,stderr=p.stderr)
    (work/"reconnect_20261021.json").write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=="__main__":
    main()

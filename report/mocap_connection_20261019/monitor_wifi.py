"""Recover status of already scheduled switch; never starts a second attempt."""
from attempt_wifi import WORK,STATE,ssh,status,log,open_ssh
from concurrent.futures import ThreadPoolExecutor
import json
import time
import sys
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
deadline=time.monotonic()+165
while time.monotonic()<deadline:
    data=status("192.168.155.186")
    if data:
        (WORK/"wifi_attempt_status.json").write_text(json.dumps(data,indent=2)+"\n",encoding="utf-8")
        print(json.dumps(data,indent=2),flush=True)
        if data["status"] in ("rolled_back","restore_failed","committed"):
            raise SystemExit(0 if data["status"]!="restore_failed" else 1)
    candidates=["192.168.1."+str(n) for n in range(2,255) if n not in (3,111)]
    with ThreadPoolExecutor(max_workers=24) as pool:
        hosts=[h for h in pool.map(open_ssh,candidates) if h]
    print("SSH candidates:",hosts,flush=True)
    for host in hosts:
        data=status(host)
        if not data or data["status"]!="awaiting_ssh_confirmation":continue
        p=ssh(host,["sudo","-n","/usr/bin/python3",STATE,"commit"])
        log(p,"wifi_commit")
        print(p.stdout,flush=True)
        if p.returncode:
            print(p.stderr,file=sys.stderr)
            continue
        data=json.loads(p.stdout)
        data["ssh_host"]=host
        (WORK/"wifi_attempt_status.json").write_text(json.dumps(data,indent=2)+"\n",encoding="utf-8")
        raise SystemExit(0)
    time.sleep(8)
raise SystemExit("No confirmed outcome; inspect scheduled rollback")

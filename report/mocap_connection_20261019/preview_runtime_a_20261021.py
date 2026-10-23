"""Bounded preview processes; exact owned PID cleanup in finally."""
from pathlib import Path
from datetime import datetime, timezone
import json
import os
import signal
import subprocess
import sys
import time

root=Path.cwd()
logs=Path(sys.argv[1])
logs.mkdir(parents=True,exist_ok=True)
commands=[
    ("vrpn",["/usr/bin/python3",str(root/"planeA_raw_source.py"),"--reader",str(root/"planeA_stream"),"--nic","192.168.1.4"]),
    ("preview",["/usr/bin/python3","-m","payload_catch.mocap_preview_node","--config",str(root/"config/mocap_a.preview.yaml")])]
children=[]
files=[]
record=dict(started_at=datetime.now(timezone.utc).isoformat(),root=str(root),processes=[],cleanup=[])
try:
    for name,argv in commands:
        f=(logs/(name+".log")).open("wb")
        files.append(f)
        p=subprocess.Popen(argv,cwd=root,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        children.append((name,p))
        record["processes"].append(dict(name=name,pid=p.pid,argv=argv))
        (logs/"processes.json").write_text(json.dumps(record,indent=2),encoding="utf-8")
    with (logs/"capture.log").open("wb") as f:
        p=subprocess.run(["/usr/bin/python3",str(root/"preview_capture.py"),str(logs)],cwd=root,stdout=f,stderr=subprocess.STDOUT,timeout=25,shell=False)
        record["capture_exit"]=p.returncode
    record["source"]="native_planeA_only"
    for name,p in children:
        if p.poll() is not None:
            raise RuntimeError(name+" exited unexpectedly: "+str(p.returncode))
    if record["capture_exit"]!=0:
        raise RuntimeError("capture failed; inspect capture.log")
finally:
    for name,p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL)
                    p.wait(timeout=3)
        record["cleanup"].append(dict(name=name,pid=p.pid,exit=p.returncode,exited=p.poll() is not None))
    for f in files:f.close()
    record["finished_at"]=datetime.now(timezone.utc).isoformat()
    (logs/"processes.json").write_text(json.dumps(record,indent=2),encoding="utf-8")
print(json.dumps(record),flush=True)

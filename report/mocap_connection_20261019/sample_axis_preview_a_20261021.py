"""Repeat the deployed bounded preview, then summarize last three stationary seconds."""
from pathlib import Path
from datetime import datetime,timezone
import ast
import hashlib
import json
import shlex
import subprocess
import sys
work=Path(__file__).resolve().parent
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
label=sys.argv[1]
allowed={"baseline","forward","right","up","yaw_baseline","yaw_left"}
if label not in allowed:raise SystemExit("invalid phase")
base=[r"C:/Windows/System32/OpenSSH/ssh.exe","-o","BatchMode=yes","-o","ConnectTimeout=6","-o","ServerAliveInterval=5","-o","ServerAliveCountMax=2","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.4"]
deployment=json.loads((work/"preview_on_a_20261021T091329Z/deployment.json").read_text(encoding="utf-8"))
root=deployment["remote"]
runid=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+"_"+label
remote_logs=root+"/logs/"+runid
local=work/("axis_check_"+runid)
local.mkdir()
record=dict(label=label,remote_root=root,remote_logs=remote_logs,local=str(local))
reference_remote=None
if label!="baseline":
    state=json.loads((work/"axis_check_state_20261021.json").read_text(encoding="utf-8"))
    expected={"forward":"baseline","right":"forward","up":"right","yaw_baseline":"up","yaw_left":"yaw_baseline"}
    if state["label"]!=expected[label]:raise RuntimeError("incorrect step reference")
    reference_remote=state["remote_logs"]+"/axis_summary.json"
    record["reference"]=state
(local/"sample_manifest.json").write_text(json.dumps(record,indent=2),encoding="utf-8")

def run(argv,name,data=None,timeout=30):
    p=subprocess.run(base+[shlex.join(argv)],cwd=work,input=data,capture_output=True,timeout=timeout,shell=False)
    (local/(name+".stdout.log")).write_bytes(p.stdout)
    (local/(name+".stderr.log")).write_bytes(p.stderr)
    if p.returncode:
        print(p.stdout.decode(errors="replace"),flush=True)
        print(p.stderr.decode(errors="replace"),file=sys.stderr,flush=True)
        raise RuntimeError(name+" exit="+str(p.returncode))
    return p.stdout

source=(work/"axis_preview_stats_20261021.py").read_bytes()
ast.parse(source.decode("utf-8"))
stats=root+"/manual_axis_checks/axis_stats.py"
checks=["from pathlib import Path","import hashlib,ast","root=Path("+repr(root)+")",
        "assert Path('/proc/device-tree/serial-number').read_text().strip('\\x00\\n')=='1612425621719'"]
for name,digest in deployment["sha256"].items():
    checks.append("assert hashlib.sha256((root/"+repr(name)+").read_bytes()).hexdigest()=="+repr(digest)+", "+repr(name))
checks.extend(["p=Path("+repr(stats)+")","p.parent.mkdir(exist_ok=True)","p.write_bytes("+repr(source)+")","ast.parse(p.read_text(encoding='utf-8'))","print('identity, deployed preview hashes and stats syntax verified')"])
upload="\n".join(checks)+"\n"
ast.parse(upload)
print(run(["/usr/bin/python3","-"],"verify",upload.encode()).decode(),flush=True)
print(run(["/bin/bash",root+"/run_disarm.sh",root],"disarmed_check",timeout=25).decode(),flush=True)
try:
    print(run(["/usr/bin/timeout","--signal=INT","--kill-after=12s","55s","/bin/bash",root+"/run_preview.sh",root,remote_logs],"runtime",timeout=70).decode(),flush=True)
    argv=["/usr/bin/python3",stats,remote_logs,label]
    if reference_remote:argv.append(reference_remote)
    print(run(argv,"stats").decode(),flush=True)
finally:
    for name in ["capture.json","poses.jsonl","processes.json","vrpn.log","preview.log","axis_summary.json"]:
        p=subprocess.run(base+[shlex.join(["/bin/cat",remote_logs+"/"+name])],cwd=work,capture_output=True,timeout=15,shell=False)
        (local/name).write_bytes(p.stdout)
        (local/(name+".fetch_stderr.log")).write_bytes(p.stderr)
        if p.returncode:print("fetch failed",name,p.returncode,file=sys.stderr)
result=json.loads((local/"axis_summary.json").read_text(encoding="utf-8"))
if not result["stationary"]:raise RuntimeError("not stationary; keep prior reference")
(work/"axis_check_state_20261021.json").write_text(json.dumps(record,indent=2),encoding="utf-8")
print("SAVED",str(local),flush=True)

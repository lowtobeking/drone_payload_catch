"""Stage only mocap preview; keep current flight release untouched."""
import ast
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
work=Path(__file__).resolve().parent
project=work.parents[1]
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
base=[r"C:/Windows/System32/OpenSSH/ssh.exe","-o","BatchMode=yes","-o","ConnectTimeout=6","-o","ServerAliveInterval=5","-o","ServerAliveCountMax=2","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.4"]
sources={
    "payload_catch/__init__.py":b"",
    "payload_catch/mocap_transform.py":(project/"payload_catch/mocap_transform.py").read_bytes(),
    "payload_catch/mocap_preview_node.py":(project/"payload_catch/mocap_preview_node.py").read_bytes(),
    "config/mocap_a.preview.yaml":(project/"config/mocap_a.preview.yaml").read_bytes(),
    "planeA_stream.cpp":(work/"planeA_stream_20261021.cpp").read_bytes(),
    "planeA_raw_source.py":(work/"planeA_raw_source_20261021.py").read_bytes(),
    "preview_capture.py":(work/"preview_capture_a_20261021.py").read_bytes(),
    "preview_disarm_check.py":(work/"preview_disarm_check_20261021.py").read_bytes(),
    "preview_runtime.py":(work/"preview_runtime_a_20261021.py").read_bytes(),
    "run_preview.sh":(work/"run_preview_capture_a_20261021.sh").read_bytes(),
    "run_disarm.sh":(work/"run_preview_disarm_a_20261021.sh").read_bytes()}
for name,data in sources.items():
    if name.endswith(".py"):ast.parse(data.decode("utf-8"),filename=name)
    if name.endswith(".sh") and (b"\r" in data or data.startswith(b"\xef\xbb\xbf")):raise RuntimeError("shell encoding")
hashes={name:hashlib.sha256(data).hexdigest() for name,data in sources.items()}
bundle=hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest()
remote="/home/nvidia/payload_catch_mocap_preview_20261021/"+bundle[:16]
runid=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
remote_logs=remote+"/logs/"+runid
local_logs=work/("preview_on_a_"+runid)
local_logs.mkdir()
(local_logs/"deployment.json").write_text(json.dumps(dict(remote=remote,logs=remote_logs,sha256=hashes,bundle=bundle),indent=2),encoding="utf-8")

def run(args,name,data=None,timeout=45):
    p=subprocess.run(base+[shlex.join(args)],cwd=work,input=data,capture_output=True,timeout=timeout,shell=False)
    (local_logs/(name+".stdout.log")).write_bytes(p.stdout)
    (local_logs/(name+".stderr.log")).write_bytes(p.stderr)
    if p.returncode:
        print(p.stdout.decode("utf-8",errors="replace"),flush=True)
        print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr,flush=True)
        raise RuntimeError(name+" exit="+str(p.returncode))
    return p.stdout

payload=["from pathlib import Path","import ast, hashlib, json, subprocess","root=Path("+repr(remote)+")",
         "assert Path('/proc/device-tree/serial-number').read_text().strip('\\x00\\n')=='1612425621719'",
         "root.mkdir(parents=True,exist_ok=True)"]
for name,data in sources.items():
    payload.extend(["p=root/"+repr(name),"p.parent.mkdir(parents=True,exist_ok=True)",
                    "data="+repr(data),
                    "assert not p.exists() or p.read_bytes()==data, 'immutable stage mismatch'",
                    "p.write_bytes(data)",
                    "assert hashlib.sha256(p.read_bytes()).hexdigest()=="+repr(hashes[name])])
    if name.endswith(".py"):payload.append("ast.parse(p.read_text(encoding='utf-8'))")
    if name.endswith(".sh"):payload.append("subprocess.run(['/bin/bash','-n',str(p)],check=True,timeout=5)")
payload.extend(["subprocess.run(['/usr/bin/g++','-std=c++17','-I/opt/ros/humble/include','-fsyntax-only',str(root/'planeA_stream.cpp')],check=True,timeout=15)", "subprocess.run(['/usr/bin/g++','-std=c++17','-I/opt/ros/humble/include',str(root/'planeA_stream.cpp'),'-L/opt/ros/humble/lib','-lvrpn','-lquat','-lpthread','-o',str(root/'planeA_stream')],check=True,timeout=20)"])
payload.append("print(json.dumps({'verified_files':"+str(len(sources))+",'root':str(root)}))")
upload="\n".join(payload)+"\n"
ast.parse(upload)
print(run(["/usr/bin/python3","-"],"upload_verified",upload.encode()).decode(),flush=True)
print(run(["/bin/bash",remote+"/run_disarm.sh",remote],"disarmed_gate",timeout=25).decode(),flush=True)
try:
    print(run(["/usr/bin/timeout","--signal=INT","--kill-after=12s","55s","/bin/bash",remote+"/run_preview.sh",remote,remote_logs],"preview_run",timeout=75).decode(),flush=True)
finally:
    for name in ["capture.json","capture.log","poses.jsonl","processes.json","vrpn.log","preview.log"]:
        p=subprocess.run(base+[shlex.join(["/bin/cat",remote_logs+"/"+name])],cwd=work,capture_output=True,timeout=15,shell=False)
        (local_logs/name).write_bytes(p.stdout)
        (local_logs/(name+".fetch_stderr.log")).write_bytes(p.stderr)
        if p.returncode:print("fetch failed",name,p.returncode,p.stderr.decode(errors="replace"),file=sys.stderr)
print("RESULT_DIRECTORY",str(local_logs),flush=True)
print((local_logs/"capture.json").read_text(encoding="utf-8"),flush=True)

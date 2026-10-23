"""Stage/compile bounded VRPN user-message timing probe and retain evidence."""
import ast
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
WORK=Path(__file__).resolve().parent
SSH=[r"C:/Windows/System32/OpenSSH/ssh.exe","-b","192.168.1.123","-o","BatchMode=yes",
     "-o","ConnectTimeout=6","-o","ServerAliveInterval=5","-o","ServerAliveCountMax=2",
     "-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186",
     "-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.101"]
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
data=(WORK/"vrpn_message_time_probe.cpp").read_bytes()
digest=hashlib.sha256(data).hexdigest()
remote="/home/nvidia/payload_catch_mocap_fusion_20261023/message_time_"+digest[:16]
local=WORK/("message_time_"+datetime.now().strftime("%Y%m%dT%H%M%S"))
local.mkdir()
code=("from pathlib import Path\nimport hashlib,subprocess\n"
      "assert Path('/proc/device-tree/serial-number').read_text().strip('\\0\\n')=='1612425621719'\n"
      "root=Path("+repr(remote)+")\nroot.mkdir(parents=True,exist_ok=True)\n"
      "data="+repr(data)+"\np=root/'probe.cpp'\n"
      "assert not p.exists() or p.read_bytes()==data\np.write_bytes(data)\n"
      "assert hashlib.sha256(p.read_bytes()).hexdigest()=="+repr(digest)+"\n"
      "subprocess.run(['/usr/bin/g++','-std=c++17','-I/opt/ros/humble/include','-fsyntax-only',str(p)],check=True,timeout=15)\n"
      "subprocess.run(['/usr/bin/g++','-std=c++17','-I/opt/ros/humble/include',str(p),"
      "'-L/opt/ros/humble/lib','-lvrpn','-lquat','-lpthread','-o',str(root/'probe')],check=True,timeout=20)\n")
ast.parse(code)
(local/"stage_remote.py").write_text(code,encoding="utf-8")
(local/"manifest.json").write_text(json.dumps({"remote":remote,"sha256":digest}),encoding="utf-8")
for label,args,stdin,timeout in [
    ("stage",["/usr/bin/python3","-"],code.encode(),40),
    ("probe",["/usr/bin/timeout","--signal=TERM","--kill-after=3s","12s",remote+"/probe"],None,20)]:
    p=subprocess.run(SSH+[shlex.join(args)],input=stdin,cwd=WORK,capture_output=True,
                     timeout=timeout,shell=False)
    (local/(label+".stdout.log")).write_bytes(p.stdout)
    (local/(label+".stderr.log")).write_bytes(p.stderr)
    print(label,"exit",p.returncode,flush=True)
    print(p.stdout.decode("utf-8",errors="replace"),flush=True)
    print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
    if p.returncode:raise SystemExit(p.returncode)
    if label=="probe":
        rows=[json.loads(line) for line in p.stdout.decode("utf-8").splitlines() if line]
        (local/"result.json").write_text(json.dumps(rows,indent=2),encoding="utf-8")
print("EVIDENCE",local)

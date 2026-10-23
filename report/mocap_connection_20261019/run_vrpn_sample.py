from pathlib import Path
import json
import subprocess
import sys
from attempt_wifi import ssh
work=Path(__file__).resolve().parent
remote="/home/nvidia/payload_catch_a_solo_20261019/mocap_check"
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
def run(argv,name,input=None,timeout=15):
    p=ssh("192.168.1.4",argv,input=input,timeout=timeout)
    (work/(name+".stdout.log")).write_text(p.stdout,encoding="utf-8")
    (work/(name+".stderr.log")).write_text(p.stderr,encoding="utf-8")
    print(p.stdout)
    print(p.stderr,file=sys.stderr)
    if p.returncode:raise SystemExit(p.returncode)
    return p.stdout
run(["mkdir","-p",remote],"sample_mkdir")
for name in ("sample_vrpn.py","sample_vrpn.sh"):
    p=ssh("192.168.1.4",["tee",remote+"/"+name],input=(work/name).read_text(encoding="utf-8"))
    if p.returncode:raise SystemExit(p.stderr)
run(["/bin/bash","-n",remote+"/sample_vrpn.sh"],"sample_bash_syntax")
run(["/usr/bin/python3","-c","import ast,pathlib; ast.parse(pathlib.Path("+repr(remote+"/sample_vrpn.py")+").read_text(encoding='utf-8'))"],"sample_python_syntax")
run(["/bin/bash",remote+"/sample_vrpn.sh"],"raw_vrpn_sample",timeout=50)
data=json.loads(run(["cat",remote+"/raw_vrpn_result.json"],"raw_vrpn_result"))
(work/"raw_vrpn_result.json").write_text(json.dumps(data,indent=2)+"\n",encoding="utf-8")

from pathlib import Path
import json
import sys
from attempt_wifi import ssh
work=Path(__file__).resolve().parent
remote="/home/nvidia/payload_catch_a_solo_20261019/mocap_check"
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
def run(argv,name,input=None,timeout=20):
    p=ssh("192.168.1.4",argv,input=input,timeout=timeout)
    (work/(name+".stdout.log")).write_text(p.stdout,encoding="utf-8")
    (work/(name+".stderr.log")).write_text(p.stderr,encoding="utf-8")
    print(p.stdout,flush=True)
    print(p.stderr,file=sys.stderr,flush=True)
    if p.returncode:raise SystemExit(p.returncode)
    return p.stdout
run(["tee",remote+"/raw_vrpn_probe.cpp"],"upload_native_probe",(work/"raw_vrpn_probe.cpp").read_text(encoding="utf-8"))
common=["/usr/bin/g++","-std=c++17","-I/opt/ros/humble/include"]
run(common+["-fsyntax-only",remote+"/raw_vrpn_probe.cpp"],"native_probe_syntax")
run(common+[remote+"/raw_vrpn_probe.cpp","-L/opt/ros/humble/lib","-lvrpn","-lquat","-lpthread","-o",remote+"/raw_vrpn_probe"],"native_probe_build")
output=run([remote+"/raw_vrpn_probe"],"native_vrpn_probe",timeout=25)
result=json.loads(output)
(work/"native_vrpn_result.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")

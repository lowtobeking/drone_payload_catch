from pathlib import Path
import json
import subprocess
result={}
for name,args in (
    ("wifi",["nmcli","-g","GENERAL.CONNECTION,IP4.ADDRESS","dev","show","wlP1p1s0"]),
    ("profile",["nmcli","-g","ipv4.routes,connection.autoconnect","con","show","payload-catch-Aero-LAB_5G"]),
    ("rollback_timer",["systemctl","is-active","payload-catch-wifi-rollback-5bf25dec.timer"]),
):
    p=subprocess.run(args,cwd="/home/nvidia",timeout=10,capture_output=True,text=True,encoding="utf-8",errors="replace")
    result[name]=dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
files={}
base=Path("/opt/ros/humble/share/vrpn_mocap")
for p in base.rglob("*"):
    if p.is_file() and p.suffix in (".yaml",".py") and ("launch" in str(p) or "config" in str(p)):
        files[str(p)]=p.read_text(encoding="utf-8")[:16000]
result["files"]=files
result["executables"]=[str(p) for p in Path("/opt/ros/humble/lib/vrpn_mocap").iterdir()]
print(json.dumps(result,indent=2))

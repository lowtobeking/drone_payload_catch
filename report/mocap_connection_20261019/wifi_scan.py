"""Targeted Wi-Fi discovery; does not disconnect current connection."""
import json
import subprocess
import time
result={}
for name,args in (
    ("target_scan",["sudo","-n","nmcli","device","wifi","rescan","ifname","wlP1p1s0","ssid","Aero-LAB_5G"]),
    ("radio",["nmcli","radio"]),
):
    p=subprocess.run(args,cwd="/home/nvidia",timeout=20,capture_output=True,text=True,encoding="utf-8",errors="replace",shell=False)
    result[name]=dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
time.sleep(4)
p=subprocess.run(["nmcli","-t","-f","SSID,SECURITY,SIGNAL,FREQ","dev","wifi","list","ifname","wlP1p1s0","--rescan","no"],cwd="/home/nvidia",timeout=10,capture_output=True,text=True,encoding="utf-8",errors="replace",shell=False)
result["visible"]=dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
print(json.dumps(result,indent=2))

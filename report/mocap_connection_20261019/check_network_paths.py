"""Read-only route diagnostics; no network configuration changes."""
from pathlib import Path
import json
import socket
import subprocess
result={}
for name,args in (
    ("saved_connections",["nmcli","-t","-f","NAME,UUID,TYPE,DEVICE","connection","show"]),
    ("server_ping",["ping","-c","2","-W","1","192.168.1.111"]),
    ("lab_gateway_ping",["ping","-c","2","-W","1","192.168.1.1"]),
    ("neighbors",["ip","neighbor","show","dev","enx00e04c680065"]),
    ("lab_profiles",["nmcli","-g","802-11-wireless.ssid","connection","show","Aero-LAB_5G"]),
):
    p=subprocess.run(args,cwd="/home/nvidia",timeout=10,capture_output=True,text=True,encoding="utf-8",errors="replace",shell=False)
    result[name]=dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
print(json.dumps(result,indent=2))

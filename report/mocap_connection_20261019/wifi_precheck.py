"""Read-only pre-switch checks; never print Wi-Fi secrets."""
from pathlib import Path
import json
import subprocess
import socket
import fcntl
assert socket.gethostname()=="nvidia-desktop"
assert Path("/proc/device-tree/serial-number").read_text().strip("\0\n")=="1612425621719"
base=Path("/home/nvidia/payload_catch_deploy")
with (base/"mission.lock").open("r+") as lock:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
rows={}
for name,args in (
    ("wifi",["nmcli","-t","-f","IN-USE,SSID,SECURITY,SIGNAL","dev","wifi","list","ifname","wlP1p1s0","--rescan","yes"]),
    ("profile",["nmcli","-f","connection.id,connection.uuid,connection.autoconnect,ipv4.route-metric,ipv4.routes","con","show","lq-mesh"]),
    ("routes",["ip","route","show"]),
    ("sudo",["sudo","-n","true"]),
    ("nm",["nmcli","--version"]),
):
    p=subprocess.run(args,cwd="/home/nvidia",timeout=20,capture_output=True,text=True,encoding="utf-8",errors="replace",shell=False)
    rows[name]=dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
print(json.dumps(rows,indent=2))

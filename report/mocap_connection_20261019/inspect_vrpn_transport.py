from pathlib import Path
import json
import subprocess
result={}
patterns=("vrpn_get_connection_by_name","tcp://","NIC","local_ip","vrpn_Tracker_Remote(")
for name in ("vrpn_Connection.h","vrpn_Tracker.h"):
    lines=Path("/opt/ros/humble/include",name).read_text(encoding="utf-8").splitlines()
    indices=set()
    for index,line in enumerate(lines):
        if any(pattern in line for pattern in patterns):
            indices.update(range(max(0,index-4),min(len(lines),index+24)))
    result[name]="\n".join(str(i+1)+": "+lines[i] for i in sorted(indices))
for name,args in (
    ("routes",["ip","route","show"]),
    ("firewall",["sudo","-n","ufw","status"]),
    ("capture",["which","tcpdump"]),
):
    try:
        p=subprocess.run(args,cwd="/home/nvidia",timeout=10,capture_output=True,text=True,encoding="utf-8",errors="replace")
        result[name]=dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
    except FileNotFoundError as error:result[name]=dict(error=str(error))
print(json.dumps(result,indent=2))

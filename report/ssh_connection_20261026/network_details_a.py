"""Read-only details needed to choose a minimal compatible route repair."""
import json
from pathlib import Path
import subprocess
def run(a):
    p=subprocess.run(a,capture_output=True,text=True,timeout=15,shell=False)
    return {"exit":p.returncode,"stdout":p.stdout,"stderr":p.stderr}
print(json.dumps({
"wifi_neighbors":run(["ip","-j","neigh","show","dev","wlP1p1s0"]),
"main_v4":run(["ip","-j","-4","route","show","table","main"]),
"rule_support":run(["sudo","-n","ip","-4","rule","show"]),
"processes":run(["ps","-p","1155,1156","-o","pid,ppid,lstart,args"]),
"service_units":run(["systemctl","list-units","--type=service","--all","--no-pager","--plain"]),
"network_profiles":run(["sudo","-n","find","/etc/NetworkManager/system-connections","-maxdepth","1","-type","f","-printf","%f\n"]),
"fc_route":run(["ip","-j","route","get","10.41.10.1"]),
"qgc_route":run(["ip","-j","route","get","192.168.1.203"]),
"mocap_route":run(["ip","-j","route","get","192.168.1.111"]),
},indent=2))

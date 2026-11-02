"""Read-only A network and service audit; no FC commands."""
import hashlib
import json
from pathlib import Path
import subprocess
import socket
import sys
def run(args):
    p=subprocess.run(args,capture_output=True,text=True,timeout=15,shell=False)
    return {"exit":p.returncode,"stdout":p.stdout,"stderr":p.stderr}
serial=Path("/proc/device-tree/serial-number").read_bytes().rstrip(bytes([0])).decode()
if serial!="1612425621719":
    raise SystemExit("Wrong Jetson")
data={"serial":serial,"hostname":socket.gethostname(),"boot_id":Path("/proc/sys/kernel/random/boot_id").read_text().strip()}
commands={
"addresses":["ip","-j","address","show"],
"routes":["ip","-j","route","show","table","all"],
"rules":["ip","-j","rule","show"],
"nm_version":["nmcli","--version"],
"ip_version":["ip","-V"],
"profiles":["nmcli","-t","-f","NAME,UUID,TYPE,DEVICE","connection","show","--active"],
"wifi_ipv4":["nmcli","-f","GENERAL.CONNECTION,GENERAL.CON-UUID,IP4.ADDRESS,IP4.GATEWAY,IP4.ROUTE","device","show","wlP1p1s0"],
"route_wifi_client":["ip","route","get","192.168.1.4","from","192.168.1.5"],
"rp_filter":["sysctl","net.ipv4.conf.all.rp_filter","net.ipv4.conf.wlP1p1s0.rp_filter"],
"services":["systemctl","show","ssh","microxrce-agent","mav-relay","-p","Id","-p","ActiveState","-p","MainPID","-p","ExecMainStartTimestamp"],
"sudo":["sudo","-n","true"],
"listeners":["ss","-lnptu"],
"fc_neighbor":["ip","-j","neigh","show","dev","eno1"],
"current_release":["readlink","-f","/home/nvidia/payload_catch_deploy/current"],
"dispatcher":["ls","-l","/etc/NetworkManager/dispatcher.d"],
}
data["checks"]={key:run(args) for key,args in commands.items()}
data["config_hashes"]={}
for p in [Path("/etc/ssh/sshd_config"),Path("/etc/NetworkManager/NetworkManager.conf"),Path("/boot/extlinux/extlinux.conf")]:
    if p.is_file():
        data["config_hashes"][str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
print(json.dumps(data,ensure_ascii=False,indent=2))

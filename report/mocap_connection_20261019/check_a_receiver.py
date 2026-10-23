"""Read-only A network/client inventory and VRPN TCP handshake check."""
from pathlib import Path
import json
import os
import socket
import subprocess
import importlib.util
result={"hostname":socket.gethostname()}
result["serial"]=Path("/proc/device-tree/serial-number").read_text().strip("\0\n")
assert result["hostname"]=="nvidia-desktop" and result["serial"]=="1612425621719"
for name,args in (
    ("addresses",["ip","-brief","address"]),
    ("server_route",["ip","route","get","192.168.1.111"]),
    ("wifi",["nmcli","-t","-f","ACTIVE,SSID,DEVICE","dev","wifi"]),
    ("sockets",["ss","-tunp"]),
    ("packages",["dpkg-query","-W","*vrpn*","*mocap*"]),
):
    p=subprocess.run(args,cwd="/home/nvidia",timeout=12,capture_output=True,text=True,encoding="utf-8",errors="replace",shell=False)
    result[name]=dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
result["vrpn_tcp"]={}
try:
    with socket.create_connection(("192.168.1.111",3883),timeout=4) as connection:
        result["vrpn_tcp"]["connected"]=True
        result["vrpn_tcp"]["local_address"]=connection.getsockname()
        connection.settimeout(3)
        try: result["vrpn_tcp"]["server_greeting"]=repr(connection.recv(128))
        except socket.timeout: result["vrpn_tcp"]["server_greeting"]="no greeting within 3s"
except OSError as error:
    result["vrpn_tcp"]=dict(connected=False,error=str(error))
processes=[]
for p in Path("/proc").iterdir():
    if not p.name.isdigit() or int(p.name)==os.getpid():continue
    try:args=(p/"cmdline").read_bytes().replace(b"\0",b" ").decode(errors="replace")
    except OSError:continue
    if any(x in args.lower() for x in ("vrpn","mocap","visual_odometry","solo_validation","run_fleet")):
        processes.append(dict(pid=p.name,args=args))
result["relevant_processes"]=processes
files=[]
for base in (Path("/home/nvidia"),Path("/opt/ros/humble"),Path("/usr/local")):
    for directory,subdirs,names in os.walk(base):
        subdirs[:]=[n for n in subdirs if n not in (".git",".cache","report","log","build","releases","node_modules","pwm_pin33_safety",".local")]
        if len(Path(directory).relative_to(base).parts)>7:
            subdirs[:]=[]
            continue
        for name in subdirs+names:
            if "vrpn" in name.lower() or "mocap" in name.lower():
                files.append(str(Path(directory)/name))
        if len(files)>200:break
result["mocap_files"]=files
result["python_vrpn_available"]=importlib.util.find_spec("vrpn") is not None
print(json.dumps(result,indent=2))

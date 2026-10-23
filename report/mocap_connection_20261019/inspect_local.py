"""Read-only WSL package inventory; no ROS nodes started."""
from pathlib import Path
import json
import os
import subprocess

root=Path(__file__).resolve().parents[2]
result={}
for name,args in (
    ("addresses",["ip","-j","address","show"]),
    ("routes",["ip","-j","route","show"]),
    ("vrpn_port",["ss","-lntup","sport = :3883"]),
):
    p=subprocess.run(args,cwd=root,timeout=10,capture_output=True,text=True,encoding="utf-8",errors="replace",shell=False)
    result[name]=dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
paths=[Path("/opt/ros/humble/share/ament_index/resource_index/packages"),Path.home()/"ros2_ws/install",Path("/usr/local/include/vrpn_Tracker.h"),Path("/usr/local/lib/libvrpn.so")]
result["known_paths"]={str(p):p.exists() for p in paths}
files=[]
for base in (Path.home(),Path("/opt/ros/humble")):
    for directory,subdirs,names in os.walk(base):
        subdirs[:]=[n for n in subdirs if n not in (".git",".cache","node_modules",".venv","PX4-Autopilot","report","log","build")]
        depth=len(Path(directory).relative_to(base).parts)
        if depth>7:
            subdirs[:]=[]
            continue
        for n in subdirs+names:
            if "vrpn" in n.lower() or "mocap" in n.lower():
                files.append(str(Path(directory)/n))
                if len(files)>=100:
                    break
        if len(files)>=100:
            break
result["mocap_paths"]=files
out=root/"report/mocap_connection_20261019/local_status.json"
out.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
print(json.dumps(result,indent=2))

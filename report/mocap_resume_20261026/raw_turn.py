"""Bounded raw-only native VRPN experiment; no ROS publishers or FC API."""
import ast
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import sys
WORK=Path(__file__).resolve().parent
SSH=["C:/Windows/System32/OpenSSH/ssh.exe","-F","NUL","-b","192.168.1.4","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.5","/usr/bin/python3 -"]
def main():
    sys.stdout.reconfigure(encoding="utf-8")
    action=sys.argv[1]
    if action=="start":
        stamp=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        remote="/home/nvidia/payload_catch_mocap_fusion_20261026/raw_turn_"+stamp
        unit="payload-catch-raw-turn-"+stamp.lower()
        code="from pathlib import Path\nimport json,subprocess\n"
        code+="assert Path('/proc/device-tree/serial-number').read_bytes().rstrip(bytes([0])).decode()=='1612425621719'\n"
        code+="p=Path("+repr(remote)+");p.mkdir(parents=True,exist_ok=False)\n"
        cmd=["sudo","-n","systemd-run","--unit="+unit,"--property=User=nvidia",
             "--property=RuntimeMaxSec=130","--property=StandardOutput=file:"+remote+"/raw.jsonl",
             "--property=StandardError=file:"+remote+"/native.stderr.log",
             "/usr/bin/env","LD_LIBRARY_PATH=/opt/ros/humble/lib",
             "/home/nvidia/payload_catch_mocap_fusion_20261023/lift_reference_8045b3c26fc236e6/mocap_vrpn_reader",
             "192.168.1.111:3883","planeA","192.168.1.5","120"]
        code+="p=subprocess.run("+repr(cmd)+",capture_output=True,text=True,timeout=15)\nprint(json.dumps(dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)))\nraise SystemExit(p.returncode)\n"
        manifest=dict(remote=remote,unit=unit,raw_only=True,seconds=120,stamp=stamp)
        target=WORK/("raw_turn_"+stamp);target.mkdir()
        (target/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    elif action in ("status","fetch"):
        target=sorted(WORK.glob("raw_turn_20*"))[-1]
        manifest=json.loads((target/"manifest.json").read_text())
        code="from pathlib import Path\nimport json,subprocess\nroot=Path("+repr(manifest["remote"])+")\n"
        if action=="status":
            code+="p=root/'raw.jsonl'\nlines=p.read_text().splitlines() if p.exists() else []\n"
            code+="s=subprocess.run(['systemctl','is-active',"+repr(manifest["unit"])+"],capture_output=True,text=True,timeout=8)\n"
            code+="print(json.dumps(dict(active=s.stdout.strip(),frames=len(lines),last=json.loads(lines[-1]) if lines else None)))\n"
        else:
            code+="print((root/'raw.jsonl').read_text(),end='')\n"
    else:raise ValueError(action)
    ast.parse(code)
    p=subprocess.run(SSH,input=code.encode(),cwd=WORK,capture_output=True,timeout=25,shell=False)
    (target/(action+".stdout.log")).write_bytes(p.stdout)
    (target/(action+".stderr.log")).write_bytes(p.stderr)
    if action=="fetch" and not p.returncode:
        (target/"raw.jsonl").write_bytes(p.stdout)
        print("Saved raw capture",target,"bytes",len(p.stdout))
    else:
        print(p.stdout.decode("utf-8",errors="replace"))
    print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
    sys.exit(p.returncode)
if __name__=="__main__":main()

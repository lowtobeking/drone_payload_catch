"""Read-only precheck: existing disarm observer and bounded native planeA capture."""
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import time
serial=Path("/proc/device-tree/serial-number").read_bytes().rstrip(bytes([0])).decode()
if serial!="1612425621719":raise RuntimeError("Wrong A")
stamp=datetime.now().strftime("%Y%m%dT%H%M%S")
work=Path("/home/nvidia/payload_catch_mocap_fusion_20261026")/("precheck_"+stamp)
work.mkdir(parents=True,exist_ok=False)
def run(args,label,timeout,env=None):
    p=subprocess.run(args,cwd=work,timeout=timeout,capture_output=True,shell=False,env=env)
    (work/(label+".stdout.log")).write_bytes(p.stdout)
    (work/(label+".stderr.log")).write_bytes(p.stderr)
    return p
out={"serial":serial,"jetson_boot":Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
     "wall":datetime.now().astimezone().isoformat(),"remote_logs":str(work),"read_only":True}
old="/home/nvidia/payload_catch_mocap_preview_20261021/9695526c87f980d0"
p=run(["/bin/bash",old+"/run_disarm.sh",old],"disarm",30)
out["disarm"]={"exit":p.returncode,"stdout":p.stdout.decode(errors="replace"),"stderr":p.stderr.decode(errors="replace")}
if p.returncode:
    print(json.dumps(out,indent=2));raise SystemExit(p.returncode)
reader=Path("/home/nvidia/payload_catch_mocap_fusion_20261023/lift_reference_8045b3c26fc236e6/mocap_vrpn_reader")
env=dict(os.environ,LD_LIBRARY_PATH="/opt/ros/humble/lib:"+os.environ.get("LD_LIBRARY_PATH",""))
p=run([str(reader),"192.168.1.111:3883","planeA","192.168.1.5","8"],"planeA",16,env)
out["vrpn"]={"exit":p.returncode,"stderr":p.stderr.decode(errors="replace"),"reader_sha256":hashlib.sha256(reader.read_bytes()).hexdigest()}
rows=[json.loads(line) for line in p.stdout.splitlines() if line.startswith(b"{")]
out["vrpn"]["frames"]=len(rows)
if len(rows)>1:
    m=[r["receive_monotonic_us"] for r in rows]
    tail=[r for r in rows if r["receive_monotonic_us"]>m[-1]-4000000]
    out["vrpn"].update(hz=(len(rows)-1)*1e6/(m[-1]-m[0]),
        max_gap_ms=max(b-a for a,b in zip(m,m[1:]))/1000,
        max_gap_last4_ms=max(b["receive_monotonic_us"]-a["receive_monotonic_us"] for a,b in zip(tail,tail[1:]))/1000,
        last=rows[-1],source_minus_receive_s=(rows[-1]["server_stamp_us"]-rows[-1]["receive_wall_us"])/1e6)
out["passed"]=p.returncode==0 and len(rows)>200
(work/"summary.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf-8")
print(json.dumps(out,indent=2),flush=True)
raise SystemExit(0 if out["passed"] else 2)

"""Run relevant offline checks without starting ROS or accessing aircraft."""
from pathlib import Path
import json
import os
import subprocess
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
names=["test_mocap_transform.py","test_mocap_odometry.py","test_mocap_receive_ground.py","test_mocap_fixed_reference.py","test_mocap_height.py","test_mocap_flight_bridge.py","test_a_solo.py"]
env=dict(os.environ,PYTHONPATH=str(ROOT),PYTHONDONTWRITEBYTECODE="1")
results=[]
for name in names:
    p=subprocess.run(["/usr/bin/python3","-B",str(ROOT/"tools"/name)],cwd=ROOT,env=env,capture_output=True,text=True,timeout=90)
    (OUT/(name+".stdout.log")).write_text(p.stdout,encoding="utf-8")
    (OUT/(name+".stderr.log")).write_text(p.stderr,encoding="utf-8")
    results.append({"test":name,"exit":p.returncode,"result_lines":p.stderr.splitlines()[-7:]})
(OUT/"offline_results.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
print(json.dumps(results,indent=2))
raise SystemExit(1 if any(r["exit"] for r in results) else 0)

"""Clone the verified bounded lift experiment; only current transport/bind/log paths change."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[2]
WORK=Path(__file__).resolve().parent
OLD=ROOT/"report/mocap_lift_20261023"
records={}
for name in ["fixed_runtime_a.py","fixed_guard_a.py","run_fixed_a.sh","run_fixed_on_a.py","control.py"]:
    source=(OLD/name).read_text(encoding="utf-8")
    updated=source
    if name=="fixed_runtime_a.py":
        old='"--bind-ip","192.168.1.4"'
        if updated.count(old)!=1:raise RuntimeError("Missing bind IP")
        updated=updated.replace(old,'"--bind-ip","192.168.1.5"')
    elif name=="run_fixed_on_a.py":
        updated=updated.replace('"192.168.1.123"','"192.168.1.4"')
        updated=updated.replace('"nvidia@192.168.1.4"','"nvidia@192.168.1.5"')
        updated=updated.replace('"/home/nvidia/payload_catch_mocap_fusion_20261023/lift_reference_"','"/home/nvidia/payload_catch_mocap_fusion_20261026/lift_reference_"')
    if name.endswith(".py"):ast.parse(updated,name)
    if name.endswith(".sh"):
        if "\r" in updated or updated.startswith("\ufeff"):raise RuntimeError("Bash encoding")
    target=WORK/name
    if target.exists():raise RuntimeError("Refusing overwrite "+name)
    target.write_text(updated,encoding="utf-8",newline="\n")
    records[name]={"source":str((OLD/name).relative_to(ROOT)),"original_sha256":hashlib.sha256(source.encode()).hexdigest(),"sha256":hashlib.sha256(target.read_bytes()).hexdigest(),"changed":source!=updated}
(WORK/"source_reuse.json").write_text(json.dumps(records,indent=2)+"\n",encoding="utf-8")
print(json.dumps(records,indent=2))

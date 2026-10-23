"""Archive task-owned one-off editors; keep sources, tests, logs and operational runners."""
from pathlib import Path
import json
import shutil
ROOT=Path(__file__).resolve().parent
assert str(ROOT)=="/home/caolihao/drone_payload_catch/report/fixed_reference_20261023"
target=ROOT/"diagnostic_helpers"
target.mkdir(exist_ok=True)
names=["prepare_run.py","fix_reconnect.py","improve_restart_probe.py","refine_evidence.py",
       "fix_ros_restart.py","batch_proof.py","refine_post_audit.py"]
moved=[]
for name in names:
    source=ROOT/name
    dest=target/name
    assert source.resolve().parent==ROOT and dest.resolve().parent==target
    if source.exists():
        if dest.exists():raise RuntimeError("archive destination already exists")
        shutil.move(str(source),str(dest));moved.append(name)
(ROOT/"helper_archive.json").write_text(json.dumps(dict(moved=moved,destination=str(target)),indent=2),encoding="utf-8")
print("Archived",len(moved),"task-owned one-off editors; no system processes or services modified")

"""Create a manually started, non-autostart ground-fusion session launcher."""
import ast
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[2]
ROOT=PROJECT/"report/mocap_continuous_20261026"
OLD=PROJECT/"report/mocap_right_translation_20261026"
runtime=PROJECT/"tools/mocap_ground_session_a.py"
content=runtime.read_text(encoding="utf-8")
content=content.replace("import json\n","import json\nimport fcntl\n",1)
anchor='        assert os.environ.get("ROS_DOMAIN_ID")=="0"'
assert content.count(anchor)==1
content=content.replace(anchor,anchor+'''
        lock_handle=Path("/home/nvidia/payload_catch_mocap_ground.lock").open("a")
        handles.append(lock_handle)
        fcntl.flock(lock_handle.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
''',1)
ast.parse(content)
runtime.write_text(content,encoding="utf-8",newline="\n")
runner=(OLD/"run_fixed_on_a.py").read_text(encoding="utf-8")
runner=runner.replace('(WORK / "fixed_runtime_a.py").read_bytes()','(PROJECT / "tools/mocap_ground_session_a.py").read_bytes()')
runner=runner.replace('sources["fixed_guard_a.py"] = (WORK / "fixed_guard_a.py").read_bytes()','sources["mocap_ground_guard_a.py"] = (PROJECT / "tools/mocap_ground_guard_a.py").read_bytes()')
runner=runner.replace('(WORK / "run_fixed_a.sh").read_bytes()','(WORK / "run_fixed_a.sh").read_bytes()')
runner=runner.replace('/right_translation_','/continuous_ground_')
runner=runner.replace('payload-catch-right-translation-','payload-catch-mocap-ground-')
runner=runner.replace('Bounded disarmed mocap right-translation acceptance','Continuous disarmed ground mocap fusion; manual start only')
runner=runner.replace('"--property=RuntimeMaxSec=950",','')
runner=runner.replace('        "/usr/bin/timeout","--signal=TERM","--kill-after=660s","310s",\n','')
runner=runner.replace('"automatic_rollback":True','"automatic_rollback":True,"continuous_ground":True,"autostart":False')
ast.parse(runner)
(ROOT/"run_fixed_on_a.py").write_text(runner,encoding="utf-8",newline="\n")
for name in ("run_fixed_a.sh","final_audit_a.py"):
    content=(OLD/name).read_text(encoding="utf-8")
    (ROOT/name).write_text(content,encoding="utf-8",newline="\n")
print("Continuous session launcher parsed; transient systemd unit, no boot enable")

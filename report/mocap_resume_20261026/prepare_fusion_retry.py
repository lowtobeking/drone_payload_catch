"""Prepare a separate retry, keeping prior failed evidence immutable."""
import ast
import json
from pathlib import Path
import subprocess

WORK=Path(__file__).resolve().parent
OLD=WORK.parent/"mocap_yaw_20261026"
names=["run_fixed_on_a.py","fixed_runtime_a.py","fixed_guard_a.py","run_fixed_a.sh","yaw_plateau.py","status.py","monitor.py","fetch_and_audit.py","verify_after.py","diagnose.py","stop_session.py"]
for name in names:
    content=(OLD/name).read_text(encoding="utf-8")
    if name=="run_fixed_on_a.py":
        content=content.replace('/auto_yaw_', '/resumed_yaw_').replace('payload-catch-ground-yaw-', 'payload-catch-resumed-yaw-')
    if name=="verify_after.py":
        content=content.replace('(OLD/"final_audit_a.py").read_bytes()', '(WORK/"final_audit_a.py").read_bytes()')
        content=content.replace('result["passed"]=(', 'result["services_unchanged"]=state["services"]==first["services"]\n    result["current_unchanged"]=state["current"]==first["current"]\n    result["passed"]=(result["services_unchanged"] and result["current_unchanged"] and ')
    if name.endswith(".py"):
        ast.parse(content,name)
    (WORK/name).write_text(content,encoding="utf-8",newline="\n")
content=(WORK.parent/"mocap_fusion_20261023/final_audit_a.py").read_text(encoding="utf-8")
content=content.replace('prefix = "/home/nvidia/payload_catch_mocap_fusion_20261023/"', 'prefixes = ("/home/nvidia/payload_catch_mocap_fusion_20261023/", "/home/nvidia/payload_catch_mocap_fusion_20261026/")')
content=content.replace('if prefix in args:', 'if any(prefix in args for prefix in prefixes):')
content=content.replace('"payload-catch-mav-relay.service",', '"payload-catch-mav-relay.service", "payload-catch-wifi-peer.service",')
ast.parse(content)
(WORK/"final_audit_a.py").write_text(content,encoding="utf-8",newline="\n")
import importlib.util
spec=importlib.util.spec_from_file_location("runner",WORK/"run_fixed_on_a.py")
runner=importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
p=subprocess.run(runner.SSH+["/usr/bin/python3 -"],input=content.encode(),cwd=WORK,capture_output=True,timeout=25,shell=False)
(WORK/"identity.stderr.log").write_bytes(p.stderr)
if p.returncode:
    raise RuntimeError(p.stderr.decode(errors="replace"))
state=json.loads(p.stdout)
assert state["boot"]=="dc981513-5789-4a7c-ba91-57367486bfa9",state
assert not state["remaining_task_processes"],state
assert state["services"].count("ActiveState=active")==3,state
(WORK/"identity.json").write_text(json.dumps(state,indent=2),encoding="utf-8")
print(json.dumps(state,indent=2))

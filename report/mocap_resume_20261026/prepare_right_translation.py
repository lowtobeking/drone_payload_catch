"""Prepare a dedicated right-translation ground acceptance from proven forward tooling."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parent.parent
OLD=ROOT/"mocap_forward_20261026"
DEST=ROOT/"mocap_right_translation_20261026"
DEST.mkdir(exist_ok=False)
names=["run_fixed_on_a.py","fixed_runtime_a.py","fixed_guard_a.py","run_fixed_a.sh","yaw_plateau.py","status.py","monitor.py","fetch_and_audit.py","verify_after.py","diagnose.py","stop_session.py","final_audit_a.py"]
def once(content,old,new):
    assert content.count(old)==1,(old,content.count(old))
    return content.replace(old,new,1)
for name in names:
    content=(OLD/name).read_text(encoding="utf-8")
    if name=="run_fixed_on_a.py":
        content=once(content,"/forward_translation_","/right_translation_")
        content=once(content,"payload-catch-forward-","payload-catch-right-translation-")
        content=content.replace("forward-translation acceptance","right-translation acceptance")
    if name=="fixed_runtime_a.py":
        content=content.replace("forward","right")
        content=once(content,"right=delta[0]*math.cos(heading)+delta[1]*math.sin(heading)","right=-delta[0]*math.sin(heading)+delta[1]*math.cos(heading)")
        content=once(content,"cross=-delta[0]*math.sin(heading)+delta[1]*math.cos(heading)","cross=delta[0]*math.cos(heading)+delta[1]*math.sin(heading)")
        content=once(content,"input_right_m=inp[0]*math.cos(heading)+inp[1]*math.sin(heading)","input_right_m=-inp[0]*math.sin(heading)+inp[1]*math.cos(heading)")
        content=once(content,"px4_right_m=px4[0]*math.cos(heading)+px4[1]*math.sin(heading)","px4_right_m=-px4[0]*math.sin(heading)+px4[1]*math.cos(heading)")
        content=once(content,"input_cross_m=-inp[0]*math.sin(heading)+inp[1]*math.cos(heading)","input_cross_m=inp[0]*math.cos(heading)+inp[1]*math.sin(heading)")
    if name in ("status.py","fetch_and_audit.py"):
        content=content.replace("forward_move.json","right_move.json")
    if name.endswith(".py"):
        ast.parse(content,name)
    (DEST/name).write_text(content,encoding="utf-8",newline="\n")
spec=importlib.util.spec_from_file_location("runner",DEST/"run_fixed_on_a.py")
runner=importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
code=(DEST/"final_audit_a.py").read_bytes()
ast.parse(code)
p=subprocess.run(runner.SSH+["/usr/bin/python3 -"],input=code,cwd=DEST,capture_output=True,timeout=25,shell=False)
(DEST/"identity.stdout.log").write_bytes(p.stdout)
(DEST/"identity.stderr.log").write_bytes(p.stderr)
if p.returncode:
    raise RuntimeError("SSH audit exit "+str(p.returncode)+" "+p.stderr.decode(errors="replace"))
state=json.loads(p.stdout)
assert state["serial"]=="1612425621719" and not state["remaining_task_processes"],state
assert state["services"].count("ActiveState=active")==3,state
(DEST/"identity.json").write_text(json.dumps(state,indent=2),encoding="utf-8")
print(json.dumps({"prepared":str(DEST),"state":state},indent=2))

"""Create independent right-turn experiment from completed left-turn tooling."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
WORK=Path(__file__).resolve().parent
DEST=WORK.parent/"mocap_right_20261026"
DEST.mkdir(exist_ok=False)
names=["run_fixed_on_a.py","fixed_runtime_a.py","fixed_guard_a.py","run_fixed_a.sh","yaw_plateau.py","status.py","monitor.py","fetch_and_audit.py","verify_after.py","diagnose.py","stop_session.py","final_audit_a.py"]
def replace_once(text,old,new):
    assert text.count(old)==1,(old,text.count(old))
    return text.replace(old,new,1)
for name in names:
    content=(WORK/name).read_text(encoding="utf-8")
    if name=="run_fixed_on_a.py":
        content=replace_once(content,"/resumed_yaw_","/right_yaw_")
        content=replace_once(content,"payload-catch-resumed-yaw-","payload-catch-right-yaw-")
        content=content.replace("mocap yaw acceptance","mocap right-turn acceptance")
    if name=="yaw_plateau.py":
        content=content.replace("negative NED yaw is a left turn","positive NED yaw is a right turn")
        content=content.replace("stable_left","stable_right")
        content=replace_once(content,"-120<sum(deltas)/len(deltas)<-30","30<sum(deltas)/len(deltas)<120")
    if name=="fixed_runtime_a.py":
        content=content.replace("stable_left","stable_right").replace("left_turn","right_turn").replace("left-turn","right-turn")
        content=content.replace("rotate level left","rotate level right")
        content=replace_once(content,"-120<input_delta<-30 and -120<px4_delta<-30","30<input_delta<120 and 30<px4_delta<120")
        content=content.replace('stage="await_lift"','stage="await_right_turn"')
    if name in ("status.py","fetch_and_audit.py"):
        content=content.replace("left_turn.json","right_turn.json")
    if name=="fetch_and_audit.py":
        content=content.replace('"right_turn.json","returned.json",','"right_turn.json",')
    if name.endswith(".py"):
        ast.parse(content,name)
    (DEST/name).write_text(content,encoding="utf-8",newline="\n")
spec=importlib.util.spec_from_file_location("runner",DEST/"run_fixed_on_a.py")
runner=importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
code=(DEST/"final_audit_a.py").read_bytes()
ast.parse(code)
p=subprocess.run(runner.SSH+["/usr/bin/python3 -"],input=code,cwd=DEST,capture_output=True,timeout=25,shell=False)
(DEST/"identity.stderr.log").write_bytes(p.stderr)
(DEST/"identity.stdout.log").write_bytes(p.stdout)
if p.returncode:
    raise RuntimeError("SSH audit exit "+str(p.returncode)+" "+p.stderr.decode(errors="replace"))
state=json.loads(p.stdout)
assert state["serial"]=="1612425621719"
assert not state["remaining_task_processes"],state
assert state["services"].count("ActiveState=active")==3,state
(DEST/"identity.json").write_text(json.dumps(state,indent=2),encoding="utf-8")
print(json.dumps({"prepared":str(DEST),"state":state},indent=2))

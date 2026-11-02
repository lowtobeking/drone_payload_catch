"""Prepare single forward-translation fusion acceptance; preserve safety guard."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parent.parent
OLD=ROOT/"mocap_right_20261026"
DEST=ROOT/"mocap_forward_20261026"
DEST.mkdir(exist_ok=False)
names=["run_fixed_on_a.py","fixed_runtime_a.py","fixed_guard_a.py","run_fixed_a.sh","yaw_plateau.py","status.py","monitor.py","fetch_and_audit.py","verify_after.py","diagnose.py","stop_session.py","final_audit_a.py"]
block='''        interaction_end=time.monotonic()+96
        from yaw_plateau import wrap_degrees
        observed_motion=False
        stable_since=None
        while True:
            spin(.1);healthy()
            now=time.monotonic()
            recent=[r for r in rows if r["kind"]=="input" and now-4<=r["monotonic"]<=now]
            left=interaction_end-now
            if left<1:raise RuntimeError("manual forward-move window expired")
            if not recent:
                continue
            import statistics
            point=[statistics.mean(r["data"]["position"][i] for r in recent) for i in range(3)]
            delta=[point[i]-baseline["input_xyz"][i] for i in range(3)]
            heading=math.radians(baseline["input_yaw_deg"])
            forward=delta[0]*math.cos(heading)+delta[1]*math.sin(heading)
            cross=-delta[0]*math.sin(heading)+delta[1]*math.cos(heading)
            if forward>.25:
                observed_motion=True
            full=len(recent)>=60 and recent[-1]["monotonic"]-recent[0]["monotonic"]>=3.8
            spans=[max(r["data"]["position"][i] for r in recent)-min(r["data"]["position"][i] for r in recent) for i in range(3)]
            angles=[]
            for r in recent:
                w,x,y,z=r["data"]["q"]
                angles.append(wrap_degrees(math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))-baseline["input_yaw_deg"]))
            valid_direction=(.25<forward<1.5 and abs(cross)<.15 and abs(delta[2])<.10)
            flat=all(math.degrees(math.acos(max(-1.,min(1.,1-2*(r["data"]["q"][1]**2+r["data"]["q"][2]**2)))))<15 for r in recent)
            stable=full and max(spans)<.02 and max(angles)-min(angles)<3 and max(abs(a) for a in angles)<10
            if observed_motion and valid_direction and stable and flat:
                if stable_since is None:
                    stable_since=now
            else:
                stable_since=None
            stable_seconds=now-stable_since if stable_since is not None else 0.
            save("live_status.json",dict(stage="await_forward_move",remaining_seconds=left,
                forward_m=forward,cross_m=cross,stable_seconds=stable_seconds,required_stable_seconds=8))
            if stable_seconds>=8:
                moved=snapshot("forward_move")
                result["phases"]["forward_move"]=moved
                inp=[moved["input_xyz"][i]-baseline["input_xyz"][i] for i in range(3)]
                px4=[moved["px4_xyz"][i]-baseline["px4_xyz"][i] for i in range(3)]
                displacement_error=math.hypot(px4[0]-inp[0],px4[1]-inp[1])
                endpoint_error=math.hypot(moved["px4_xyz"][0]-moved["input_xyz"][0],moved["px4_xyz"][1]-moved["input_xyz"][1])
                yaw_error=abs(wrap_degrees(moved["px4_yaw_deg"]-moved["input_yaw_deg"]))
                result["translation"]=dict(input_displacement_m=inp,px4_displacement_m=px4,
                    input_forward_m=inp[0]*math.cos(heading)+inp[1]*math.sin(heading),
                    px4_forward_m=px4[0]*math.cos(heading)+px4[1]*math.sin(heading),
                    input_cross_m=-inp[0]*math.sin(heading)+inp[1]*math.cos(heading),
                    horizontal_displacement_difference_m=displacement_error,
                    horizontal_endpoint_difference_m=endpoint_error,yaw_difference_deg=yaw_error,
                    ground_functional_tolerance_m=.05,settling_time_after_stable_detection_s=stable_seconds,
                    independent_physical_distance_not_measured=True)
                result["passed"]=(displacement_error<.05 and endpoint_error<.05 and yaw_error<5)
                save("movement_result.json",result)
                save("live_status.json",dict(stage="forward_move_captured",sample=moved,translation=result["translation"]))
                break
'''
for name in names:
    content=(OLD/name).read_text(encoding="utf-8")
    if name=="run_fixed_on_a.py":
        content=content.replace("/right_yaw_","/forward_translation_").replace("payload-catch-right-yaw-","payload-catch-forward-")
        content=content.replace("right-turn acceptance","forward-translation acceptance")
    if name=="fixed_runtime_a.py":
        start=content.index("        interaction_end=time.monotonic()+96")
        end=content.index("    except BaseException as exc:",start)
        content=content[:start]+block+content[end:]
        content=content.replace("after baseline prompt, rotate level right 45-90 degrees; hold 5 seconds; one phase only","after baseline prompt, move level forward 0.4-0.6 m without turning; hold still 15 seconds")
        content=content.replace('stage="await_right_turn"','stage="await_forward_move"')
    if name in ("status.py","fetch_and_audit.py"):
        content=content.replace("right_turn.json","forward_move.json")
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
    raise RuntimeError(p.stderr.decode(errors="replace"))
state=json.loads(p.stdout)
assert state["serial"]=="1612425621719" and not state["remaining_task_processes"],state
assert state["services"].count("ActiveState=active")==3,state
(DEST/"identity.json").write_text(json.dumps(state,indent=2),encoding="utf-8")
print(json.dumps({"prepared":str(DEST),"boot":state["boot"]}))

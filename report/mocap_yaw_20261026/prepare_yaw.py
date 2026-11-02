"""Prepare one automatic left-turn plateau test; retain all timing and FC gates."""
import ast
from pathlib import Path
WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[1]
OLD=ROOT/"report/mocap_auto_lift_20261026"
for name in ["fixed_guard_a.py","run_fixed_a.sh","verify_after.py","stop_session.py","monitor.py","identity.json"]:
    data=(OLD/name).read_bytes()
    if name.endswith(".py"):ast.parse(data)
    (WORK/name).write_bytes(data)
text=(OLD/"fixed_runtime_a.py").read_text(encoding="utf-8")
start=text.index("        from lift_plateau import stable_phase")
end=text.index("    except BaseException as exc:",start)
replacement='''        from yaw_plateau import stable_left,wrap_degrees
        while True:
            spin(.1);healthy()
            now=time.monotonic()
            recent=[r for r in rows if r["kind"]=="input" and now-4<=r["monotonic"]<=now]
            def yaw_degrees(row):
                w,x,y,z=row["data"]["q"]
                return math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
            values=[yaw_degrees(r) for r in recent]
            left=interaction_end-now
            if left<1:raise RuntimeError("manual left-turn window expired")
            save("live_status.json",dict(stage="await_left_turn",remaining_seconds=left,
                input_yaw_deg=values[-1] if values else None))
            flat=all(math.degrees(math.acos(max(-1.,min(1.,1-2*(r["data"]["q"][1]**2+r["data"]["q"][2]**2)))))<15 for r in recent)
            full=(len(recent)>=60 and recent[-1]["monotonic"]-recent[0]["monotonic"]>=3.8)
            if full and flat and stable_left(values,baseline["input_yaw_deg"]):
                turned=snapshot("left_turn")
                result["phases"]["left_turn"]=turned
                input_delta=wrap_degrees(turned["input_yaw_deg"]-baseline["input_yaw_deg"])
                px4_delta=wrap_degrees(turned["px4_yaw_deg"]-baseline["px4_yaw_deg"])
                error=abs(wrap_degrees(px4_delta-input_delta))
                result["yaw"]=dict(input_change_deg=input_delta,px4_change_deg=px4_delta,
                    displacement_difference_deg=error,operator_physical_angle_not_measured=True)
                result["passed"]=(-120<input_delta<-30 and -120<px4_delta<-30 and error<5)
                save("movement_result.json",result)
                save("live_status.json",dict(stage="left_turn_captured",sample=turned,yaw=result["yaw"]))
                break
'''
text=text[:start]+replacement+text[end:]
text=text.replace('instruction="lift slowly; hold steady 5 seconds; return after elevated captured; phases detected from fresh input"',
                  'instruction="after baseline prompt, rotate level left 45-90 degrees; hold 5 seconds; one phase only"')
# Circular mean prevents wraparound at +/-180 within a stationary plateau.
text=text.replace('statistics.mean(yaw(d["q"]) for d in inp)',
                  'math.atan2(statistics.mean(math.sin(yaw(d["q"])) for d in inp),statistics.mean(math.cos(yaw(d["q"])) for d in inp))')
text=text.replace('statistics.mean(d["heading"] for d in loc)',
                  'math.atan2(statistics.mean(math.sin(d["heading"]) for d in loc),statistics.mean(math.cos(d["heading"]) for d in loc))')
ast.parse(text)
(WORK/"fixed_runtime_a.py").write_text(text,encoding="utf-8")
runner=(OLD/"run_fixed_on_a.py").read_text(encoding="utf-8").replace('sources["lift_plateau.py"] = (WORK/"lift_plateau.py").read_bytes()','sources["yaw_plateau.py"] = (WORK/"yaw_plateau.py").read_bytes()')
runner=runner.replace('/auto_lift_', '/auto_yaw_').replace('payload-catch-ground-lift-', 'payload-catch-ground-yaw-').replace('Bounded disarmed mocap lift acceptance','Bounded disarmed mocap yaw acceptance')
ast.parse(runner)
(WORK/"run_fixed_on_a.py").write_text(runner,encoding="utf-8")
for name in ["status.py","fetch_and_audit.py"]:
    text=(OLD/name).read_text(encoding="utf-8").replace("'elevated.json'","'left_turn.json'").replace('"elevated.json"','"left_turn.json"')
    ast.parse(text)
    (WORK/name).write_text(text,encoding="utf-8")
print("Prepared single-action left-turn test; bounded timing and all safety gates retained.")

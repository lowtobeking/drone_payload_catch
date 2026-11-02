"""Prepare bounded automatic lift capture, detached from SSH lifecycle."""
import ast
import json
from pathlib import Path
WORK=Path(__file__).resolve().parent
PROJECT=WORK.parents[1]
OLD=PROJECT/"report/mocap_continue_20261026"
for name in ["fixed_guard_a.py","run_fixed_a.sh","verify_after.py","stop_session.py","monitor.py"]:
    source=(OLD/name).read_text(encoding="utf-8")
    if name.endswith(".py"):ast.parse(source)
    (WORK/name).write_text(source,encoding="utf-8",newline="\n")
runtime=(OLD/"fixed_runtime_a.py").read_text(encoding="utf-8")
start=runtime.index('        for phase in ("elevated","returned"):')
end=runtime.index('        base=result["phases"]["baseline"]',start)
replacement='''        from lift_plateau import stable_phase
        for phase in ("elevated","returned"):
            while True:
                spin(.1);healthy()
                now=time.monotonic()
                recent=[r for r in rows if r["kind"]=="input" and now-4<=r["monotonic"]<=now]
                zvalues=[r["data"]["position"][2] for r in recent]
                left=interaction_end-now
                if left<1: raise RuntimeError("manual interaction window expired")
                save("live_status.json",dict(stage="await_"+phase,remaining_seconds=left,
                    input_z=latest.get("input",({"position":[None,None,None]},0))[0]["position"][2],
                    px4_z=latest.get("local",({},0))[0].get("z")))
                full_window=(len(recent)>=60 and recent[-1]["monotonic"]-recent[0]["monotonic"]>=3.8)
                if full_window and stable_phase(zvalues,baseline["input_xyz"][2],phase):
                    sample=snapshot(phase)
                    result["phases"][phase]=sample
                    save("live_status.json",dict(stage=phase+"_captured",sample=sample,
                        remaining_seconds=interaction_end-time.monotonic()))
                    break
'''
runtime=runtime[:start]+replacement+runtime[end:]
runtime=runtime.replace('instruction="lift slowly then confirm; return only after elevated captured"',
                        'instruction="lift slowly; hold steady 5 seconds; return after elevated captured; phases detected from fresh input"')
ast.parse(runtime)
(WORK/"fixed_runtime_a.py").write_text(runtime,encoding="utf-8")
runner=(OLD/"run_fixed_on_a.py").read_text(encoding="utf-8")
runner=runner.replace('    hashes = {k: hashlib.sha256(v).hexdigest() for k,v in sources.items()}',
    '    sources["lift_plateau.py"] = (WORK/"lift_plateau.py").read_bytes()\n    hashes = {k: hashlib.sha256(v).hexdigest() for k,v in sources.items()}')
runner=runner.replace('"/home/nvidia/payload_catch_mocap_fusion_20261026/lift_reference_"','"/home/nvidia/payload_catch_mocap_fusion_20261026/auto_lift_"')
start=runner.index('    try:\n        proc = run(')
end=runner.index('\n\nif __name__',start)
runner=runner[:start]+'''    unit="payload-catch-ground-lift-"+runid.lower()
    launch=run(["sudo","-n","systemd-run","--unit="+unit,
        "--description=Bounded disarmed mocap lift acceptance",
        "--property=User=nvidia","--property=WorkingDirectory="+remote,
        "--property=RuntimeMaxSec=950","--property=TimeoutStopSec=660",
        "--property=KillMode=process",
        "/usr/bin/timeout","--signal=TERM","--kill-after=660s","310s",
        "/bin/bash",remote+"/run_fixed_a.sh",remote,logs],"launch_detached",timeout=15)
    m=json.loads((local/"manifest.json").read_text())
    m.update(unit=unit,detached=True)
    (local/"manifest.json").write_text(json.dumps(m,indent=2),encoding="utf-8")
    print(launch.stdout.decode("utf-8",errors="replace"),flush=True)
    print("Started bounded independent A-machine unit",unit,"EVIDENCE",local,flush=True)
'''+runner[end:]
ast.parse(runner)
(WORK/"run_fixed_on_a.py").write_text(runner,encoding="utf-8")
# Identity baseline for independent post-run verification.
(WORK/"identity.json").write_bytes((OLD/"identity.json").read_bytes())
print("Prepared detached bounded auto-plateau lift test; all safety limits retained.")

"""Read-only post-test full parameter and process/service verification."""
import ast
from datetime import datetime,timezone
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
sys.dont_write_bytecode=True
WORK=Path(__file__).resolve().parent
OLD=WORK.parent/"mocap_fusion_20261023"
spec=importlib.util.spec_from_file_location("fixed_runner",WORK/"run_fixed_on_a.py")
runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
def main():
    sys.stdout.reconfigure(encoding="utf-8");sys.stderr.reconfigure(encoding="utf-8")
    test=sorted(WORK.glob("lift_reference_20*"))[-1]
    local=WORK/("after_"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"));local.mkdir()
    audit=(OLD/"final_audit_a.py").read_bytes();ast.parse(audit)
    p=subprocess.run(runner.SSH+["/usr/bin/python3 -"],input=audit,cwd=WORK,timeout=25,capture_output=True,shell=False)
    (local/"audit.stdout.log").write_bytes(p.stdout);(local/"audit.stderr.log").write_bytes(p.stderr)
    if p.returncode: raise RuntimeError(p.stderr.decode(errors="replace"))
    state=json.loads(p.stdout)
    (local/"audit.json").write_bytes(p.stdout)
    spec=importlib.util.spec_from_file_location("post_backup",OLD/"run_parameter_backup.py")
    b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
    b.SSH=runner.SSH;b.WORK=local
    (local/"backup_parameters_a.py").write_bytes((OLD/"backup_parameters_a.py").read_bytes())
    b.main()
    afterpath=sorted(local.glob("parameters_*/parameters_backup.json"))[-1]
    before=json.loads((test/"parameters_before.json").read_text(encoding="utf-8"))
    after=json.loads(afterpath.read_text(encoding="utf-8"))
    diffs={k:dict(before=before["parameters"].get(k),after=after["parameters"].get(k))
           for k in set(before["parameters"])|set(after["parameters"])
           if before["parameters"].get(k)!=after["parameters"].get(k)}
    first=json.loads((WORK/"identity.json").read_text(encoding="utf-8"))
    print("Identity baseline keys:",list(first))
    result=dict(test=str(test),post=str(local),parameter_differences=diffs,parameter_count=len(after["parameters"]),
                complete=after["complete"],jetson_boot=state["boot"],remaining_task_processes=state["remaining_task_processes"],
                services=state["services"],current=state["current"])
    result["exact_parameter_match"]=not diffs
    result["test_parameters_restored"]=all(after["parameters"][k]["value"]==v for k,v in
        (("EKF2_MAG_TYPE",0),("EKF2_EV_CTRL",0),("EKF2_HGT_REF",1),("UXRCE_DDS_SYNCT",1)))
    result["jetson_boot_unchanged"]=state["boot"]==first["boot"]
    result["autonomous_calibration_candidates"]={k:v for k,v in diffs.items() if k=="CAL_BARO0_OFF"}
    result["unexpected_parameter_changes"]={k:v for k,v in diffs.items() if k!="CAL_BARO0_OFF"}
    result["passed"]=(result["test_parameters_restored"] and result["jetson_boot_unchanged"]
        and not result["unexpected_parameter_changes"] and after["complete"] and not state["remaining_task_processes"])
    (local/"comparison.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result["passed"] else 2
if __name__=="__main__": raise SystemExit(main())

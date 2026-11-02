"""Fetch complete evidence of the detached auto-lift run, then independently audit."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
WORK=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("runner",WORK/"run_fixed_on_a.py")
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
test=sorted(WORK.glob("lift_reference_20*"))[-1]
manifest=json.loads((test/"manifest.json").read_text())
files=["validation.json","process_cleanup.json","samples.jsonl","bridge.log","source.log","source/reader_process.json",
       "source/reader_cleanup.json","source/native.stderr.log","task_processes.json","parameters_before.json",
       "parameter_lease_result.json","guard.log","configure_reboot_requested.json","configure_reboot_verified.json",
       "restore_reboot_requested.json","restore_reboot_verified.json","backup_ready.json","enabled.json",
       "manual_ready.json","baseline.json","left_turn.json","returned.json","movement_result.json","live_status.json"]
missing=[]
for name in files:
    p=subprocess.run(r.SSH+["/bin/cat "+manifest["remote_logs"]+"/"+name],cwd=WORK,timeout=35,capture_output=True,shell=False)
    (test/("fetch_"+name.replace("/","_")+".stderr.log")).write_bytes(p.stderr)
    if p.returncode:
        missing.append(name)
        continue
    target=test/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(p.stdout)
(test/"fetch_summary.json").write_text(json.dumps({"missing":missing},indent=2)+"\n",encoding="utf-8")
if "validation.json" in missing or "parameters_before.json" in missing:
    raise RuntimeError("Essential evidence missing")
p=subprocess.run([sys.executable,"-B",str(WORK/"verify_after.py")],cwd=WORK,timeout=100,capture_output=True,shell=False)
(WORK/"independent_audit.stdout.log").write_bytes(p.stdout)
(WORK/"independent_audit.stderr.log").write_bytes(p.stderr)
sys.stdout.reconfigure(encoding="utf-8")
print("Missing:",missing,flush=True)
print(p.stdout.decode("utf-8",errors="replace"));print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
sys.exit(p.returncode)

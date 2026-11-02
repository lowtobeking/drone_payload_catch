"""Recover exact interrupted run evidence and invoke independent parameter audit."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
WORK=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("runner",WORK/"run_fixed_on_a.py")
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
test=WORK/"lift_reference_20261026T030147Z"
manifest=json.loads((test/"manifest.json").read_text())
files=["validation.json","process_cleanup.json","samples.jsonl","source/reader_cleanup.json",
"parameter_lease_result.json","restore_reboot_requested.json","restore_reboot_verified.json","guard.log"]
for name in files:
    p=subprocess.run(r.SSH+["/bin/cat "+manifest["remote_logs"]+"/"+name],cwd=WORK,timeout=35,capture_output=True,shell=False)
    (test/("recovery_"+name.replace("/","_")+".stderr.log")).write_bytes(p.stderr)
    if p.returncode:raise RuntimeError("fetch failed "+name)
    target=test/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(p.stdout)
print("Recovered all interrupted-run evidence.",flush=True)
p=subprocess.run([sys.executable,"-B",str(WORK/"verify_after.py")],cwd=WORK,timeout=100,capture_output=True,shell=False)
(WORK/"manual_retry_audit.stdout.log").write_bytes(p.stdout)
(WORK/"manual_retry_audit.stderr.log").write_bytes(p.stderr)
print(p.stdout.decode("utf-8",errors="replace"));print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
sys.exit(p.returncode)

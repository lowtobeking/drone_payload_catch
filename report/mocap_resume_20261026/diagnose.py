"""Read exact test runtime stdout and newest bridge states without changing anything."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
WORK=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("r",WORK/"run_fixed_on_a.py")
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
test=sorted(WORK.glob("lift_reference_20*"))[-1]
m=json.loads((test/"manifest.json").read_text())
code="from pathlib import Path\nimport json,subprocess\nroot=Path("+repr(m["remote_logs"])+")\nout={}\n"
code+="for name in ['validation.json','process_cleanup.json','parameter_lease_result.json']:\n p=root/name\n if p.exists():out[name]=json.loads(p.read_text())\n"
code+="p=root/'bridge.log'\nout['bridge_tail']=p.read_text().splitlines()[-2:] if p.exists() else []\n"
code+="p=subprocess.run(['journalctl','-u',"+repr(m["unit"])+",'--no-pager','-n','15'],capture_output=True,text=True,timeout=10)\nout['runtime_journal']=p.stdout\nprint(json.dumps(out,indent=2))\n"
ast.parse(code)
p=subprocess.run(r.SSH+["/usr/bin/python3 -"],input=code.encode(),cwd=WORK,timeout=20,capture_output=True,shell=False)
(test/"failure_diagnosis.stdout.log").write_bytes(p.stdout);(test/"failure_diagnosis.stderr.log").write_bytes(p.stderr)
sys.stdout.reconfigure(encoding="utf-8")
print(p.stdout.decode("utf-8",errors="replace"));print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
sys.exit(p.returncode)

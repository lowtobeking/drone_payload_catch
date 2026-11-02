"""Read bounded ground run status; wait up to 45 seconds for baseline if requested."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
WORK=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("runner",WORK/"run_fixed_on_a.py")
runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
test=sorted(WORK.glob("lift_reference_20*"))[-1]
manifest=json.loads((test/"manifest.json").read_text())
wait=45 if len(sys.argv)>1 and sys.argv[1]=="wait" else 0
code="from pathlib import Path\nimport json,time\nroot=Path("+repr(manifest["remote_logs"])+")\nend=time.monotonic()+"+str(wait)+"\n"
code+="while not (root/'manual_ready.json').exists() and not (root/'validation.json').exists() and time.monotonic()<end:\n time.sleep(.25)\n"
code+="out={}\nfor name in ['live_status.json','baseline.json','left_turn.json','returned.json','movement_result.json','validation.json','parameter_lease_result.json']:\n p=root/name\n if p.exists():out[name]=json.loads(p.read_text())\n"
code+="p=root/'guard.log'\nout['guard_tail']=p.read_text().splitlines()[-1:] if p.exists() else []\nprint(json.dumps(out,indent=2))\n"
ast.parse(code)
p=subprocess.run(runner.SSH+["/usr/bin/python3 -"],input=code.encode(),cwd=WORK,timeout=wait+15,capture_output=True,shell=False)
(test/"latest_status.stdout.log").write_bytes(p.stdout)
(test/"latest_status.stderr.log").write_bytes(p.stderr)
sys.stdout.reconfigure(encoding="utf-8")
print(p.stdout.decode("utf-8",errors="replace"),flush=True)
print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr,flush=True)
sys.exit(p.returncode)

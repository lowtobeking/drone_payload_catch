"""Read only current guard tail and cleanup status for this exact run."""
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
code="from pathlib import Path\nimport json\nroot=Path("+repr(manifest["remote_logs"])+")\nout={}\n"
code+="for name in ['parameter_lease_result.json','process_cleanup.json','restore_reboot_requested.json','restore_reboot_verified.json']:\n p=root/name\n if p.exists():out[name]=json.loads(p.read_text())\n"
code+="p=root/'guard.log'\nout['guard_tail']=p.read_text().splitlines()[-2:] if p.exists() else []\nprint(json.dumps(out,indent=2))\n"
ast.parse(code)
p=subprocess.run(r.SSH+["/usr/bin/python3 -"],input=code.encode(),cwd=WORK,timeout=15,capture_output=True,shell=False)
(WORK/"monitor.stdout.log").write_bytes(p.stdout);(WORK/"monitor.stderr.log").write_bytes(p.stderr)
sys.stdout.reconfigure(encoding="utf-8")
print(p.stdout.decode("utf-8",errors="replace"));print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
sys.exit(p.returncode)

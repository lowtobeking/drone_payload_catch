"""Read-only compact progress for latest staged test."""
import importlib.util
import json
from pathlib import Path
import subprocess
WORK=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("fixed",WORK/"run_fixed_on_a.py")
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
test=sorted(WORK.glob("fixed_reference_20*"))[-1]
m=json.loads((test/"manifest.json").read_text(encoding="utf-8"))
code="from pathlib import Path\nimport json\np=Path("+repr(m["remote_logs"])+")\n"
code+="for name in ['configure_reboot_requested.json','configure_reboot_verified.json','enabled.json','restore_reboot_verified.json','parameter_lease_result.json','validation.json']:\n f=p/name\n print(name, f.read_text()[:1800] if f.exists() else 'pending')\n"
code+="for name in ['guard.log','bridge.log']:\n f=p/name\n print(name, '\\n'.join(f.read_text().splitlines()[-2:]) if f.exists() else 'pending')\n"
import ast
ast.parse(code)
(test/"progress_readonly_a.py").write_text(code,encoding="utf-8")
p=subprocess.run(r.SSH+["/usr/bin/python3 -"],input=code.encode(),cwd=WORK,timeout=15,capture_output=True,shell=False)
(test/"progress.stdout.log").write_bytes(p.stdout);(test/"progress.stderr.log").write_bytes(p.stderr)
print(p.stdout.decode(errors="replace"));print(p.stderr.decode(errors="replace"))
raise SystemExit(p.returncode)

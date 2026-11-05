"""Read A deployment build state and logs."""
import ast
import json
from pathlib import Path
import subprocess
from probe_a import SSH
WORK=Path(__file__).resolve().parent
m=json.loads((WORK/"manifest.json").read_text(encoding="utf-8"))
code="from pathlib import Path\nimport json,subprocess\nroot=Path("+repr(m["remote"])+")\np=subprocess.run(['systemctl','show',"+repr(m["build_unit"])+",'-p','ActiveState','-p','SubState','-p','ExecMainStatus'],capture_output=True,text=True,timeout=8)\nout=dict(unit=p.stdout,stdout=(root/'build.stdout.log').read_text() if (root/'build.stdout.log').exists() else '',stderr=(root/'build.stderr.log').read_text() if (root/'build.stderr.log').exists() else '',built=Path("+repr(m["release"]+"/BUILT")+").exists())\nprint(json.dumps(out,indent=2))\n"
ast.parse(code)
p=subprocess.run(SSH+["/usr/bin/python3 -"],input=code.encode(),cwd=WORK,capture_output=True,timeout=25,shell=False)
(WORK/"build_status.stdout.log").write_bytes(p.stdout)
(WORK/"build_status.stderr.log").write_bytes(p.stderr)
if p.returncode: raise RuntimeError(p.stderr.decode(errors="replace"))
data=json.loads(p.stdout)
(WORK/"build_status.json").write_text(json.dumps(data,indent=2),encoding="utf-8")
print(json.dumps(data,indent=2))

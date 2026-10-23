"""Read live phase results or mark only the operator-confirmed lift/return plateau."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("lift_runner",ROOT/"run_fixed_on_a.py")
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
def main():
    action=sys.argv[1] if len(sys.argv)>1 else "status"
    if action not in ("status","elevated","returned"):raise ValueError("action not allowlisted")
    test=sorted(ROOT.glob("lift_reference_20*"))[-1]
    m=json.loads((test/"manifest.json").read_text(encoding="utf-8"))
    code="from pathlib import Path\nimport json\n"
    code+="assert Path('/proc/device-tree/serial-number').read_text().strip('\\0\\n')=='1612425621719'\n"
    code+="root=Path("+repr(m["remote_logs"])+")\n"
    if action!="status":
        code+="state=json.loads((root/'live_status.json').read_text())\n"
        code+="assert state['stage']=="+repr("await_"+action)+",state\n"
        code+="(root/"+repr(action+".request")+").touch()\n"
    code+="out={}\nfor name in ['live_status.json','manual_ready.json','baseline.json','elevated.json','returned.json','movement_result.json','validation.json','parameter_lease_result.json']:\n f=root/name\n if f.exists():out[name]=json.loads(f.read_text())\n"
    code+="if not out:\n f=root/'guard.log'\n out['guard_tail']=f.read_text().splitlines()[-1:] if f.exists() else ['not_started']\n"
    code+="print(json.dumps(out,indent=2))\n"
    ast.parse(code)
    (test/("control_"+action+"_remote.py")).write_text(code,encoding="utf-8")
    p=subprocess.run(r.SSH+["/usr/bin/python3 -"],input=code.encode(),cwd=ROOT,timeout=15,capture_output=True,shell=False)
    (test/("control_"+action+".stdout.log")).write_bytes(p.stdout)
    (test/("control_"+action+".stderr.log")).write_bytes(p.stderr)
    print(p.stdout.decode(errors="replace"));print(p.stderr.decode(errors="replace"))
    return p.returncode
if __name__=="__main__":raise SystemExit(main())

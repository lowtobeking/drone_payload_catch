"""Gracefully stop exactly this run's observer; its finally block restores parameters."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("lift",ROOT/"run_fixed_on_a.py")
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
test=sorted(ROOT.glob("lift_reference_20*"))[-1]
m=json.loads((test/"manifest.json").read_text(encoding="utf-8"))
code="""from pathlib import Path
import json,os,signal
assert Path('/proc/device-tree/serial-number').read_text().strip('\\0\\n')=='1612425621719'
"""
code+="script="+repr(m["remote"]+"/fixed_runtime_a.py")+"\nlogs="+repr(m["remote_logs"])+"\n"
code+="""matched=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        args=[x.decode() for x in (p/'cmdline').read_bytes().split(b'\\0') if x]
        if len(args)==3 and (p/'cwd').resolve()==Path(script).parent and args[1] in (script,Path(script).name) and args[2]==logs:
            matched.append(int(p.name))
    except (OSError,UnicodeError):pass
assert len(matched)<=1,matched
for pid in matched:os.kill(pid,signal.SIGTERM)
print(json.dumps(dict(graceful_stop_sent_to=matched,scope='exact task runtime only')))
"""
ast.parse(code)
(test/"stop_remote.py").write_text(code,encoding="utf-8")
p=subprocess.run(r.SSH+["/usr/bin/python3 -"],input=code.encode(),cwd=ROOT,timeout=15,capture_output=True,shell=False)
(test/"stop.stdout.log").write_bytes(p.stdout);(test/"stop.stderr.log").write_bytes(p.stderr)
print(p.stdout.decode(errors="replace"));print(p.stderr.decode(errors="replace"))
raise SystemExit(p.returncode)

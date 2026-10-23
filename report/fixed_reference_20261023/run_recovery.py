"""Stage and run recovery-only script, preserving full output."""
import ast
import importlib.util
import json
from pathlib import Path
import shlex
import subprocess
import sys
WORK=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("fixed_runner",WORK/"run_fixed_on_a.py")
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
def main():
    sys.stdout.reconfigure(encoding="utf-8");sys.stderr.reconfigure(encoding="utf-8")
    test=sorted(WORK.glob("fixed_reference_20*"))[-1]
    manifest=json.loads((test/"manifest.json").read_text(encoding="utf-8"))
    remote=manifest["remote"];logs=manifest["remote_logs"]+"/recovery"
    source=(WORK/"restore_original_a.py").read_bytes();ast.parse(source)
    code="from pathlib import Path\nimport ast\np=Path("+repr(remote+"/restore_original_a.py")+")\ndata="+repr(source)+"\nast.parse(data)\np.write_bytes(data)\n"
    ast.parse(code)
    (test/"stage_recovery.py").write_text(code,encoding="utf-8")
    for name,args,data,timeout in [
        ("stage_recovery",["/usr/bin/python3","-"],code.encode(),15),
        ("recovery",["sudo","-n","/usr/bin/env","PYTHONPATH=/home/nvidia/.local/lib/python3.10/site-packages","/usr/bin/timeout","110s","/usr/bin/python3",remote+"/restore_original_a.py",logs],None,125),
        ("fetch_recovery",["/bin/cat",logs+"/recovery_result.json"],None,15)]:
        p=subprocess.run(m.SSH+[shlex.join(args)],input=data,cwd=WORK,timeout=timeout,capture_output=True,shell=False)
        (test/(name+".stdout.log")).write_bytes(p.stdout);(test/(name+".stderr.log")).write_bytes(p.stderr)
        print(name,p.returncode,p.stdout.decode(errors="replace"),p.stderr.decode(errors="replace"),flush=True)
        if name=="stage_recovery" and p.returncode: return p.returncode
        if name=="fetch_recovery" and p.returncode==0:(test/"recovery_result.json").write_bytes(p.stdout)
    return 0
if __name__=="__main__":raise SystemExit(main())

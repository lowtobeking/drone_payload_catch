"""Read-only post-test parameter and process verification; no FC writes."""
import ast
from pathlib import Path
import subprocess
import sys
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import run_parameter_backup
run_parameter_backup.SSH=[s.replace("192.168.1.2","192.168.1.123").replace("nvidia@192.168.1.4","nvidia@192.168.1.101")
                          for s in run_parameter_backup.SSH]
run_parameter_backup.main()
script=ROOT/"final_audit_a.py"
ast.parse(script.read_text(encoding="utf-8"))
proc=subprocess.run(run_parameter_backup.SSH+["/usr/bin/python3 -"],input=script.read_bytes(),
                    cwd=ROOT,capture_output=True,timeout=20,shell=False)
(ROOT/"forward_final_audit.stdout.log").write_bytes(proc.stdout)
(ROOT/"forward_final_audit.stderr.log").write_bytes(proc.stderr)
if proc.returncode==0:
    (ROOT/"forward_final_audit.json").write_bytes(proc.stdout)
print(proc.stdout.decode("utf-8",errors="replace"),flush=True)
if proc.returncode:
    print(proc.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
raise SystemExit(proc.returncode)

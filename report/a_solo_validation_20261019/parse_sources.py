"""Parse task sources without executing them or producing bytecode."""
import ast
from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[2]
names=("payload_catch/solo_validation.py","payload_catch/solo_validation_node.py",
    "tools/a_solo_preflight.py","tools/a_solo_control.py","tools/test_a_solo.py",
    "tools/check_a_solo_ros.py","tools/run_a_solo_isolated.py","launch/a_solo_validation_launch.py",
    "tools/a_solo_runtime.py", "setup.py")
for name in names:
    ast.parse((root/name).read_text(encoding="utf-8"), filename=name)
for name in ("deploy/run_a_solo.sh","tools/run_a_solo_isolated.sh",
             "report/a_solo_validation_20261019/run_checks.sh"):
    subprocess.run(["/bin/bash","-n",str(root/name)],cwd=root,timeout=5,check=True)
print("AST/Bash parse checks OK")

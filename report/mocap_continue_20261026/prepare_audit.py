"""Prepare post-test independent audit from today's identity baseline."""
import ast
import json
from pathlib import Path
WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[1]
pre=json.loads((WORK/"precheck.stdout.log").read_text(encoding="utf-8"))
(WORK/"identity.json").write_text(json.dumps({"serial":pre["serial"],"boot":pre["jetson_boot"]},indent=2)+"\n",encoding="utf-8")
code=(ROOT/"report/mocap_lift_20261023/verify_after.py").read_text(encoding="utf-8")
ast.parse(code)
(WORK/"verify_after.py").write_text(code,encoding="utf-8")
print("Post-run audit prepared; not executed yet.")

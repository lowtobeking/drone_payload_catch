"""Remove only obsolete bootstrap files and bytecode owned by this task."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
report=ROOT/"report/a_solo_validation_20261019"
names=("report/a_solo_validation_20261019/write_implementation.py",
       "report/a_solo_validation_20261019/fix_packaging.py",
       "payload_catch/__pycache__/solo_validation.cpython-310.pyc",
       "tools/__pycache__/a_solo_preflight.cpython-310.pyc")
removed=[]
for name in names:
    target=(ROOT/name).resolve()
    if not target.is_relative_to(ROOT.resolve()) or target.is_symlink():
        raise RuntimeError("unexpected cleanup target: "+str(target))
    if target.exists():
        if not target.is_file():
            raise RuntimeError("not a regular owned file: "+str(target))
        target.unlink()
        removed.append(str(target))
(report/"cleanup.json").write_text(json.dumps(dict(removed=removed,
    kept="source, tests, reproducible checks, backup and verification logs"),indent=2)+"\n",encoding="utf-8")
print(json.dumps(removed,indent=2))

"""Point the continuous-session monitor at its own report directory."""
import ast
from pathlib import Path
path=Path(__file__).resolve().parent/"monitor_continuous.py"
content=path.read_text(encoding="utf-8")
old='WORK=Path(__file__).resolve().parent\nrun=sorted(WORK.glob("lift_reference_20*"))[-1]'
new='WORK=Path(__file__).resolve().parent.parent/"mocap_continuous_20261026"\nrun=sorted(WORK.glob("lift_reference_20*"))[-1]'
assert content.count(old)==1
content=content.replace(old,new,1)
ast.parse(content)
path.write_text(content,encoding="utf-8",newline="\n")
print("Monitor target corrected")

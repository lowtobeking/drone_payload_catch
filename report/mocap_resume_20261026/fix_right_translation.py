"""Correct accidental stream counter rename; verify right projection and stream schema."""
import ast
import json
import math
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent/"mocap_right_translation_20261026"
path=ROOT/"fixed_runtime_a.py"
content=path.read_text(encoding="utf-8")
assert content.count('"righted"')==2
archive=ROOT/"fixed_runtime_a.before_counter_fix.py"
assert not archive.exists()
archive.write_text(content,encoding="utf-8",newline="\n")
content=content.replace('"righted"','"forwarded"')
tree=ast.parse(content)
assert '"righted"' not in content and content.count('get("forwarded",0)')==2
assignments={}
for node in ast.walk(tree):
    if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name) and node.targets[0].id in ("right","cross"):
        assignments[node.targets[0].id]=node.value
assert set(assignments)=={"right","cross"}
for angle,delta,expected in [(0,[0,.5,0],(.5,0)),(90,[-.5,0,0],(.5,0)),(-90,[.5,0,0],(.5,0)),(180,[0,-.5,0],(.5,0)),(0,[.5,0,0],(0,.5))]:
    actual=[]
    for name in ("right","cross"):
        expression=ast.Expression(assignments[name])
        actual.append(eval(compile(expression,"right_projection","eval"),{"__builtins__":{}},{"delta":delta,"heading":math.radians(angle),"math":math}))
    assert all(abs(a-e)<1e-9 for a,e in zip(actual,expected)),(angle,actual,expected)
path.write_text(content,encoding="utf-8",newline="\n")
result={"stream_count_field":"forwarded","projection_checks":5,"scope":"test runtime only; production bridge/guards unchanged"}
(ROOT/"counter_fix_verification.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
print(json.dumps(result))

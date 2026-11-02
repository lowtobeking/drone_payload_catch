"""Remove the unused finite-motion helper from continuous launcher."""
import ast
from pathlib import Path
path=Path(__file__).resolve().parent/"../mocap_continuous_20261026/run_fixed_on_a.py"
path=path.resolve()
content=path.read_text(encoding="utf-8")
line='    sources["yaw_plateau.py"] = (WORK/"yaw_plateau.py").read_bytes()\n'
assert content.count(line)==1
content=content.replace(line,"",1)
ast.parse(content)
path.write_text(content,encoding="utf-8",newline="\n")
print("Removed unused yaw plateau dependency")

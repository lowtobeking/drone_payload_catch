"""Add the missing parent directory used only by offline tests."""
import ast
from pathlib import Path
path=Path(__file__).resolve().parent/"validate_and_activate.py"
text=path.read_text(encoding="utf-8")
old="for name in ['fixed_reference_20261023','mocap_z_20261023']:"
new="for name in ['fixed_reference_20261023','mocap_z_20261023','mocap_fusion_20261023']:"
assert text.count(old)==1
text=text.replace(old,new)
ast.parse(text)
path.write_text(text,encoding="utf-8",newline="\n")
print("Offline-test directory preparation corrected")

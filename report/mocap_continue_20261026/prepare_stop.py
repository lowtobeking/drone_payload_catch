from pathlib import Path
import ast
root=Path(__file__).resolve().parents[2]
target=Path(__file__).with_name("stop_session.py")
text=(root/"report/mocap_lift_20261023/stop_session.py").read_text(encoding="utf-8")
ast.parse(text)
target.write_text(text,encoding="utf-8")

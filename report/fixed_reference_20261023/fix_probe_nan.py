from pathlib import Path
import ast
p=Path(__file__).resolve().parent/"fc_time_probe_a.py"
s=p.read_text(encoding="utf-8")
s=s.replace('result["messages"].setdefault(m.get_type(),m.to_dict())',
'''result["messages"].setdefault(m.get_type(),{"type":m.get_type(),
                    "time_boot_ms":getattr(m,"time_boot_ms",None)})''',1)
ast.parse(s)
p.write_text(s,encoding="utf-8")

from pathlib import Path
import ast
p=Path(__file__).resolve().parent/"run_fc_time_probe.py"
s=p.read_text(encoding="utf-8")
s=s.replace('            "(p/\'backup_parameters_a.py\').write_bytes(data)\\n")',
'''            "(p/'fc_time_probe_a.py').write_bytes(data)\\n"
            "(p/'backup_parameters_a.py').write_bytes("+repr(backup)+")\\n")''',1)
assert '"(p/\'fc_time_probe_a.py\').write_bytes(data)' in s
ast.parse(s)
p.write_text(s,encoding="utf-8")

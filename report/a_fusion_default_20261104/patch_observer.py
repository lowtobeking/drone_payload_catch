from pathlib import Path
p=Path("report/a_fusion_default_20261104/check_odom_time.py")
s=p.read_text(encoding="utf-8")
old='"source_protocol":b[1].source_protocol} if b else None)}))'
new='"source_protocol":b[1].source_protocol} if b else None)}, default=float))'
assert s.count(old)==1
p.write_text(s.replace(old,new),encoding="utf-8")

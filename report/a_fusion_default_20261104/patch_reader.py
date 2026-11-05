from pathlib import Path
p=Path("tools/a_solo_parameters_passive.py")
s=p.read_text(encoding="utf-8")
old="from pymavlink import mavutil"
new='sys.path.insert(0, "/home/nvidia/.local/lib/python3.10/site-packages")\nfrom pymavlink import mavutil'
assert s.count(old)==1
p.write_text(s.replace(old,new),encoding="utf-8")

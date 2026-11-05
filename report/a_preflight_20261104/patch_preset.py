from pathlib import Path
p=Path("payload_catch/fcu_params.py")
s=p.read_text(encoding="utf-8")
old="('EKF2_BARO_CTRL', 0, '视觉高度源，关气压计（两套高度源别混）'),"
new="('EKF2_BARO_CTRL', 1, '保留气压计辅助；视觉高度为 EKF2_HGT_REF=3'),\n        ('EKF2_HGT_REF', 3, '视觉高度参考'),"
assert s.count(old)==1
p.write_text(s.replace(old,new),encoding="utf-8")

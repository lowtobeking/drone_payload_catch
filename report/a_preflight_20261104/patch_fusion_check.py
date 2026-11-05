from pathlib import Path
p=Path("payload_catch/fcu_params.py")
s=p.read_text(encoding="utf-8")
old="int(p.get('EKF2_BARO_CTRL', -1)) == 0"
assert s.count(old)==1
s=s.replace(old,"int(p.get('EKF2_BARO_CTRL', -1)) == 1",1)
old="and int(p.get('EKF2_MAG_TYPE', -1)) == 5)"
assert s.count(old)==1
s=s.replace(old,"and int(p.get('EKF2_HGT_REF', -1)) == 3\n                  and int(p.get('EKF2_MAG_TYPE', -1)) == 5)",1)
s=s.replace("EKF2 源=动捕(0/11/0/5)","EKF2 源=动捕(0/11/1/3/5)")
p.write_text(s,encoding="utf-8")
p=Path("tools/test_fcu_params.py")
s=p.read_text(encoding="utf-8")
old="'EKF2_BARO_CTRL': 0, 'EKF2_MAG_TYPE': 5"
assert s.count(old)==1
s=s.replace(old,"'EKF2_BARO_CTRL': 1, 'EKF2_HGT_REF': 3, 'EKF2_MAG_TYPE': 5",1)
s=s.replace("EKF2 源=动捕(0/11/0/5)","EKF2 源=动捕(0/11/1/3/5)")
p.write_text(s,encoding="utf-8")

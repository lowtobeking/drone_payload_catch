"""Restore A barometer aiding; retain EV height reference and GPS disabled."""
import json
from pathlib import Path
import sys
from mocap_ground_guard_a import Tap, wait_ground
from configure_px4_mocap import set_int

LOG=Path(sys.argv[1])
tap=Tap()
try:
    wait_ground(tap)
    tap.verify_identity()
    expected={"EKF2_GPS_CTRL":0,"EKF2_BARO_CTRL":0,"EKF2_EV_CTRL":11,
              "EKF2_HGT_REF":3,"EKF2_MAG_TYPE":5,"UXRCE_DDS_SYNCT":1}
    for key,value in expected.items():
        if tap.read(key)!=(value,6):
            raise RuntimeError("unexpected PX4 setting: "+key)
    set_int(tap,"EKF2_BARO_CTRL",1)
    tap.reboot_fc("restore_baro")
    expected["EKF2_BARO_CTRL"]=1
    for key,value in expected.items():
        if tap.read(key)!=(value,6):
            raise RuntimeError("post-reboot setting mismatch: "+key)
    print(json.dumps({"passed":True,"parameters":expected,"armed":False}))
finally:
    tap.close()

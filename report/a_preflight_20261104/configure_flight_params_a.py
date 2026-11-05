"""Configure only A's failed solo-flight PX4 limits while landed and disarmed."""
import json
import math
from pathlib import Path
import struct
import sys
import time

import backup_parameters_a as backup
from mocap_ground_guard_a import Tap, wait_ground

LOG=Path(sys.argv[1])
OLD={"GF_ACTION":2,"COM_RC_IN_MODE":3,"CBRK_FLIGHTTERM":121212,
     "MPC_TKO_SPEED":1.5,"MPC_LAND_SPEED":0.7,
     "MPC_Z_VEL_MAX_UP":3.0,"MPC_Z_VEL_MAX_DN":1.5}
NEW={"GF_ACTION":4,"COM_RC_IN_MODE":0,"CBRK_FLIGHTTERM":0,
     "MPC_TKO_SPEED":0.3,"MPC_LAND_SPEED":0.3,
     "MPC_Z_VEL_MAX_UP":1.0,"MPC_Z_VEL_MAX_DN":0.5}
INTEGER={"GF_ACTION","COM_RC_IN_MODE","CBRK_FLIGHTTERM"}

def equal(a,b):
    return math.isclose(a,b,rel_tol=1e-5,abs_tol=1e-5)

def set_checked(tap,key,value):
    if key not in NEW or not tap.ground():
        raise RuntimeError("write outside task allowance or ground state lost")
    kind=6 if key in INTEGER else 9
    wire=struct.unpack("<f",struct.pack("<i",value))[0] if kind==6 else float(value)
    for _ in range(2):
        tap.tx.param_set_send(1,1,key.encode("ascii"),wire,kind)
        end=time.monotonic()+1.5
        while time.monotonic()<end:
            tap.receive()
        got,t= tap.read(key)
        if t==kind and equal(got,value):
            return
    raise RuntimeError("PX4 parameter write/readback failed: "+key)

def main():
    if Path("/proc/device-tree/serial-number").read_bytes().rstrip(b"\0").decode()!="1612425621719":
        raise RuntimeError("wrong A board")
    if backup.main()!=0:
        raise RuntimeError("complete baseline parameter snapshot failed")
    snap=json.loads((LOG/"parameters_backup_passive.json").read_text(encoding="utf-8"))
    if not snap["complete"] or snap["version"]["uid"]!=3544984969635313204:
        raise RuntimeError("wrong PX4 or incomplete baseline")
    for key,value in OLD.items():
        row=snap["parameters"][key]
        if row["type"]!=(6 if key in INTEGER else 9) or not equal(row["value"],value):
            raise RuntimeError("baseline changed: "+key)
    (LOG/"flight_parameters_before.json").write_text(json.dumps(snap,indent=2),encoding="utf-8")
    tap=Tap()
    changed=[]
    try:
        wait_ground(tap)
        tap.verify_identity()
        for key,value in OLD.items():
            got,kind=tap.read(key)
            if kind!=(6 if key in INTEGER else 9) or not equal(got,value):
                raise RuntimeError("live baseline changed: "+key)
        for key,value in NEW.items():
            set_checked(tap,key,value)
            changed.append(key)
        for key,value in NEW.items():
            got,kind=tap.read(key)
            if kind!=(6 if key in INTEGER else 9) or not equal(got,value):
                raise RuntimeError("post-write mismatch: "+key)
        result={"passed":True,"parameters":NEW,"flight_commands":0,"armed":False}
        (LOG/"flight_parameters_result.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
        print(json.dumps(result))
    except BaseException:
        if changed and tap.ground():
            for key in reversed(changed):
                got,kind=tap.read(key)
                if kind!=(6 if key in INTEGER else 9) or not (equal(got,OLD[key]) or equal(got,NEW[key])):
                    raise RuntimeError("independent PX4 parameter change; audit: "+key)
                set_checked(tap,key,OLD[key])
        raise
    finally:
        tap.close()

if __name__=="__main__":
    main()

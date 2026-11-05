"""Set A's persistent PX4 indoor mocap parameters while disarmed and landed."""
import json
from pathlib import Path
import struct
import sys
import time

from pymavlink import mavutil
import backup_parameters_a as backup
from mocap_ground_guard_a import Tap, wait_ground

LOG = Path(sys.argv[1])
OLD = {"EKF2_GPS_CTRL": 7, "EKF2_BARO_CTRL": 1, "EKF2_EV_CTRL": 0,
       "EKF2_HGT_REF": 1, "EKF2_MAG_TYPE": 0, "UXRCE_DDS_SYNCT": 1}
NEW = {"EKF2_GPS_CTRL": 0, "EKF2_BARO_CTRL": 0, "EKF2_EV_CTRL": 11,
       "EKF2_HGT_REF": 3, "EKF2_MAG_TYPE": 5, "UXRCE_DDS_SYNCT": 1}


def set_int(tap, key, value):
    if key not in OLD or key == "UXRCE_DDS_SYNCT" or value not in (OLD[key], NEW[key]):
        raise ValueError("parameter write outside task allowlist")
    if not tap.ground():
        raise RuntimeError("fresh landed/disarmed state required")
    wire = struct.unpack("<f", struct.pack("<i", value))[0]
    for _ in range(2):
        tap.tx.param_set_send(1, 1, key.encode("ascii"), wire, 6)
        end = time.monotonic() + 2
        while time.monotonic() < end:
            tap.receive()
        if tap.read(key) == (value, 6):
            return
    raise RuntimeError("parameter write/readback failed: " + key)


def main():
    assert Path("/proc/device-tree/serial-number").read_bytes().rstrip(b"\0").decode() == "1612425621719"
    if backup.main() != 0:
        raise RuntimeError("complete baseline snapshot required")
    baseline = json.loads((LOG / "parameters_backup_passive.json").read_text(encoding="utf-8"))
    if not baseline["complete"] or baseline["version"]["uid"] != 3544984969635313204:
        raise RuntimeError("PX4 identity or snapshot mismatch")
    if any(baseline["parameters"][key]["value"] != val or
           baseline["parameters"][key]["type"] != 6 for key, val in OLD.items()):
        raise RuntimeError("PX4 parameters changed from audited baseline")
    (LOG / "parameters_before.json").write_text(json.dumps(baseline, indent=2), encoding="utf-8")
    tap = Tap()
    changed = False
    try:
        wait_ground(tap)
        tap.verify_identity()
        for key, val in OLD.items():
            if tap.read(key) != (val, 6):
                raise RuntimeError("live parameter changed: " + key)
        for key in ("EKF2_GPS_CTRL", "EKF2_BARO_CTRL", "EKF2_EV_CTRL",
                    "EKF2_HGT_REF", "EKF2_MAG_TYPE"):
            set_int(tap, key, NEW[key])
            changed = True
        tap.reboot_fc("enable_mocap")
        for key, val in NEW.items():
            if tap.read(key) != (val, 6):
                raise RuntimeError("post-reboot parameter mismatch: " + key)
        result = {"passed": True, "px4_mocap_parameters": NEW,
                  "flight_commands": 0, "aircraft_armed": False}
        (LOG / "configuration_result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result))
    except BaseException:
        if changed and tap.ground():
            for key in ("EKF2_GPS_CTRL", "EKF2_BARO_CTRL", "EKF2_EV_CTRL",
                        "EKF2_HGT_REF", "EKF2_MAG_TYPE"):
                current, kind = tap.read(key)
                if kind != 6 or current not in (OLD[key], NEW[key]):
                    raise RuntimeError("independent parameter change; manual audit required: " + key)
                set_int(tap, key, OLD[key])
            tap.reboot_fc("restore_baseline")
        raise
    finally:
        tap.close()


if __name__ == "__main__":
    main()

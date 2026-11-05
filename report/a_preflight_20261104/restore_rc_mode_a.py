"""Restore A's working RC input mode 3 after mode 0 suppressed RC telemetry."""
from pathlib import Path
from mocap_ground_guard_a import Tap, wait_ground
from configure_flight_params_a import set_checked

if Path("/proc/device-tree/serial-number").read_bytes().rstrip(b"\0").decode()!="1612425621719":
    raise RuntimeError("wrong A board")
tap=Tap()
try:
    wait_ground(tap)
    tap.verify_identity()
    if tap.read("COM_RC_IN_MODE")!=(0,6):
        raise RuntimeError("RC mode changed independently")
    set_checked(tap,"COM_RC_IN_MODE",3)
    if tap.read("COM_RC_IN_MODE")!=(3,6):
        raise RuntimeError("RC mode rollback readback failed")
    print("COM_RC_IN_MODE=3 restored; aircraft remained disarmed")
finally:
    tap.close()

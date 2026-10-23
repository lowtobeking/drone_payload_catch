"""Recovery-only: restore the two test parameters with fresh disarmed/identity checks."""
import json
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parent
LOG=Path(sys.argv[1])
LOG.mkdir(parents=True,exist_ok=True)
import fixed_guard_a as guard
class RecoveryTap(guard.Tap):
    def ground(self):
        # Restore only: EV yaw unavailable can prevent landed initialization.
        # No-prop physically grounded state is user-confirmed; still require fresh DISARMED.
        return self.disarmed()
    def receive(self):
        ms=super().receive()
        if time.monotonic()-getattr(self,"last_trace",0)>1:
            self.last_trace=time.monotonic()
            print(json.dumps(dict(boot_ms=self.boot_ms,disarmed=self.disarmed(),
                landed=self.landed,land_age=time.monotonic()-self.land_mono)),flush=True)
        return ms
def main():
    tap=RecoveryTap()
    result=dict(recovery_only=True,parameters_restored=False,restart_verified=False)
    try:
        guard.wait_ground(tap,15);tap.verify_identity()
        result["before"]={k:tap.read(k)[0] for k in ("EKF2_EV_CTRL","EKF2_MAG_TYPE","UXRCE_DDS_SYNCT")}
        if result["before"]["EKF2_EV_CTRL"] not in (0,9) or result["before"]["EKF2_MAG_TYPE"] not in (0,5):
            raise RuntimeError("unexpected independent parameter changes")
        if result["before"]["UXRCE_DDS_SYNCT"]!=1: raise RuntimeError("sync changed independently")
        tap.set_checked("EKF2_EV_CTRL",0);tap.set_checked("EKF2_MAG_TYPE",0)
        result["parameters_restored"]=True
        try:
            tap.reboot_fc("recovery_restore")
            result["restart_verified"]=True
        except RuntimeError as exc:
            result["restart_error"]=repr(exc)
        result["after"]={k:tap.read(k)[0] for k in ("EKF2_EV_CTRL","EKF2_MAG_TYPE","UXRCE_DDS_SYNCT")}
        result["disarmed"]=tap.disarmed()
    except BaseException as exc: result["error"]=repr(exc)
    finally:
        tap.close()
        (LOG/"recovery_result.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
        print(json.dumps(result),flush=True)
    return 0 if result["parameters_restored"] and result.get("after")=={"EKF2_EV_CTRL":0,"EKF2_MAG_TYPE":0,"UXRCE_DDS_SYNCT":1} and result["restart_verified"] else 2
if __name__=="__main__": raise SystemExit(main())

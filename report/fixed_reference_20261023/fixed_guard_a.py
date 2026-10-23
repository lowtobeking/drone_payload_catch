"""Two-parameter bounded ground lease with FC-only reboot and verified rollback."""
import json
import os
from pathlib import Path
import signal
import socket
import struct
import sys
import time
from pymavlink import mavutil
import backup_parameters_a as backup
from receive_parameter_guard_a import Tap as OriginalTap

ROOT=Path(__file__).resolve().parent
LOG=Path(sys.argv[1])
LOG.mkdir(parents=True,exist_ok=True)
stopping=False
def stop(*_):
    global stopping
    stopping=True
for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP): signal.signal(sig,stop)
def save(name,data):
    target=LOG/name
    temp=target.with_suffix(target.suffix+".tmp")
    temp.write_text(json.dumps(data,indent=2,allow_nan=False),encoding="utf-8")
    temp.replace(target)

class Tap(OriginalTap):
    def __init__(self):
        super().__init__()
        self.boot_ms=None
        self.landed=None
        self.land_mono=0.
        self.boot_mono=0.
    def receive(self):
        messages=super().receive()
        for m in messages:
            stamp=getattr(m,"time_boot_ms",None)
            if isinstance(stamp,int) and 0<stamp<2**32:
                self.boot_ms=stamp
                self.boot_mono=time.monotonic()
            if m.get_type()=="EXTENDED_SYS_STATE":
                self.landed=m.landed_state==1
                self.land_mono=time.monotonic()
        return messages
    def ground(self):
        return self.disarmed() and self.landed is True and time.monotonic()-self.land_mono<3.
    def set_checked(self,key,value):
        if key not in ("EKF2_MAG_TYPE","EKF2_EV_CTRL"):
            raise ValueError("write not allowlisted")
        if type(value) is not int or value not in ((0,5) if key=="EKF2_MAG_TYPE" else (0,9)):
            raise ValueError("write value not allowlisted")
        if not self.ground(): raise RuntimeError("parameter write requires fresh disarmed/landed")
        wire=struct.unpack("<f",struct.pack("<i",value))[0]
        for attempt in range(2):
            self.tx.param_set_send(1,1,key.encode("ascii"),wire,6)
            end=time.monotonic()+2
            while time.monotonic()<end:
                for m in self.receive():
                    if m.get_type()=="PARAM_VALUE":
                        name=m.param_id.decode("ascii") if isinstance(m.param_id,bytes) else m.param_id
                        if name.rstrip("\0")==key and m.param_type==6:
                            if backup.decode_parameter(m.param_value,m.param_type,True)==value:
                                if self.read(key)==(value,6): return
        raise RuntimeError("parameter write/readback failed "+key)
    def verify_identity(self):
        self.tx.command_long_send(1,1,512,0,148,0,0,0,0,0,0)
        end=time.monotonic()+5
        while time.monotonic()<end:
            for m in self.receive():
                if m.get_type()=="AUTOPILOT_VERSION":
                    if m.uid!=3544984969635313204 or (int(m.flight_sw_version)>>8)!=0x011000:
                        raise RuntimeError("FC version/UID mismatch")
                    return
        raise RuntimeError("version identity not read back")
    def reboot_fc(self,label):
        if not self.ground() or self.boot_ms is None or time.monotonic()-self.boot_mono>2:
            raise RuntimeError("cannot reboot without fresh ground and FC uptime evidence")
        previous=self.boot_ms
        if previous<10000: raise RuntimeError("FC uptime too short to verify restart")
        save(label+"_reboot_requested.json",dict(previous_boot_ms=previous,wall_us=time.time_ns()//1000,jetson_boot_unchanged=True))
        self.tx.command_long_send(1,1,246,0,1,0,0,0,0,0,0)
        started=time.monotonic()
        end=started+180
        next_request=started+2
        next_trace=started
        next_reopen=time.monotonic()+3
        reset_seen=False
        reset_mono=0.
        while time.monotonic()<end:
            self.receive()
            if (time.monotonic()>=next_reopen and not reset_seen):
                # AF_PACKET may remain silent across carrier/device transitions.
                # Reopen only our own two sockets, preserving host interfaces/services.
                self.close()
                OriginalTap.__init__(self)
                self.boot_ms=None
                self.boot_mono=0.
                self.land_mono=0.
                self.landed=None
                next_reopen=time.monotonic()+3
            now=time.monotonic()
            if now>=next_request:
                self.tx.param_request_read_send(1,1,b"EKF2_MAG_TYPE",-1)
                self.tx.command_long_send(1,1,512,0,148,0,0,0,0,0,0)
                next_request=now+2
            if now>=next_trace:
                print(json.dumps(dict(restart_phase=label,elapsed_s=now-started,
                    boot_ms=self.boot_ms,heartbeat_age_s=now-self.heartbeat_mono,
                    land_age_s=now-self.land_mono,ground=self.ground())),flush=True)
                next_trace=now+5
            if (self.boot_ms is not None and self.boot_ms<previous+(now-started)*1000-5000
                    and self.boot_ms<=(now-started)*1000+3000):
                if not reset_seen:
                    reset_mono=time.monotonic()
                reset_seen=True
            if (reset_seen and self.ground() and self.heartbeat_mono>reset_mono
                    and self.land_mono>reset_mono):
                self.verify_identity()
                save(label+"_reboot_verified.json",dict(old_boot_ms=previous,new_boot_ms=self.boot_ms,
                                                       fc_uid=3544984969635313204,ground=True,elapsed_s=time.monotonic()-started,wall_us=time.time_ns()//1000))
                return
        raise RuntimeError("FC-only reboot not verified")
def wait_ground(tap,seconds=8):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        tap.receive()
        if tap.ground() and tap.boot_ms is not None: return
    raise RuntimeError("fresh disarmed landed FC evidence unavailable")
def proof(tap):
    required={"EKF2_MAG_TYPE":5,"EKF2_EV_CTRL":9,"UXRCE_DDS_SYNCT":1}
    vals={}
    started=time.monotonic()
    deadline=started+1.2
    retry=0.
    while len(vals)<len(required) and time.monotonic()<deadline:
        if time.monotonic()>=retry:
            for key in required:
                if key not in vals: tap.tx.param_request_read_send(1,1,key.encode("ascii"),-1)
            retry=time.monotonic()+.35
        for m in tap.receive():
            if m.get_type()!="PARAM_VALUE": continue
            key=m.param_id.decode("ascii") if isinstance(m.param_id,bytes) else m.param_id
            key=key.rstrip("\\0")
            if key in required:
                value=backup.decode_parameter(m.param_value,m.param_type,True)
                if m.param_type!=6 or value!=required[key]:
                    raise RuntimeError("live parameter policy changed "+key)
                vals[key]=value
    if len(vals)!=3: raise RuntimeError("bounded batched parameter proof incomplete")
    print(json.dumps(dict(parameter_proof_read_seconds=time.monotonic()-started)),flush=True)
    if not tap.ground(): raise RuntimeError("ground state lost")
    save("fixed_parameter_proof.json",dict(schema=1,jetson_serial="1612425621719",
        jetson_boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        fc_uid=3544984969635313204,readback_monotonic_us=int(started*1e6),
        parameters=vals,fc_restart_verified=True))
def main():
    tap=None
    attempted=False
    configured=False
    result=dict(changed=False,restored=False,restore_restart_verified=False,flight_approval=False,
                flight_commands=0,fc_only_reboots=0)
    try:
        if backup.main()!=0: raise RuntimeError("complete parameter backup failed")
        snap=json.loads((ROOT/"parameters_backup_passive.json").read_text(encoding="utf-8"))
        save("parameters_before.json",snap)
        vals=snap["parameters"]
        if not snap["complete"] or snap["version"]["uid"]!=3544984969635313204:
            raise RuntimeError("wrong FC or incomplete snapshot")
        if any(vals[k]["value"]!=v or vals[k]["type"]!=6 for k,v in
               (("EKF2_MAG_TYPE",0),("EKF2_EV_CTRL",0),("UXRCE_DDS_SYNCT",1))):
            raise RuntimeError("requires original MAG0 EV0 SYNCT1")
        tap=Tap(); wait_ground(tap); tap.verify_identity()
        save("backup_ready.json",dict(ready=True,pid=os.getpid(),restore_values={"EKF2_MAG_TYPE":0,"EKF2_EV_CTRL":0}))
        end=time.monotonic()+30
        while not (LOG/"configure.request").exists() and not (LOG/"release.request").exists() and not stopping and time.monotonic()<end:
            tap.receive()
        if stopping or (LOG/"release.request").exists() or not (LOG/"configure.request").exists():
            raise RuntimeError("configuration not requested")
        if not tap.ground(): raise RuntimeError("ground state changed")
        attempted=True; result["changed"]=True
        tap.set_checked("EKF2_EV_CTRL",9)
        tap.set_checked("EKF2_MAG_TYPE",5)
        result["fc_only_reboots"]+=1
        tap.reboot_fc("configure")
        configured=True
        proof(tap)
        save("enabled.json",dict(EKF2_MAG_TYPE=5,EKF2_EV_CTRL=9,max_lease_seconds=80))
        end=time.monotonic()+80
        next_proof=0.
        while not stopping and not (LOG/"release.request").exists() and time.monotonic()<end:
            tap.receive()
            if not tap.ground(): raise RuntimeError("ground telemetry lost during lease")
            if time.monotonic()>=next_proof:
                proof(tap)
                next_proof=time.monotonic()+.45
        result["lease_ended"]=True
    except BaseException as exc:
        result["error"]=repr(exc)
    finally:
        try:
            p=LOG/"fixed_parameter_proof.json"
            if p.exists(): p.unlink()  # Invalidate before starting rollback.
        except OSError as exc: result["proof_cleanup_error"]=repr(exc)
        if attempted and tap is not None:
            for attempt in range(3):
                try:
                    wait_ground(tap,10)
                    # Do not overwrite unexpected independent changes.
                    for key,allowed in (("EKF2_MAG_TYPE",(0,5)),("EKF2_EV_CTRL",(0,9))):
                        current,kind=tap.read(key)
                        if kind!=6 or current not in allowed:
                            raise RuntimeError("parameter independently changed "+key)
                    tap.set_checked("EKF2_EV_CTRL",0)
                    tap.set_checked("EKF2_MAG_TYPE",0)
                    result["fc_only_reboots"]+=1
                    tap.reboot_fc("restore")
                    result["restored"]=all(tap.read(k)==(v,6) for k,v in
                        (("EKF2_EV_CTRL",0),("EKF2_MAG_TYPE",0),("UXRCE_DDS_SYNCT",1)))
                    result["restore_restart_verified"]=True
                    if result["restored"]: break
                except BaseException as exc:
                    result.setdefault("restore_errors",[]).append(repr(exc))
        if tap is not None: tap.close()
        save("parameter_lease_result.json",result)
        print(json.dumps(result),flush=True)
    return 0 if configured and result["restored"] and result["restore_restart_verified"] and "error" not in result else 2
if __name__=="__main__": raise SystemExit(main())

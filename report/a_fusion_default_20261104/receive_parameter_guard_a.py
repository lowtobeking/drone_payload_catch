"""Single-purpose bounded EKF2_EV_CTRL ground-test lease, with autonomous rollback."""
import json
from pathlib import Path
import signal
import socket
import struct
import sys
import time
from pymavlink import mavutil
import backup_parameters_a as backup

ROOT = Path(__file__).resolve().parent
LOG = Path(sys.argv[1])
LOG.mkdir(parents=True, exist_ok=True)
stop = False
def stopping(*_):
    global stop
    stop = True
signal.signal(signal.SIGTERM, stopping)
signal.signal(signal.SIGINT, stopping)
signal.signal(signal.SIGHUP, stopping)

def save(name, data):
    (LOG / name).write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")

class Tap:
    def __init__(self):
        self.sniff = socket.socket(socket.AF_PACKET, socket.SOCK_DGRAM, socket.htons(0x0800))
        self.sniff.bind(("eno1", 0))
        self.sniff.settimeout(.15)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sender.bind(("10.41.10.100", 0))
        self.decoder = mavutil.mavlink.MAVLink(None)
        self.decoder.robust_parsing = True
        self.tx = mavutil.mavlink.MAVLink(self, srcSystem=244, srcComponent=191)
        self.heartbeat = None
        self.heartbeat_mono = 0.
    def write(self, data):
        self.sender.sendto(data, ("10.41.10.2", 14550))
    def receive(self):
        try:
            packet = self.sniff.recv(65535)
        except socket.timeout:
            return []
        if len(packet)<28 or packet[0]>>4!=4 or packet[9]!=17:
            return []
        ihl=(packet[0]&15)*4
        if len(packet)<ihl+8 or socket.inet_ntoa(packet[12:16])!="10.41.10.2":
            return []
        sport,_,length,_=struct.unpack("!HHHH",packet[ihl:ihl+8])
        if sport!=14550 or length<8 or len(packet)<ihl+length:
            return []
        messages=[m for m in (self.decoder.parse_buffer(packet[ihl+8:ihl+length]) or [])
                  if (m.get_srcSystem(),m.get_srcComponent())==(1,1)]
        for m in messages:
            if m.get_type()=="HEARTBEAT":
                self.heartbeat=m
                self.heartbeat_mono=time.monotonic()
        return messages
    def disarmed(self):
        return (self.heartbeat is not None and self.heartbeat.autopilot==12
                and not self.heartbeat.base_mode & 128
                and time.monotonic()-self.heartbeat_mono<2.5)
    def read(self, key):
        self.tx.param_request_read_send(1,1,key.encode("ascii"),-1)
        end=time.monotonic()+4
        while time.monotonic()<end:
            for m in self.receive():
                if m.get_type()!="PARAM_VALUE":
                    continue
                name=m.param_id.decode("ascii") if isinstance(m.param_id,bytes) else m.param_id
                if name.rstrip("\0")==key:
                    return backup.decode_parameter(m.param_value,m.param_type,True),m.param_type
        raise RuntimeError("parameter read timeout: "+key)
    def set_ev(self, value):
        if value not in (0,9):
            raise ValueError("this lease can only set EV_CTRL 0 or 9")
        wire=struct.unpack("<f",struct.pack("<i",value))[0]
        self.tx.param_set_send(1,1,b"EKF2_EV_CTRL",wire,6)
        end=time.monotonic()+4
        while time.monotonic()<end:
            for m in self.receive():
                if m.get_type()=="PARAM_VALUE":
                    name=m.param_id.decode("ascii") if isinstance(m.param_id,bytes) else m.param_id
                    if name.rstrip("\0")=="EKF2_EV_CTRL":
                        decoded=backup.decode_parameter(m.param_value,m.param_type,True)
                        if m.param_type==6 and decoded==value:
                            return
        if self.read("EKF2_EV_CTRL")!=(value,6):
            raise RuntimeError("EV_CTRL write/readback mismatch")
    def close(self):
        self.sniff.close()
        self.sender.close()

def main():
    result={"changed":False,"restored":False,"flight_commands":0,"uxrce_sync_changed":False}
    tap=None
    original=None
    try:
        if backup.main()!=0:
            raise RuntimeError("full parameter backup incomplete")
        snap=json.loads((ROOT/"parameters_backup_passive.json").read_text(encoding="utf-8"))
        save("parameters_before.json",snap)
        if not snap["complete"] or snap["encoding"]!="bytewise" or snap["firmware"][:3]!=[1,16,0]:
            raise RuntimeError("unexpected firmware or incomplete backup")
        values=snap["parameters"]
        original=values["EKF2_EV_CTRL"]["value"]
        if original!=0 or values["EKF2_EV_CTRL"]["type"]!=6 or values["UXRCE_DDS_SYNCT"]["value"]!=1:
            raise RuntimeError("requires original EV_CTRL=0 and unchanged SYNCT=1")
        tap=Tap()
        end=time.monotonic()+5
        while time.monotonic()<end and not tap.disarmed():
            tap.receive()
        if not tap.disarmed():
            raise RuntimeError("fresh disarmed PX4 A heartbeat missing")
        if tap.read("EKF2_EV_CTRL")!=(0,6):
            raise RuntimeError("EV_CTRL changed since backup")
        save("backup_ready.json",{"ready":True,"original_ev_ctrl":0,"watchdog_pid":__import__("os").getpid()})
        end=time.monotonic()+35
        while (not (LOG/"enable.request").exists() and not (LOG/"release.request").exists()
               and time.monotonic()<end and not stop):
            tap.receive()
        if stop or not (LOG/"enable.request").exists() or (LOG/"release.request").exists():
            raise RuntimeError("enable request not received within bounded window")
        if not tap.disarmed():
            raise RuntimeError("disarmed check failed before parameter write")
        if tap.read("UXRCE_DDS_SYNCT")[0]!=1:
            raise RuntimeError("DDS sync must remain enabled")
        result["changed"]=True
        tap.set_ev(9)
        save("enabled.json",{"EV_CTRL":9,"horizontal_position":True,"yaw":True,"vertical":False,
                             "velocity_input":False,"max_lease_seconds":45})
        end=time.monotonic()+45
        while not (LOG/"release.request").exists() and time.monotonic()<end and not stop:
            tap.receive()
            if not tap.disarmed():
                raise RuntimeError("PX4 armed or heartbeat stale during ground lease")
        result["lease_ended"]=True
    except BaseException as exc:
        result["error"]=repr(exc)
    finally:
        if result["changed"] and tap is not None:
            for attempt in range(3):
                try:
                    current,kind=tap.read("EKF2_EV_CTRL")
                    if kind!=6 or current not in (0,9):
                        raise RuntimeError("EV_CTRL independently changed; refusing overwrite")
                    if current!=original:
                        tap.set_ev(original)
                    result["restored"]=tap.read("EKF2_EV_CTRL")== (original,6)
                    if result["restored"]:
                        break
                except BaseException as exc:
                    result["restore_error"]=repr(exc)
            try:
                result["sync_after"]=tap.read("UXRCE_DDS_SYNCT")[0]
            except BaseException as exc:
                result["sync_read_error"]=repr(exc)
        if tap is not None:
            tap.close()
        save("parameter_lease_result.json",result)
        print(json.dumps(result),flush=True)
    return 0 if result["changed"] and result["restored"] and "error" not in result else 2

if __name__=="__main__":
    raise SystemExit(main())

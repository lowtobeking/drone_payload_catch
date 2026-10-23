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
    tap=Tap()
    result={"messages":{}}
    try:
        end=time.monotonic()+4
        while time.monotonic()<end:
            for m in tap.receive():
                result["messages"].setdefault(m.get_type(),{"type":m.get_type(),
                    "time_boot_ms":getattr(m,"time_boot_ms",None)})
        token=time.time_ns()
        tap.tx.timesync_send(0,token)
        tap.tx.command_long_send(1,1,512,0,2,0,0,0,0,0,0)
        end=time.monotonic()+5
        while time.monotonic()<end:
            for m in tap.receive():
                if m.get_type() in ("TIMESYNC","SYSTEM_TIME"):
                    result.setdefault("time_responses",[]).append(m.to_dict())
        result["disarmed"]=tap.disarmed()
    finally:
        tap.close()
    save("fc_time_probe.json",result)
    print(json.dumps(result),flush=True)
if __name__=="__main__": main()

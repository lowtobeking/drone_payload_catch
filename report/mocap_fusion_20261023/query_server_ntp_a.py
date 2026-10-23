"""Read-only UDP NTP query to mocap server. Never changes clock or config."""
import json
from pathlib import Path
import socket
import struct
import time
from datetime import datetime
from zoneinfo import ZoneInfo

assert Path("/proc/device-tree/serial-number").read_text().strip("\0\n") == "1612425621719"
NTP_EPOCH = 2208988800
out = {"read_only": True, "server": "192.168.1.111", "port": 123, "samples": []}
for _ in range(3):
    row = {}
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stream:
            stream.bind(("192.168.1.4", 0))
            stream.settimeout(1.5)
            packet = bytearray(48)
            packet[0] = 0x23
            t1 = time.time()
            sec = int(t1) + NTP_EPOCH
            frac = int((t1-int(t1))*2**32)
            struct.pack_into("!II", packet, 40, sec, frac)
            stream.sendto(packet, ("192.168.1.111", 123))
            data, sender = stream.recvfrom(512)
            t4 = time.time()
            if sender != ("192.168.1.111", 123) or len(data) < 48:
                raise ValueError("unexpected NTP response")
            mode, leap, stratum = data[0] & 7, data[0] >> 6, data[1]
            if mode != 4 or leap == 3 or not 1 <= stratum <= 15 or data[24:32] != packet[40:48]:
                raise ValueError("NTP response not synchronized or not matching request")
            def stamp(offset):
                s,f = struct.unpack_from("!II", data, offset)
                return s-NTP_EPOCH + f/2**32
            t2, t3 = stamp(32), stamp(40)
            delay = (t4-t1)-(t3-t2)
            offset = ((t2-t1)+(t3-t4))/2
            row.update(valid=True, stratum=stratum, delay_s=delay, server_minus_local_s=offset,
                       server_transmit_beijing=datetime.fromtimestamp(t3,ZoneInfo("Asia/Shanghai")).isoformat(),
                       local_receive_beijing=datetime.fromtimestamp(t4,ZoneInfo("Asia/Shanghai")).isoformat())
    except (OSError, ValueError) as exc:
        row.update(valid=False, error=str(exc))
    out["samples"].append(row)
print(json.dumps(out,indent=2),flush=True)

"""Passive post-reconnect FC metadata observation; no commands or payload storage."""
import json
import socket
import struct
import subprocess
import time
from pathlib import Path
packets=[]
with socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(0x0800)) as capture:
    capture.bind(("eno1",0))
    capture.settimeout(1)
    deadline=time.monotonic()+12
    while time.monotonic()<deadline:
        try: frame=capture.recv(4096)
        except socket.timeout: continue
        if len(frame)<42 or frame[12:14]!=b"\x08\x00": continue
        ip=frame[14:]; h=(ip[0]&15)*4
        if ip[9]!=17 or len(ip)<h+8: continue
        source=socket.inet_ntoa(ip[12:16])
        if source=="10.41.10.2":
            ports=struct.unpack("!HH",ip[h:h+4])
            packets.append(dict(source=source,sport=ports[0],dport=ports[1],bytes=len(frame)))
def run(a):
    p=subprocess.run(a,capture_output=True,text=True,timeout=10)
    return dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
print(json.dumps(dict(fc_inbound_udp_12s=len(packets),examples=packets[:5],
    peer=Path("/run/payload-catch-wifi-peer.json").read_text(),
    helper=run(["systemctl","show","payload-catch-wifi-peer","-p","ActiveState","-p","NRestarts","-p","UnitFileState"]),
    fc_neighbor=run(["ip","neigh","show","dev","eno1"]),
    original_processes=run(["ps","-p","1155,1156","-o","pid,lstart,args"]),
    rollback_timer=run(["systemctl","is-active","payload-catch-wifi-peer-rollback.timer"])),indent=2))

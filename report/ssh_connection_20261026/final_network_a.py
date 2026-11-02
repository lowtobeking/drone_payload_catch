"""Final independent network comparison, safe commit, and passive FC UDP observation."""
import hashlib
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
BACKUP=Path("/home/nvidia/payload_catch_network_backup_20261026_v3")
def run(args,check=True):
    p=subprocess.run(args,capture_output=True,text=True,timeout=15,shell=False)
    if check and p.returncode:
        raise RuntimeError(repr(args)+":"+p.stderr)
    return {"exit":p.returncode,"stdout":p.stdout,"stderr":p.stderr}
baseline=json.loads((BACKUP/"baseline.json").read_text())
hashes=baseline["profile_and_unit_hashes"]
changed=[name for name,expected in hashes.items() if not Path(name).is_file() or hashlib.sha256(Path(name).read_bytes()).hexdigest()!=expected]
if changed:
    raise RuntimeError("Original configurations changed: "+str(changed))
before=json.loads(baseline["routes"]["stdout"])
after=json.loads(run(["ip","-j","-4","route","show","table","main"])["stdout"])
def stable(rows):
    return sorted(json.dumps({k:v for k,v in r.items() if k not in ("expires",)},sort_keys=True) for r in rows if not (r.get("dst") in ("192.168.1.4","192.168.1.4/32") and r.get("dev")=="wlP1p1s0" and r.get("metric")==9 and str(r.get("protocol")) in ("186","bgp")))
if stable(before)!=stable(after):
    raise RuntimeError("Unexpected main route change")
services=run(["systemctl","show","payload-catch-agent","payload-catch-relay","ssh","-p","Id","-p","MainPID","-p","ExecMainStartTimestamp"])
if services!=baseline["services"]:
    raise RuntimeError("Original services restarted or changed")
# Passive socket capture: observe flight-controller packets, send no commands.
packets=[]
with socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(0x0800)) as capture:
    capture.bind(("eno1",0))
    capture.settimeout(1)
    deadline=time.monotonic()+6
    while time.monotonic()<deadline:
        try:
            frame=capture.recv(4096)
        except socket.timeout:
            continue
        if len(frame)<42 or frame[12:14]!=b"\x08\x00":
            continue
        ip=frame[14:]
        ihl=(ip[0]&15)*4
        if ip[9]!=17 or len(ip)<ihl+8:
            continue
        source=socket.inet_ntoa(ip[12:16])
        sport,dport=struct.unpack("!HH",ip[ihl:ihl+4])
        if source=="10.41.10.2":
            packets.append(dict(source=source,sport=sport,dport=dport,bytes=len(frame)))
    # only aggregate metadata; no payload persisted
data={
"original_config_hashes_unchanged":len(hashes),
"original_routes_unchanged":True,
"original_services_unchanged":True,
"serial":Path("/proc/device-tree/serial-number").read_bytes().rstrip(bytes([0])).decode(),
"boot_id":Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
"fc_link":run(["ip","-j","link","show","dev","eno1"]),
"fc_neighbor":run(["ip","-j","neigh","show","dev","eno1"]),
"fc_inbound_udp_count_6s":len(packets),
"fc_inbound_udp_examples":packets[:5],
"wifi_state":Path("/run/payload-catch-wifi-peer.json").read_text(),
"helper_status":run(["systemctl","show","payload-catch-wifi-peer","-p","MainPID","-p","NRestarts","-p","ActiveState"]),
"commit":run(["sudo","-n","/usr/bin/python3",str(BACKUP/"transaction.py"),"commit"]),
}
print(json.dumps(data,indent=2))

"""Bounded VRPN server TCP probe from this laptop."""
from pathlib import Path
import json
import socket
result={"server":"192.168.1.111","port":3883}
try:
    with socket.create_connection(("192.168.1.111",3883),timeout=4) as sock:
        result["connected"]=True
        result["source"]=sock.getsockname()
        sock.settimeout(2)
        try:result["server_greeting"]=repr(sock.recv(128))
        except socket.timeout:result["server_greeting"]="none within 2s"
except OSError as error:
    result.update(connected=False,error=str(error))
(Path(__file__).resolve().parent/"windows_server_probe.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
print(json.dumps(result,indent=2))

"""Windows per-socket interface test. Does not change routes or adapters."""
import json
from pathlib import Path
import socket
import struct
import time

WORK = Path(__file__).resolve().parent
results = []
for index, local, target in [(23, "192.168.1.2", "192.168.1.4"),
                              (8, "192.168.1.123", "192.168.1.101"),
                              (8, "192.168.1.123", "192.168.1.4")]:
    row = dict(interface_index=index, local=local, target=target, port=22)
    begin = time.monotonic()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as stream:
            stream.settimeout(5)
            # Windows IP_UNICAST_IF requires a network-byte-order interface DWORD.
            stream.setsockopt(socket.IPPROTO_IP, 31, struct.pack("!I", index))
            stream.bind((local, 0))
            stream.connect((target, 22))
            row["connected"] = True
            row["banner"] = stream.recv(256).decode("ascii", errors="replace")
    except OSError as exc:
        row.update(connected=False, error=str(exc))
    row["elapsed_s"] = time.monotonic() - begin
    results.append(row)
result = dict(read_only_network_configuration=True, probes=results)
(WORK / "per_interface_probe.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
print(json.dumps(result, indent=2))

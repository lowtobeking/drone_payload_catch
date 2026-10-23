"""Read-only reconnection evidence; no restarts, cleanup actions or config writes."""
from pathlib import Path
import json
import os
import subprocess
serial = Path("/proc/device-tree/serial-number").read_text().strip("\0\n")
assert serial == "1612425621719"
out = {"serial": serial, "boot": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
       "uptime_seconds": float(Path("/proc/uptime").read_text().split()[0]),
       "related_processes": []}
for path in Path("/proc").iterdir():
    if not path.name.isdigit() or int(path.name) == os.getpid():
        continue
    try:
        argv = (path/"cmdline").read_bytes().replace(b"\0",b" ").decode("utf-8",errors="replace")
        if any(x in argv for x in ("planeA_stream", "probe_wire_time", "mocap_odometry_node",
                                   "mocap_source_node", "diagnostic_runtime")):
            out["related_processes"].append({"pid": int(path.name), "argv": argv})
    except OSError:
        pass
for name, args in [
    ("boot_time", ["uptime", "-s"]),
    ("ip_addresses", ["ip", "-brief", "address"]),
    ("routes", ["ip", "route"]),
    ("services", ["systemctl", "show", "ssh.service", "payload-catch-agent.service",
                   "payload-catch-mav-relay.service", "-p", "Id", "-p", "ActiveState"]),
    ("boot_history", ["journalctl", "--list-boots", "--no-pager", "-n", "3"])]:
    p = subprocess.run(args, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", timeout=8, shell=False)
    out[name] = {"exit": p.returncode, "stdout": p.stdout, "stderr": p.stderr}
print(json.dumps(out,indent=2),flush=True)

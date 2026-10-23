"""Read-only final audit; do not stop any process or service."""
from pathlib import Path
import json
import os
import subprocess

serial = Path("/proc/device-tree/serial-number").read_text().strip("\0\n")
assert serial == "1612425621719"
owned = []
prefix = "/home/nvidia/payload_catch_mocap_fusion_20261023/"
for path in Path("/proc").iterdir():
    if not path.name.isdigit() or int(path.name) == os.getpid():
        continue
    try:
        args = (path / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", errors="replace")
        if prefix in args:
            owned.append({"pid": int(path.name), "args": args})
    except (OSError, PermissionError):
        pass
proc = subprocess.run(["systemctl", "show", "payload-catch-agent.service",
                       "payload-catch-mav-relay.service", "-p", "Id", "-p", "ActiveState", "-p", "MainPID"],
                      capture_output=True, text=True, encoding="utf-8", errors="replace",
                      timeout=10, shell=False)
if proc.returncode:
    raise RuntimeError(proc.stderr)
result = {"read_only": True, "serial": serial,
          "boot": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
          "current": str(Path("/home/nvidia/payload_catch_deploy/current").resolve()),
          "remaining_task_processes": owned, "services": proc.stdout}
print(json.dumps(result, indent=2), flush=True)

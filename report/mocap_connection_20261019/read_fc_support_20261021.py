from pathlib import Path
import sys
root=Path("/home/nvidia/payload_catch_deploy/current/src")
for path in [Path("/opt/payload_catch/managed_20261016/mavlink_relay.py"),root/"payload_catch/solo_validation.py",root/"tools/live_probe.py"]:
    print("FILE",str(path),flush=True)
    lines=path.read_text(encoding="utf-8").splitlines()
    if path.name=="solo_validation.py":
        for n,line in enumerate(lines):
            if line.startswith("def preflight_errors") or line.startswith("def parameter_issues"):
                print("\n".join(lines[n:n+130]),flush=True)
    else:
        print("\n".join(lines[:180]),flush=True)

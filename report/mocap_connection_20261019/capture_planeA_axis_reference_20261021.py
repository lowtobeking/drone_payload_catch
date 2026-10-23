"""Read-only VRPN snapshot on A; save local axis-check evidence."""
from pathlib import Path
from datetime import datetime, timezone
import json
import math
import subprocess
import sys

work = Path(__file__).resolve().parent
stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
out = work / ("planeA_axis_baseline_" + stamp + ".json")
args = [
    r"C:/Windows/System32/OpenSSH/ssh.exe",
    "-o", "BatchMode=yes", "-o", "ConnectTimeout=6",
    "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2",
    "-o", "StrictHostKeyChecking=yes",
    "-o", "HostKeyAlias=192.168.155.186",
    "-i", str(Path.home() / ".ssh/id_ed25519"),
    "nvidia@192.168.1.4",
    "/home/nvidia/payload_catch_a_solo_20261019/mocap_check_20261021/planeA_probe",
    "192.168.1.4",
]
started = datetime.now(timezone.utc).isoformat()
p = subprocess.run(args, cwd=work, timeout=30, capture_output=True,
                   text=True, encoding="utf-8", errors="replace", shell=False)
out.with_suffix(".stdout.log").write_text(p.stdout, encoding="utf-8")
out.with_suffix(".stderr.log").write_text(p.stderr, encoding="utf-8")
if p.returncode:
    sys.exit("VRPN probe exit=" + str(p.returncode) + "\n" + p.stderr)
sample = json.loads(p.stdout)
position = sample.get("position", [])
q = sample.get("quaternion_xyzw", [])
if (sample.get("tracker") != "planeA" or not sample.get("connected")
        or sample.get("count", 0) < 2 or sample.get("invalid", 0)
        or len(position) != 3 or len(q) != 4
        or not all(math.isfinite(v) for v in position + q)
        or sample.get("age_s", 99) > .2
        or abs(sum(v*v for v in q) - 1) > .02):
    sys.exit("Invalid reference snapshot; see saved raw logs")
result = dict(
    kind="planeA_axis_check_baseline",
    started_at_utc=started, completed_at_utc=datetime.now(timezone.utc).isoformat(),
    source="A Jetson native VRPN probe; 192.168.1.111:3883",
    coordinates="raw motion-capture world; axis directions not yet established",
    quaternion_order="xyzw",
    statistic="latest sample after ~10 seconds, not sample mean",
    operator_request="record current reference before manual movement",
    position_m=position, quaternion_xyzw=q,
    forwarded_to_px4=False, raw_probe=sample,
)
with out.open("x", encoding="utf-8") as f:
    json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False)
    f.write("\n")
print(json.dumps(result, ensure_ascii=False, indent=2))
print("SAVED", out)

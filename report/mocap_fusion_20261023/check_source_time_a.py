"""Read-only clock comparison. Run bounded planeA reader, no FC connection."""
from datetime import datetime
from zoneinfo import ZoneInfo
import json
from pathlib import Path
import statistics
import subprocess
import time

serial = Path("/proc/device-tree/serial-number").read_text().strip("\0\n")
if serial != "1612425621719":
    raise RuntimeError("not A")
beijing = ZoneInfo("Asia/Shanghai")
def formatted(us):
    return datetime.fromtimestamp(us / 1e6, beijing).isoformat()
reader = "/home/nvidia/payload_catch_mocap_preview_20261021/9695526c87f980d0/planeA_stream"
child = None
stdout = stderr = b""
try:
    child = subprocess.Popen([reader, "192.168.1.4"], stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, shell=False)
    try:
        stdout, stderr = child.communicate(timeout=6)
    except subprocess.TimeoutExpired:
        child.terminate()
        stdout, stderr = child.communicate(timeout=3)
finally:
    if child is not None and child.poll() is None:
        child.kill()
        stdout, stderr = child.communicate(timeout=3)
now_us = time.time_ns() // 1000
rows = [json.loads(line) for line in stdout.decode("utf-8").splitlines() if line]
result = dict(read_only=True, serial=serial, jetson_now_beijing=formatted(now_us),
              frames=len(rows), reader_pid=child.pid, reader_exit=child.returncode,
              reader_stderr=stderr.decode("utf-8", errors="replace"),
              flight_parameters_changed=False, system_clock_changed=False)
if rows:
    last = int(rows[-1]["server_stamp_us"])
    first = int(rows[0]["server_stamp_us"])
    result.update(source_first_beijing=formatted(first), source_last_beijing=formatted(last),
                  source_last_minus_jetson_end_s=(last-now_us)/1e6,
                  source_span_s=(last-first)/1e6,
                  nonincreasing=sum(b["server_stamp_us"] <= a["server_stamp_us"]
                                    for a,b in zip(rows, rows[1:])))
for name, args in [
    ("timedatectl", ["timedatectl", "show", "-p", "Timezone", "-p", "NTPSynchronized", "-p", "LocalRTC"]),
    ("chrony_tracking", ["chronyc", "tracking"])]:
    p = subprocess.run(args, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", timeout=5, shell=False)
    result[name] = dict(exit_code=p.returncode, stdout=p.stdout, stderr=p.stderr)
print(json.dumps(result, indent=2, ensure_ascii=False), flush=True)

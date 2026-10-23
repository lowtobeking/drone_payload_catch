from pathlib import Path
import sys
from attempt_wifi import ssh
work=Path(__file__).resolve().parent
path="/home/nvidia/payload_catch_a_solo_20261019/mocap_check/sample_vrpn.sh"
code="from pathlib import Path; p=Path("+repr(path)+"); p.write_bytes(p.read_bytes().replace(bytes([13,10]),bytes([10])))"
p=ssh("192.168.1.4",["/usr/bin/python3","-c",code])
if p.returncode:raise SystemExit(p.stderr)
p=ssh("192.168.1.4",["/bin/bash","-n",path])
if p.returncode:raise SystemExit(p.stderr)
p=ssh("192.168.1.4",["/bin/bash",path],timeout=50)
(work/"raw_vrpn_sample.stdout.log").write_text(p.stdout,encoding="utf-8")
(work/"raw_vrpn_sample.stderr.log").write_text(p.stderr,encoding="utf-8")
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
print(p.stdout)
print(p.stderr,file=sys.stderr)
r=ssh("192.168.1.4",["cat","/home/nvidia/payload_catch_a_solo_20261019/mocap_check/raw_vrpn_result.json"])
if r.returncode==0:
    (work/"raw_vrpn_result.json").write_text(r.stdout,encoding="utf-8")
raise SystemExit(p.returncode)

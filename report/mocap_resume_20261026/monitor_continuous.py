"""Read-only monitor for the manual continuous ground session."""
import ast
import json
from pathlib import Path
import subprocess
WORK=Path(__file__).resolve().parent.parent/"mocap_continuous_20261026"
run=sorted(WORK.glob("lift_reference_20*"))[-1]
manifest=json.loads((run/"manifest.json").read_text(encoding="utf-8"))
remote=manifest["remote_logs"]
code="""from pathlib import Path
import json
root=Path(%r)
out={}
for name in ["backup_ready.json","configure_reboot_verified.json","enabled.json","baseline.json","live_status.json","parameter_lease_result.json","validation.json","process_cleanup.json"]:
    p=root/name
    if p.exists():
        out[name]=json.loads(p.read_text())
p=root/"guard.log"
out["guard_tail"]=p.read_text().splitlines()[-3:] if p.exists() else []
p=root/"source/reader_cleanup.json"
if p.exists(): out["reader_cleanup"]=json.loads(p.read_text())
print(json.dumps(out,indent=2))
""" % remote
ast.parse(code)
ssh=["C:/Windows/System32/OpenSSH/ssh.exe","-F","NUL","-b","192.168.1.4","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","HostKeyAlias=192.168.155.186","-o","StrictHostKeyChecking=yes","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.5","/usr/bin/python3 -"]
proc=subprocess.run(ssh,input=code.encode(),cwd=WORK,capture_output=True,timeout=20,shell=False)
(run/"monitor.stdout.log").write_bytes(proc.stdout)
(run/"monitor.stderr.log").write_bytes(proc.stderr)
print(proc.stdout.decode("utf-8",errors="replace"))
print(proc.stderr.decode("utf-8",errors="replace"))
raise SystemExit(proc.returncode)

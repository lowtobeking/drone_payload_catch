"""Post-battery read-only identity/parameter snapshot using the currently reachable route."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
WORK=Path(__file__).resolve().parent
OLD=ROOT/"report/mocap_fusion_20261023"
SSH=[r"C:/Windows/System32/OpenSSH/ssh.exe","-b","192.168.1.123",
     "-o","BatchMode=yes","-o","ConnectTimeout=6","-o","StrictHostKeyChecking=yes",
     "-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.4"]
def main():
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    data=(OLD/"reconnect_readonly_a.py").read_bytes()
    ast.parse(data)
    p=subprocess.run(SSH+["/usr/bin/python3 -"],input=data,cwd=WORK,timeout=45,capture_output=True,shell=False)
    (WORK/"reconnect.stdout.log").write_bytes(p.stdout)
    (WORK/"reconnect.stderr.log").write_bytes(p.stderr)
    print(p.stdout.decode("utf-8",errors="replace"),flush=True)
    if p.returncode:
        print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr)
        raise SystemExit(p.returncode)
    identity=json.loads(p.stdout)
    (WORK/"identity.json").write_bytes(p.stdout)
    assert identity["serial"]=="1612425621719" and not identity["related_processes"]
    spec=importlib.util.spec_from_file_location("backup_runner",OLD/"run_parameter_backup.py")
    runner=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.SSH=SSH
    runner.WORK=WORK
    (WORK/"backup_parameters_a.py").write_bytes((OLD/"backup_parameters_a.py").read_bytes())
    runner.main()
if __name__=="__main__": main()
